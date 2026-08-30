#include "RealtimePlot.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {

// fixed, visually-distinguishable palette; cycles if more signals are added
// than colors here
const float kPalette[][3] = {
    {0.20f, 0.90f, 0.30f}, // green
    {0.95f, 0.60f, 0.15f}, // orange
    {0.30f, 0.60f, 0.95f}, // blue
    {0.90f, 0.25f, 0.25f}, // red
    {0.85f, 0.85f, 0.20f}, // yellow
    {0.75f, 0.30f, 0.90f}, // purple
    {0.20f, 0.85f, 0.85f}, // cyan
    {0.90f, 0.90f, 0.90f}, // white
};
constexpr int kPaletteSize = sizeof(kPalette) / sizeof(kPalette[0]);

// GLFW's error callback is process-wide, not per-window; the first attempt
// at this class had none, so a glfwCreateWindow() failure was completely
// silent -- exactly matching "window doesn't appear, no error either".
// Installed once, the first time any RealtimePlot is made.
void glfwErrorCallback(int error, const char *description)
{
    std::fprintf(stderr, "GLFW error %d: %s\n", error, description);
}

} // namespace

RealtimePlot::RealtimePlot(mjModel *model, int width, int height, const char *title, double timeWindowSeconds)
    : window_(nullptr), con_(), fig_(), timeWindowSeconds_(timeWindowSeconds), xRangeManual_(false),
      title_(title), numLines_(0)
{
    static bool errorCallbackInstalled = false;
    if (!errorCallbackInstalled)
    {
        glfwSetErrorCallback(glfwErrorCallback);
        errorCallbackInstalled = true;
    }

    // Deliberately no GLFW_OPENGL_PROFILE hint: that hint is only valid
    // alongside an explicitly requested OpenGL >= 3.2 context (GLFW error
    // "Context profiles are only defined for OpenGL version 3.2 and above"),
    // and requesting it without a matching version makes glfwCreateWindow()
    // fail outright -- which it did, silently, the first time this class was
    // written, since nothing checked its return value. UIctr's own MuJoCo
    // window (sim_interface/GLFW_callbacks.cpp) never sets any window hints
    // either, so we take the same untouched defaults here.
    glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
    window_ = glfwCreateWindow(width, height, title, nullptr, nullptr);
    glfwDefaultWindowHints();

    if (!window_)
    {
        std::fprintf(stderr, "RealtimePlot: glfwCreateWindow(\"%s\") failed -- "
                              "see the GLFW error above for why.\n", title);
        return;
    }

    // some window managers create new windows without raising/focusing them,
    // making them easy to miss behind an already-open window
    glfwShowWindow(window_);
    glfwFocusWindow(window_);

    // mjr_makeContext sets up MuJoCo's GPU rendering resources (including
    // the font atlas mjr_figure's text/legend/axis labels use) on whichever
    // GL context is current -- make this window's context current first so
    // it gets its own, independent of UIctr's.
    glfwMakeContextCurrent(window_);
    // Plot windows don't need monitor-sync pacing -- only the main 3D view
    // does. Each RealtimePlot owns a separate GL context, and its swap
    // interval otherwise falls back to the driver default (often vsync-on
    // too); with everything rendered from one thread, N vsync'd windows'
    // glfwSwapBuffers() calls stack sequentially (up to N frame-waits per
    // loop iteration), so more plot windows made the whole app laggier.
    glfwSwapInterval(0);
    mjr_defaultContext(&con_);
    mjr_makeContext(model, &con_, mjFONTSCALE_150);

    mjv_defaultFigure(&fig_);
    fig_.flg_legend = 1;
    fig_.flg_extend = 1; // auto-grow y-axis to fit data
    refreshTitle();
    std::snprintf(fig_.xlabel, sizeof(fig_.xlabel), "time (s)");
    fig_.range[0][0] = 0;
    fig_.range[0][1] = 1; // updated per addPoint() call below, to a sliding window
    fig_.range[1][0] = 0;
    fig_.range[1][1] = 0; // (min>=max) -> automatic y-axis range
}

RealtimePlot::~RealtimePlot()
{
    if (window_)
    {
        glfwMakeContextCurrent(window_);
        mjr_freeContext(&con_);
        glfwDestroyWindow(window_);
    }
}

void RealtimePlot::refreshTitle()
{
    if (yLabel_.empty())
        std::snprintf(fig_.title, sizeof(fig_.title), "%s", title_.c_str());
    else
        std::snprintf(fig_.title, sizeof(fig_.title), "%s  (%s)", title_.c_str(), yLabel_.c_str());
}

void RealtimePlot::setXLabel(const std::string &label)
{
    if (!window_)
        return;
    std::snprintf(fig_.xlabel, sizeof(fig_.xlabel), "%s", label.c_str());
}

void RealtimePlot::setYLabel(const std::string &label)
{
    if (!window_)
        return;
    yLabel_ = label;
    refreshTitle();
}

void RealtimePlot::setXLimit(double xMin, double xMax)
{
    if (!window_)
        return;
    xRangeManual_ = true;
    fig_.range[0][0] = (float)xMin;
    fig_.range[0][1] = (float)xMax;
}

void RealtimePlot::setXLimitAuto()
{
    xRangeManual_ = false; // addPoint() resumes driving fig_.range[0] next call
}

void RealtimePlot::setYLimit(double yMin, double yMax, bool autoExtend)
{
    if (!window_)
        return;
    fig_.range[1][0] = (float)yMin;
    fig_.range[1][1] = (float)yMax;
    fig_.flg_extend = autoExtend ? 1 : 0;
}

void RealtimePlot::setYLimitAuto()
{
    if (!window_)
        return;
    fig_.range[1][0] = 0;
    fig_.range[1][1] = 0; // (min>=max) -> automatic, per mjvFigure's own convention
    fig_.flg_extend = 1;
}

void RealtimePlot::setLineWidth(float width)
{
    if (!window_)
        return;
    fig_.linewidth = width;
}

void RealtimePlot::setPlotStyle(PlotStyle style)
{
    if (!window_)
        return;
    fig_.flg_barplot = (style == PlotStyle::Segments) ? 1 : 0;
}

int RealtimePlot::getOrCreateLine(const std::string &name)
{
    auto it = nameToLineIndex_.find(name);
    if (it != nameToLineIndex_.end())
        return it->second;

    if (numLines_ >= mjMAXLINE)
    {
        std::fprintf(stderr, "RealtimePlot: mjMAXLINE (%d) signals already in use, "
                              "ignoring new signal \"%s\"\n", mjMAXLINE, name.c_str());
        return -1;
    }

    int idx = numLines_++;
    std::snprintf(fig_.linename[idx], sizeof(fig_.linename[idx]), "%s", name.c_str());
    int paletteIdx = idx % kPaletteSize;
    fig_.linergb[idx][0] = kPalette[paletteIdx][0];
    fig_.linergb[idx][1] = kPalette[paletteIdx][1];
    fig_.linergb[idx][2] = kPalette[paletteIdx][2];
    fig_.linepnt[idx] = 0;

    nameToLineIndex_[name] = idx;
    return idx;
}

void RealtimePlot::addPoint(const std::string &signalName, double time, double value)
{
    if (!window_)
        return;

    int idx = getOrCreateLine(signalName);
    if (idx < 0)
        return;

    // Reject non-increasing timestamps for this signal: e.g. a caller whose
    // own loop has stopped advancing time (finished, or paused) but keeps
    // calling addPoint() every frame anyway would otherwise flood the fixed-
    // size buffer with duplicate-timestamp points, pushing genuine history
    // out and collapsing the visible time range toward that one frozen
    // instant -- exactly what happened here before this check existed.
    if (fig_.linepnt[idx] > 0 && time <= fig_.linedata[idx][0])
        return;

    int pnt = mjMIN(mjMAXLINEPNT, fig_.linepnt[idx] + 1);
    // shift older points back to make room for the new one at index 0
    // (points are stored newest-first, same convention as
    // UIctr::updateSensorFigure)
    for (int i = pnt - 1; i > 0; i--)
    {
        fig_.linedata[idx][2 * i] = fig_.linedata[idx][2 * i - 2];
        fig_.linedata[idx][2 * i + 1] = fig_.linedata[idx][2 * i - 1];
    }
    fig_.linepnt[idx] = pnt;
    fig_.linedata[idx][0] = time;
    fig_.linedata[idx][1] = value;

    // Everything below drives fig_.range[0] (the x-axis) automatically --
    // skipped entirely once setXLimit() has put the axis under manual
    // control, until setXLimitAuto() hands it back. In manual mode, points
    // simply accumulate up to mjMAXLINEPNT (the shift loop's own cap above).
    if (xRangeManual_)
        return;

    // overall latest time across all lines (this signal's new point included)
    // -- the page boundary below is computed from this shared value, not
    // just this one signal's own time, so multiple signals fed at slightly
    // different moments still land on the same page together
    double latest = time;
    for (int n = 0; n < numLines_; n++)
        if (fig_.linepnt[n] > 0)
            latest = std::max(latest, (double)fig_.linedata[n][0]);

    if (timeWindowSeconds_ > 0.0)
    {
        // Paged x-axis: fixed-width windows [k*W, (k+1)*W) that jump forward
        // once the newest data crosses the current page's edge, instead of
        // continuously sliding/growing. E.g. with a 5s window, the axis
        // stays exactly [0,5] for the whole first 5 seconds -- even before
        // 5s of data actually exists yet -- then snaps to [5,10], [10,15],
        // etc., rather than always showing [0, latest].
        double pageStart = std::floor(latest / timeWindowSeconds_) * timeWindowSeconds_;
        fig_.range[0][0] = (float)pageStart;
        fig_.range[0][1] = (float)(pageStart + timeWindowSeconds_);

        // drop each line's points from before this page -- previous-page
        // data isn't visible in the new range, no reason to keep buffering it
        for (int n = 0; n < numLines_; n++)
        {
            int kept = 0;
            while (kept < fig_.linepnt[n] && fig_.linedata[n][2 * kept] >= pageStart)
                kept++;
            fig_.linepnt[n] = kept;
        }
    }
    else
    {
        // unbounded: axis grows to show the full history buffered so far
        float xMin = 1e30f, xMax = -1e30f;
        for (int n = 0; n < numLines_; n++)
        {
            if (fig_.linepnt[n] == 0)
                continue;
            xMax = std::max(xMax, fig_.linedata[n][0]);
            xMin = std::min(xMin, fig_.linedata[n][2 * (fig_.linepnt[n] - 1)]);
        }
        if (xMax > xMin)
        {
            fig_.range[0][0] = xMin;
            fig_.range[0][1] = xMax;
        }
    }
}

bool RealtimePlot::shouldClose() const
{
    return window_ && glfwWindowShouldClose(window_);
}

void RealtimePlot::render()
{
    if (!window_ || glfwWindowShouldClose(window_) || numLines_ == 0)
        return;

    glfwMakeContextCurrent(window_);

    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
    if (viewport.width <= 0 || viewport.height <= 0)
        return;

    glClearColor(0.10f, 0.10f, 0.12f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    mjr_figure(viewport, &fig_, &con_);

    glfwSwapBuffers(window_);
}
