// made by AI

#pragma once

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <string>
#include <unordered_map>

// A standalone, real-time line-plot window: its own GLFW window and OpenGL
// context, separate from any MuJoCo viewer window (e.g. UIctr's) -- but
// rendered with MuJoCo's own mjvFigure/mjr_figure, the same font/legend/axis
// style as UIctr::initSensorFigure()/updateSensorFigure()'s touch-sensor
// plot, rather than a hand-drawn font.
//
// Like UIctr's sensor figure, this needs an mjModel* purely so
// mjr_makeContext() can set up its font/rendering resources -- no 3D
// scene/camera is created or needed, and this class does not take ownership
// of the model (the caller keeps owning/freeing it, same as it already does
// for its own mjModel/mjData/UIctr).
//
// Designed so you never need to pre-declare which signals you'll plot --
// just call addPoint() with a new name and a line for it is created
// automatically (auto-colored from a fixed palette), so you can add new
// signals later without touching this class or its constructor call.
//
// Usage:
//   RealtimePlot plot(mj_model, 500, 400, "My Plot");
//   ...
//   while (running) {
//       plot.addPoint("base height", simTime, actualHeight);
//       plot.addPoint("target",      simTime, targetHeight);
//       plot.render();      // once per rendered frame
//   }
//   glfwPollEvents();       // once per frame total, same as for any other
//                           // GLFW window in your program -- not called by
//                           // this class, since it's a process-wide GLFW
//                           // call, not specific to this one window
class RealtimePlot
{
public:
    // timeWindowSeconds sets a "paged" x-axis: fixed-width windows
    // [k*timeWindowSeconds, (k+1)*timeWindowSeconds) that jump forward once
    // the newest data crosses the current page's edge, rather than
    // continuously sliding/growing. E.g. with 5.0, the axis stays exactly
    // [0,5] for the whole first 5 seconds (even before 5s of data actually
    // exists yet), then snaps to [5,10], [10,15], etc. Data from the
    // previous page is dropped once a new page starts. <= 0 means unbounded:
    // the axis grows to show the full history buffered so far (up to
    // mjMAXLINEPNT points, MuJoCo's own hard per-line cap).
    RealtimePlot(mjModel *model, int width, int height, const char *title,
                 double timeWindowSeconds = 10.0);
    ~RealtimePlot();

    RealtimePlot(const RealtimePlot &) = delete;
    RealtimePlot &operator=(const RealtimePlot &) = delete;

    // Appends one data point to the named signal. The first call with a new
    // name creates that line automatically, auto-colored from a fixed
    // palette (cycling if you add more signals than the palette/mjMAXLINE
    // allow).
    void addPoint(const std::string &signalName, double time, double value);

    // Change the displayed/retained time window at any point, e.g. to zoom
    // in/out on the fly. No effect while a manual setXLimit() is active --
    // see setXLimitAuto() to go back to paging.
    void setTimeWindow(double timeWindowSeconds) { timeWindowSeconds_ = timeWindowSeconds; }
    double timeWindow() const { return timeWindowSeconds_; }

    // ---- matplotlib-style figure/axis adjustments -----------------------
    // mjvFigure (MuJoCo's 2D plot struct, what this class renders through)
    // is more limited than matplotlib: no separate y-axis label widget (only
    // title + x-label + per-line legend names exist), and no per-line
    // dash/marker styles -- only a single figure-wide line width and a
    // connected-line vs. isolated-segments toggle. The API below exposes
    // exactly what mjvFigure actually supports, honestly named.

    void setXLabel(const std::string &label);

    // mjvFigure has no native y-axis label field -- folded into the title
    // as "<title>  (<label>)" instead, the only place free text can go.
    void setYLabel(const std::string &label);

    // Manually fix the x-axis range, overriding the automatic paged window
    // (see the constructor's timeWindowSeconds) until setXLimitAuto() is
    // called to hand control back to it.
    void setXLimit(double xMin, double xMax);
    void setXLimitAuto();

    // Set the y-axis range. autoExtend=true (default) keeps this only as a
    // *baseline*: mjvFigure will still grow the range automatically if the
    // data goes beyond it, rather than clipping -- matches "auto-scale if
    // the value goes over the limit". Pass autoExtend=false for a hard
    // range that never grows regardless of data.
    void setYLimit(double yMin, double yMax, bool autoExtend = true);
    void setYLimitAuto(); // fully automatic range with no baseline (the default)

    // Figure-wide line width (mjvFigure has no per-line width, only one
    // shared value for every signal in the figure).
    void setLineWidth(float width);

    // "line" (default): each signal drawn as a connected line strip.
    // "segments": each signal drawn as isolated 2-point segments
    // (mjvFigure's own flg_barplot, despite the name -- see mjvisualize.h:
    // "isolated line segments (i.e. GL_LINES)", not an actual bar chart).
    enum class PlotStyle { Line, Segments };
    void setPlotStyle(PlotStyle style);

    // Makes this window's context current, draws all signals via
    // mjr_figure(), and swaps buffers. Call once per rendered frame. No-op
    // if window/context creation failed or the window has been closed.
    void render();

    bool shouldClose() const;
    bool isValid() const { return window_ != nullptr; }

private:
    GLFWwindow *window_;
    mjrContext con_;
    mjvFigure fig_;
    double timeWindowSeconds_;
    bool xRangeManual_;
    std::string title_, yLabel_;
    std::unordered_map<std::string, int> nameToLineIndex_;
    int numLines_;

    int getOrCreateLine(const std::string &name);
    void refreshTitle(); // recomposes fig_.title from title_ + yLabel_
};
