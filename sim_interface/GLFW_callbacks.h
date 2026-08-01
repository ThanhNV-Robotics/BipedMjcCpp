#pragma once
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <string>
#include <memory>

// Raw-GLFW MuJoCo viewer, single-threaded like OpenLoong-Dyn-Control's
// UIctr: call updateScene() once per iteration from your own physics loop,
// on the same thread (no mj::Simulate, no second thread required).
//
// Extends OpenLoong's original UIctr with mouse perturbation, matching the
// interaction MuJoCo's own simulate.cc offers:
//   - double left-click a body to select it for perturbation
//   - double right-click empty space/a body to move the camera's look-at
//     point there; hold Ctrl while doing so to start tracking that body
//   - Ctrl + left-drag on a selected body: apply a torque (rotate it)
//   - Ctrl + right-drag on a selected body: apply a force (push/pull it)
// Call applyPerturbation() once per simulation step, right before
// mj_step()/mj_forward(), so the drag is actually felt by the physics:
//
//   while (!glfwWindowShouldClose(ui.window)) {
//     while (mj_data->time - simstart < 1.0/60.0 && ui.runSim) {
//       ui.applyPerturbation();
//       mj_step(mj_model, mj_data);
//     }
//     ui.updateScene();
//   }
class UIctr{
public:
    GLFWwindow *window;
    // keyboard
    struct ButtonState {
        bool key_w{false};
        bool key_s{false};
        bool key_a{false};
        bool key_d{false};
        bool key_h{false};
        bool key_j{false};
        bool key_space{false};
    } buttonRead;

    // mouse interaction
    bool button_left{false};
    bool button_middle{false};
    bool button_right{false};

    bool runSim{true};
    bool isContinuous{true};
    double lastx{0};
    double lasty{0};
    mjModel* mj_model;
    mjData* mj_data;

    UIctr(mjModel *modelIn, mjData *dataIn);
    void iniGLFW();
    void createWindow(const char * windowTitle, bool saveVideo);
    void updateScene();

    // real-time sensor plot
    void initSensorFigure(const char* title, const char* lineNames[], const float lineColors[][3], int numLines);
    void updateSensorFigure(double time, const double* values, int numLines);

    // keyboard callback
    void Keyboard(int key, int scancode, int act, int mods);
    // mouse button callback
    void Mouse_button(int button, int act, int mods);
    // mouse move callback
    void Mouse_move(double xpos, double ypos);
    // scroll callback
    void Scroll(double xoffset, double yoffset);

    ButtonState getButtonState();

    // apply the current mouse perturbation (if any) to mj_data, before
    // stepping/forwarding the physics: a spring-like force/torque toward the
    // drag target while running, or a direct qpos edit while paused
    void applyPerturbation();

    void Close();

    void enableTracking();
    void disableTracking();

private:
    unsigned char* image_rgb_;
    float* image_depth_;

    FILE* file;

    int width{1200};
    int height{800};
    bool save_video{false};

    bool isTrack{false};

    // double-click detection, used to select a body for perturbation
    double lastClickTime_{0};
    int lastClickButton_{-1};
    static constexpr double kDoubleClickSeconds = 0.3;

    // UI handler
    mjvCamera cam;                      // abstract camera
    mjvOption opt;                      // visualization options
    mjvScene scn;                       // abstract scene
    mjrContext con;                     // custom GPU context
    mjvPerturb pert;                    // mouse perturbation / selection state

    // realt-time sensor plot
    mjvFigure figSensor;
    bool sensorFigureIni{false};
    static constexpr int sensorFigMaxPnt = 150; // rolling window length, in pushed samples
};
