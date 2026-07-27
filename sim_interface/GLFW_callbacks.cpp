#include "GLFW_callbacks.h"

UIctr::UIctr(mjModel *modelIn, mjData *dataIn) {
    mj_model=modelIn;
    mj_data=dataIn;
    cam=mjvCamera();
    opt=mjvOption();
    scn=mjvScene();
    con=mjrContext();
    pert=mjvPerturb();
}


static void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    ((UIctr*)(glfwGetWindowUserPointer(window)))->Scroll(xoffset, yoffset);
}
static void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    ((UIctr*)(glfwGetWindowUserPointer(window)))->Mouse_move(xpos, ypos);
}
static void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    ((UIctr*)(glfwGetWindowUserPointer(window)))->Mouse_button(button, act, mods);
}

static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    ((UIctr*)(glfwGetWindowUserPointer(window)))->Keyboard(key, scancode, act, mods);
}

static void window_close_callback(GLFWwindow* window)
{
    ((UIctr*)(glfwGetWindowUserPointer(window)))->Close();
}

void UIctr::iniGLFW() {
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");
}

// create window, make OpenGL context current, request v-sync, adjust view, bond callbacks, etc.
void UIctr::createWindow(const char* windowTitle, bool saveVideo) {
    window=glfwCreateWindow(width, height, windowTitle, NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // frame the camera using the model's own <statistic> center/extent
    mjv_defaultFreeCamera(mj_model, &cam);

    if (isTrack) {
        cam.type = mjCAMERA_TRACKING;
        cam.trackbodyid = 1;
    }
    else
        cam.type = mjCAMERA_FREE;

    mjv_defaultOption(&opt);
    mjv_defaultPerturb(&pert);
    pert.select = 0;
    pert.flexselect = -1;
    pert.skinselect = -1;
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(mj_model, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(mj_model, &con, mjFONTSCALE_150);   // model-specific context
    mjv_moveCamera(mj_model, mjMOUSE_ROTATE_H, 0.0, 0.0, &scn, &cam);

    // install GLFW mouse and keyboard callbacks
    glfwSetWindowUserPointer(window, this);
    glfwSetWindowCloseCallback(window, window_close_callback);
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    save_video=saveVideo;
    if (save_video)
    {
        image_rgb_ = (unsigned char*)malloc(3*width*height*sizeof(unsigned char));
        image_depth_ = (float*)malloc(sizeof(float)*width*height);

        // create output rgb file
        file = fopen("../record/rgbRec.out", "wb");
        if( !file )
            mju_error("Could not open rgbfile for writing");
    }
}

void UIctr::updateScene() {
    if (!isContinuous)
        runSim= false;

    buttonRead.key_w=false;
    buttonRead.key_a=false;
    buttonRead.key_s=false;
    buttonRead.key_d=false;
    buttonRead.key_space=false;
    buttonRead.key_h=false;
    buttonRead.key_j=false;

    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwMakeContextCurrent(window);
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene (include the perturbation force/torque arrow while dragging) and render
    mjv_updateScene(mj_model, mj_data, &opt, &pert, &cam, mjCAT_ALL, &scn);
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    mjr_render(viewport, &scn, &con);
    std::string timeStr = "Simulation Time: " + std::to_string(mj_data->time);
    char buffer[100];
    std::sprintf(buffer, "Time: %.3f", mj_data->time);

    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, viewport, buffer, NULL, &con);


    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);
    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();

    if (save_video)
    {
        mjr_readPixels(image_rgb_, image_depth_, viewport, &con);
        fwrite(image_rgb_, sizeof(unsigned char), 3*width*height, file);
    }
}



// keyboard callback
void UIctr::Keyboard(int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(mj_model, mj_data);
        mj_forward(mj_model, mj_data);
    }

    if (act==GLFW_RELEASE && key==GLFW_KEY_1)
    {
        runSim=!runSim;
        isContinuous= true;
    }

    if (act==GLFW_RELEASE && key==GLFW_KEY_2)
    {
        runSim= true;
        isContinuous= false;
    }

    if (act==GLFW_RELEASE && key==GLFW_KEY_W){
        buttonRead.key_w= true;
    }

    if (act==GLFW_RELEASE && key==GLFW_KEY_A){
        buttonRead.key_a= true;
    }

    if (act==GLFW_RELEASE && key==GLFW_KEY_S){
        buttonRead.key_s= true;
    }

    if (act==GLFW_RELEASE && key==GLFW_KEY_D){
        buttonRead.key_d= true;
    }

    if (act==GLFW_RELEASE && key==GLFW_KEY_H){
        buttonRead.key_h= true;
    }

    if (act==GLFW_RELEASE && key==GLFW_KEY_J){
        buttonRead.key_j= true;
    }

    if (act==GLFW_RELEASE && key==GLFW_KEY_SPACE){
        buttonRead.key_space= true;
    }
}

// mouse button callback
void UIctr::Mouse_button(int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);

    // release: stop any ongoing perturbation drag
    if (act == GLFW_RELEASE) {
        pert.active = 0;
        return;
    }

    if (button != GLFW_MOUSE_BUTTON_LEFT && button != GLFW_MOUSE_BUTTON_RIGHT) {
        return;
    }

    bool mod_ctrl = (mods & GLFW_MOD_CONTROL) != 0;

    // double-click: select a body (left) or move the camera look-at point
    // (right; Ctrl+right also starts tracking it) -- mirrors simulate.cc
    double now = glfwGetTime();
    bool doubleClick = (button == lastClickButton_) && (now - lastClickTime_ < kDoubleClickSeconds);
    lastClickTime_ = now;
    lastClickButton_ = button;

    if (doubleClick) {
        int fbWidth, fbHeight;
        glfwGetFramebufferSize(window, &fbWidth, &fbHeight);
        if (fbWidth <= 0 || fbHeight <= 0) {
            return;
        }

        mjtNum selpnt[3];
        int selgeom, selflex, selskin;
        int selbody = mjv_select(mj_model, mj_data, &opt,
                                  static_cast<mjtNum>(fbWidth) / fbHeight,
                                  lastx / fbWidth, 1.0 - lasty / fbHeight,
                                  &scn, selpnt, &selgeom, &selflex, &selskin);

        // selection mode: left click always selects for perturbation;
        // right click only moves the look-at point (and tracks if Ctrl held)
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            pert.active = 0;
            if (selbody >= 0) {
                pert.select = selbody;
                pert.flexselect = selflex;
                pert.skinselect = selskin;

                mjtNum tmp[3];
                mju_sub3(tmp, selpnt, mj_data->xpos + 3*selbody);
                mju_mulMatTVec(pert.localpos, mj_data->xmat + 9*selbody, tmp, 3, 3);
            } else {
                pert.select = 0;
                pert.flexselect = -1;
                pert.skinselect = -1;
            }
        } else {
            if (selbody >= 0) {
                mju_copy3(cam.lookat, selpnt);
            }
            if (mod_ctrl && selbody > 0) {
                cam.type = mjCAMERA_TRACKING;
                cam.trackbodyid = selbody;
                cam.fixedcamid = -1;
            }
        }
        return;
    }

    // Ctrl + click (not a double-click) on an already-selected body:
    // start perturbing it -- right drags (translate), left twists (rotate)
    if (mod_ctrl && pert.select > 0) {
        mjv_initPerturb(mj_model, mj_data, &scn, &pert);
        pert.active = (button == GLFW_MOUSE_BUTTON_RIGHT) ? mjPERT_TRANSLATE : mjPERT_ROTATE;
    }
}


// mouse move callback
void UIctr::Mouse_move(double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // drag the perturbation target if one is active, otherwise move the camera
    if (pert.active) {
        mjv_movePerturb(mj_model, mj_data, action, dx/height, dy/height, &scn, &pert);
    } else {
        mjv_moveCamera(mj_model, action, dx/height, dy/height, &scn, &cam);
    }
}


// scroll callback
void UIctr::Scroll(double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(mj_model, mjMOUSE_ZOOM, 0, 0.05*yoffset, &scn, &cam);
}

void UIctr::applyPerturbation()
{
    if (!pert.select) {
        return;
    }

    if (runSim) {
        // clear old perturbations, apply new (mocap bodies kinematically,
        // dynamic bodies via a spring-like force/torque in xfrc_applied)
        mju_zero(mj_data->xfrc_applied, 6*mj_model->nbody);
        mjv_applyPerturbPose(mj_model, mj_data, &pert, 0);
        mjv_applyPerturbForce(mj_model, mj_data, &pert);
    } else {
        // paused: let the drag move mocap bodies and dynamic bodies' qpos directly
        mjv_applyPerturbPose(mj_model, mj_data, &pert, 1);
    }
}

void UIctr::Close() {
    // Free mujoco objects
    mj_deleteData(mj_data);
    mj_deleteModel(mj_model);
    mjr_freeContext(&con);
    mjv_freeScene(&scn);


    glfwTerminate();
}

void UIctr::enableTracking() {
    isTrack=true;
}

void UIctr::disableTracking(){
    isTrack = false;
}

UIctr::ButtonState UIctr::getButtonState() {
    ButtonState tmp=buttonRead;
    buttonRead.key_w= false;
    buttonRead.key_a= false;
    buttonRead.key_s= false;
    buttonRead.key_d= false;
    buttonRead.key_h= false;
    buttonRead.key_j= false;
    buttonRead.key_space= false;
    return tmp;
}
