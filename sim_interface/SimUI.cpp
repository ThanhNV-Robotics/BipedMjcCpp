#include "SimUI.h"

SimUI::SimUI(mjModel* modelIn, mjData* dataIn, const std::string& windowTitle)
    : mj_model_(modelIn), mj_data_(dataIn), window_title_(windowTitle) {
  mjv_defaultCamera(&cam_);
  mjv_defaultOption(&opt_);
  mjv_defaultPerturb(&pert_);

  // frame the camera using the model's own <statistic> center/extent, rather
  // than a fixed distance tuned for a different robot's scale
  mjv_defaultFreeCamera(mj_model_, &cam_);

  sim_ = std::make_unique<mj::Simulate>(std::make_unique<mj::GlfwAdapter>(), &cam_, &opt_,
                                         &pert_, /*is_passive=*/false);
}

void SimUI::LoadIntoUI() { sim_->Load(mj_model_, mj_data_, window_title_.c_str()); }

void SimUI::RenderLoop() { sim_->RenderLoop(); }

void SimUI::Step() {
  const mj::MutexLock lock(mutex());
  if (mj_model_ && mj_data_) {
    if (IsRunning()) {
      mj_step(mj_model_, mj_data_);
    } else {
      mj_forward(mj_model_, mj_data_);
    }
  }
}

void SimUI::EnableTracking(int bodyId) {
  cam_.type = mjCAMERA_TRACKING;
  cam_.trackbodyid = bodyId;
}
