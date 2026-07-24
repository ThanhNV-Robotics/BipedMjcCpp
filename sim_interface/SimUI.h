#pragma once

#include <memory>
#include <string>

#include <mujoco/mujoco.h>

#include "glfw_adapter.h"
#include "simulate.h"

namespace mj = ::mujoco;

// Wraps MuJoCo's own `simulate` GUI (mj::Simulate + GlfwAdapter) to visualize
// an externally-owned mjModel/mjData. SimUI never loads or frees the model:
// that stays the caller's responsibility (e.g. MJ_Interface); this class only
// renders and forwards user interaction (camera, pause/run, sliders) through
// MuJoCo's native UI.
//
// mj::Simulate requires two threads, same as MuJoCo's own simulate/main.cc:
//   - a "render" thread that calls RenderLoop() (must be the main thread on
//     macOS, so always use main() for it to keep the code portable)
//   - a "physics" thread that calls LoadIntoUI() once and then repeatedly
//     steps the model, e.g.:
//
//   SimUI ui(model, data, "scene.xml");        // construct on main thread
//   std::thread physics([&] {
//     ui.LoadIntoUI();                         // must run off the render thread
//     while (!ui.ShouldExit()) {
//       ui.Step();                             // locks internally
//     }
//   });
//   ui.RenderLoop();                           // blocking, main thread
//   physics.join();
//
// Constructing SimUI is safe on the main thread (it only sets up GLFW/camera
// state); calling LoadIntoUI() there before RenderLoop() starts would
// deadlock, since nothing would be servicing the load request yet.
class SimUI {
 public:
  SimUI(mjModel* modelIn, mjData* dataIn, const std::string& windowTitle = "BipedMjcCpp");

  // Registers the model with the UI's render thread and blocks until it has
  // been picked up. Call exactly once, from a thread other than the one
  // running RenderLoop().
  void LoadIntoUI();

  // Blocking call: runs the render/UI loop on the calling thread. Returns
  // once the user closes the window.
  void RenderLoop();

  // Advances the simulation by one step if the UI's Run/Pause toggle is set
  // to Run, otherwise calls mj_forward() so edits made through the UI
  // (sliders, watch panel, perturbations) are still reflected. Locks the UI
  // mutex for the duration of the call. Call this repeatedly from the
  // physics thread.
  void Step();

  // Points the camera to track a body (e.g. the robot's floating base)
  // instead of the default free camera. Call before RenderLoop() starts.
  void EnableTracking(int bodyId = 1);

  bool IsRunning() const { return sim_->run != 0; }
  bool ShouldExit() const { return sim_->exitrequest.load() != 0; }
  void RequestExit() { sim_->exitrequest.store(1); }

  mj::SimulateMutex& mutex() { return sim_->mtx; }

 private:
  mjModel* mj_model_;
  mjData* mj_data_;
  std::string window_title_;

  mjvCamera cam_;
  mjvOption opt_;
  mjvPerturb pert_;

  // declared after cam_/opt_/pert_ so it is destroyed first (Simulate keeps
  // references to them, bound at construction)
  std::unique_ptr<mj::Simulate> sim_;
};
