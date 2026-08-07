// Sanity-check test: launch MuJoCo's own "simulate" GUI (built from source,
// see third_party/mujoco/simulate) loaded with the biped robot model, via
// the project's SimUI wrapper (sim_interface/SimUI.h).
// Run with no arguments to load models/mjcf/scene.xml, or pass an explicit
// path to another MJCF/URDF/.mjb file to override it.

#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include <mujoco/mujoco.h>

#include "SimUI.h"

const std::string BIPED_MODEL_DIR = "models/mjcf";

int main(int argc, char** argv) {
  std::printf("MuJoCo version %s\n", mj_versionString());

  const std::string default_model = BIPED_MODEL_DIR + "/scene.xml";
  const std::string model_path = (argc > 1) ? argv[1] : default_model;

  char loadError[1024] = "";
  mjModel* m = mj_loadXML(model_path.c_str(), nullptr, loadError, sizeof(loadError));
  if (!m) {
    std::fprintf(stderr, "failed to load %s: %s\n", model_path.c_str(), loadError);
    return 1;
  }
  mjData* d = mj_makeData(m);

  SimUI ui(m, d, model_path);

  std::thread physics([&ui]() {
    ui.LoadIntoUI();
    while (!ui.ShouldExit()) {
      ui.Step();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  ui.RenderLoop();  // blocking call, runs the GUI on this thread
  physics.join();

  mj_deleteData(d);
  mj_deleteModel(m);

  return 0;
}
