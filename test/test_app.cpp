#include "b3_sim_app.hpp"

int main(int argc, char* argv[]) {

    box3d::b3SimApp app;

    app.load_scene("sphere.json");

    box3d::b3GUIViewer viewer;

    viewer.set_world(app.get_world());

    viewer.launch();

    return 0;
}

