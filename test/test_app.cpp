#include "b3_sim_app.hpp"

int main(int argc, char* argv[]) {

    b3SimApp app;

    app.load_scene("gear.json");

    app.launch(true);

    return 0;
}

