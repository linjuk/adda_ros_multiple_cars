#include <despot/simple_tui.h>
#include "Car.h"
#include "ros/ros.h"

using namespace despot;

class TUI: public SimpleTUI {
public:
    TUI() {}

    DSPOMDP* InitializeModel(option::Option* options) {
        DSPOMDP* model = new Car();
        return model;
    }

    void InitializeDefaultParameters() {
        Globals::config.num_scenarios = 400; // Used for sampling from particle vector
        Globals::config.search_depth = 40; // Tree search depth
        Globals::config.sim_len = 200; // Max runtime length
    }
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "despot");
    return TUI().run(argc, argv);
}