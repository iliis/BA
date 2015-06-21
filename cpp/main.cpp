#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>
#include <SFML/Graphics.hpp>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "gui/scene.h"
#include "gui/sf_image_data.h"
#include "gui/live.h"
#include "gui/minimization_gui.h"
#include "core/image_data.h"
#include "core/transformation.h"
#include "core/camera_intrinsics.h"
#include "core/warp.h"
#include "core/minimization.h"

using namespace std;
using namespace Eigen;

//sf::RenderWindow window(sf::VideoMode(1280, 768), "dense odometry");
//sf::RenderWindow window(sf::VideoMode(4*752+6, 3*480), "dense odometry");
//sf::RenderWindow window(sf::VideoMode(256*4+2, 128*5), "dense odometry");
//sf::RenderWindow window(sf::VideoMode(256*3+4, 128*4+4), "dense odometry");

///////////////////////////////////////////////////////////////////////////////
void write_trajectory(const Scene& scene, const Warp::Parameters& params, string output_dir = "")
{
    std::vector<Transformation> traj = findTrajectory(scene, params);

    if (output_dir.empty()) {
        output_dir = scene.getSourceDirectory();
    }

    string path = output_dir + "/measured_trajectory.csv";
    ofstream outfile(path.c_str());

    outfile << "x, y, z, alpha, beta, gamma" << endl;

    for(size_t i = 0; i < traj.size(); i++) {
        traj[i].printCSV(outfile) << endl;
    }

    outfile.close();
}
///////////////////////////////////////////////////////////////////////////////
void write_trajectory_rosbag(const string& rosbag_path, const Warp::Parameters& params, string output_dir = ".")
{
    std::vector<Transformation> traj = findTrajectoryFromRosbag(rosbag_path, params);

    if (traj.empty()) {
        cout << "empty trajectory. maybe this is raw footage?" << endl;

        traj = findTrajectoryFromRosbagRaw(rosbag_path, params);

        if (traj.empty()) {
            cerr << "nope, no raw data. giving up." << endl;
            return;
        }
    }

    string path = output_dir + "/measured_trajectory.csv";
    ofstream outfile(path.c_str());

    outfile << "x, y, z, alpha, beta, gamma" << endl;

    for(size_t i = 0; i < traj.size(); i++) {
        traj[i].printCSV(outfile) << endl;
    }

    outfile.close();

    cout << "wrote trajectory to disk" << endl;
}
///////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(1400, 768), "dense odometry", sf::Style::Default, settings);

    sf::Font font;
    font.loadFromFile("resources/fonts/default.otf");

    Scene testscene;
    //testscene.loadFromSceneDirectory("../matlab/input/test_wide");
    //testscene.loadFromSceneDirectory("../matlab/input/trajectory1");
    //testscene.loadFromSceneDirectory("../matlab/input/testscene1");
    //testscene.loadFromSceneDirectory("../matlab/input/courtyard/lux");
    //testscene.loadFromSceneDirectory("../matlab/input/courtyard_circle");
    //testscene.loadFromSceneDirectory("../matlab/input/courtyard/normal"); // step 22 is nice!
    //testscene.loadFromBagFile("/home/samuel/data/2015-06-11-16-30-01.bag");
    
    //const string raw_bag = "/home/samuel/data/visensor/graveyard_small_circle1_forward.bag";
    const string raw_bag = "/home/samuel/data/visensor/graveyard_path4.bag";
    //testscene.loadFromBagFileRaw(raw_bag);

    //cout << testscene.getIntrinsics() << endl;

    Warp::Parameters params(new ErrorWeightNone());
    params.pyramid_levels = 4;
    params.max_iterations = 100;
    params.T_init = Transformation(0,0,0,0,0,0);
    params.gradient_norm_threshold = 0.01;

    //write_trajectory(testscene, params, ".");
    //write_trajectory_rosbag("/home/samuel/data/2015-06-11-16-30-01.bag", params, ".");
    write_trajectory_rosbag(raw_bag, params, ".");

    //run_minimization(window, font, testscene, params);

    //show_live_data(window, font, argc, argv);


    delete params.weight_function;

    return EXIT_SUCCESS;
}
///////////////////////////////////////////////////////////////////////////////
