#include <stdlib.h>
#include <iostream>
#include <SFML/Graphics.hpp>

#include "core/scene.h"
#include "core/image_data.h"
#include "core/transformation.h"
#include "core/camera_intrinsics.h"
#include "core/warp.h"

using namespace std;

int main()
{
    sf::RenderWindow window(sf::VideoMode(1024, 768), "SFML test");

    sf::Image img;
    if (!img.loadFromFile("../matlab/input/testscene1/color0001.png")) {
        cerr << "cannot load image." << endl;
        return EXIT_FAILURE;
    }


    cout << "image size: " << img.getSize().x << " x " << img.getSize().y << endl;
    cout << "first pixel: " << (int) img.getPixel(0, 0).r << endl;

    ImageData imgdata2;
    imgdata2.loadFromImage(img);

    ImageData imgdata;
    imgdata.loadFromMatrix(imgdata2.getData());

    CameraImage scene_image;
    scene_image.loadFromSceneDirectory("../matlab/input/testscene1", 1);




    CameraIntrinsics intrinsics;
    intrinsics.loadFromCSV("../matlab/input/testscene1/camera_intrinsics.csv");

    cout << "intrinsics:" << intrinsics << endl;


    Transformation t;
    cout << "some Transformation: " << t << endl;

    vector<Transformation> path = Transformation::loadFromCSV("../matlab/input/testscene1/camera_trajectory_relative.csv");

    cout << "loaded " << path.size() << " frames" << endl;


    Scene testscene;
    testscene.loadFromSceneDirectory("../matlab/input/testscene1");

    CameraStep step = testscene.getStep(0);

    Pixel p = step.frame_second.getPixel(Eigen::Vector2i(100,100));
    cout << "pixel [0,0]: " << p.pos.x() << " " << p.pos.y() << " = " << p.intensity << " / " << p.depth << endl;



    sf::Texture tex;
    if (!tex.loadFromImage(img)) {
        cerr << "cannot convert image to texture (i.e. moving it to graphics card)" << endl;
    }
    sf::Sprite sprite;
    sprite.setTexture(tex);
    sprite.setPosition(500,0);


    sf::Clock clock;
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear();
        window.draw(sprite);
        //window.draw(imgdata);
        //window.draw(scene_image);

        clock.restart();
        //Warp::drawError(window, testscene.getStep(0), Transformation(0,0,0,0,0,0));
        Warp::drawError(window, testscene.getStep(0), testscene.getStep(0).ground_truth);
        cout << clock.getElapsedTime().asMilliseconds() << endl;
        window.display();
    }

    return 0;
}
