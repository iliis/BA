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

    Scene testscene;
    testscene.loadFromSceneDirectory("../matlab/input/trajectory1");



    ImageData errorplot;

    CameraStep step = testscene.getStep(0);

    cout << step.ground_truth << endl;

    sf::Clock clock;
    clock.restart();
    //Warp::drawError(window, testscene.getStep(0), Transformation(0,0,0,0,0,0));
    //Warp::drawError(window, testscene.getStep(0), testscene.getStep(0).ground_truth);
    Warp::renderErrorSurface(errorplot, step, step.ground_truth, Warp::PlotRange(0,-1,1,40), Warp::PlotRange(4,-.1,.1,40));
    //Warp::renderErrorSurface(errorplot, step, step.ground_truth, Warp::PlotRange(0,-1,1,3), Warp::PlotRange(4,-.1,.1,3));
    //Warp::renderErrorSurface(errorplot, step, step.ground_truth, Warp::PlotRange(4,-.1,.1,3), Warp::PlotRange(0,-1,1,3));
    sf::Int32 ms = clock.getElapsedTime().asMilliseconds();

    cout << "size: " << errorplot.getWidth() << " x " << errorplot.getHeight() << endl;

    cout << "rendered surface in " << ms << "ms (" << ((float) ms) / (errorplot.getWidth() * errorplot.getHeight()) << "ms per point)" << endl;

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear();
        //window.draw(errorplot);
        errorplot.drawAt(window, sf::Vector2f(0,0), 16);
        window.display();
    }

    return 0;
}
