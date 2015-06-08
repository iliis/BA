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




    CameraStep step = testscene.getStep(0);

    cout << step.ground_truth << endl;

    Warp::PlotRange range1(0,-1,1,40);
    Warp::PlotRange range2(4,-.1,.1,40);

    Eigen::MatrixXf errors(range1.steps, range2.steps);

    sf::Clock clock;
    clock.restart();
    Warp::renderErrorSurface(errors, step, step.ground_truth, range1, range2);
    sf::Int32 ms = clock.getElapsedTime().asMilliseconds();

    cout << "rendered surface in " << ms << "ms (" << ((float) ms) / (errors.rows() * errors.cols()) << "ms per point)" << endl;

    ImageData errorplot;
    errorplot.loadFromMatrix(errors, Colormap::Jet());

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

        sf::sleep(sf::milliseconds(100));
    }

    return 0;
}
