#include <stdlib.h>
#include <iostream>
#include <SFML/Graphics.hpp>

#include "utils/eigen_sfml_conversions.h"

using namespace std;

int main()
{
    sf::RenderWindow window(sf::VideoMode(1024, 768), "SFML test");

    sf::Image img;
    if (!img.loadFromFile("../matlab/input/trajectory1/color0001.png")) {
        cerr << "cannot load image." << endl;
        return EXIT_FAILURE;
    }


    cout << "image size: " << img.getSize().x << " x " << img.getSize().y << endl;
    cout << "first pixel: " << (int) img.getPixel(0, 0).r << endl;

    Eigen::MatrixXf* matrix = image_to_matrix(img);

    sf::Image* img2 = matrix_to_image(*matrix);

    img.copy(*img2, 0, 0);

    delete img2;
    delete matrix;


    sf::Texture tex;
    if (!tex.loadFromImage(img)) {
        cerr << "cannot convert image to texture (i.e. moving it to graphics card)" << endl;
    }
    sf::Sprite sprite;
    sprite.setTexture(tex);

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
        window.display();
    }

    return 0;
}
