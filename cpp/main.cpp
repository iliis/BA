#include <stdlib.h>
#include <iostream>
#include <SFML/Graphics.hpp>
//#include <SFML/Image.hpp>

using namespace std;

int main()
{
    sf::RenderWindow window(sf::VideoMode(1024, 768), "SFML test");

    sf::Image img;
    if (!img.loadFromFile("../matlab/input/trajectory1/color0001.png")) {
        cerr << "cannot load image." << endl;
        return EXIT_FAILURE;
    }

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
