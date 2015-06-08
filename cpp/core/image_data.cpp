#include "image_data.h"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
void ImageData::create(const unsigned int W, const unsigned int H)
{
    assert(W > 0); assert(H > 0);

    sf_image.create(W,H);
    data    .resize(H,W);
    width  = W;
    height = H;
}
///////////////////////////////////////////////////////////////////////////////
void ImageData::loadFromMatrix(const Eigen::MatrixXf& source_data)
{
    width  = source_data.cols();
    height = source_data.rows();

    // copy data
    data   = source_data;
    updateImageFromMatrix();

    // cout << "loaded image from matrix " << width << " x " << height << endl;
}
///////////////////////////////////////////////////////////////////////////////
void ImageData::loadFromImage(const sf::Image& source_data)
{
    width    = source_data.getSize().x;
    height   = source_data.getSize().y;

    // copy data
    sf_image = source_data;
    sf_texture.loadFromImage(sf_image);
    updateMatrixFromImage();

    // cout << "loaded image from image " << width << " x " << height << endl;
}
///////////////////////////////////////////////////////////////////////////////
void ImageData::loadFromFile(const std::string source_file)
{
    sf_image.loadFromFile(source_file);
    sf_texture.loadFromImage(sf_image);

    width    = sf_image.getSize().x;
    height   = sf_image.getSize().y;

    updateMatrixFromImage();

    // cout << "loaded image from file " << width << " x " << height << endl;
}
///////////////////////////////////////////////////////////////////////////////
void ImageData::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
    this->drawAt(target, sf::Vector2f(0,0), 1, states);
}
///////////////////////////////////////////////////////////////////////////////
void ImageData::drawAt(sf::RenderTarget& target, const sf::Vector2f pos, const float scale, sf::RenderStates states) const
{
    sf::Sprite sf_sprite;
    sf_sprite.setPosition(pos.x, pos.y);
    sf_sprite.setTexture(sf_texture);
    sf_sprite.setScale(scale, scale);
    target.draw(sf_sprite, states);
}
///////////////////////////////////////////////////////////////////////////////
void ImageData::updateMatrixFromImage()
{
    image_to_matrix(sf_image, data);
}
///////////////////////////////////////////////////////////////////////////////
void ImageData::updateImageFromMatrix()
{
    matrix_to_image(data, sf_image);
    sf_texture.loadFromImage(sf_image);
}
///////////////////////////////////////////////////////////////////////////////
void ImageData::image_to_matrix(const sf::Image& source, Eigen::MatrixXf& dest)
{
    // cout << "converting image to matrix ...";

    // make sure destination matrix can hold the entire image
    // this operation is a no-op if dest already has the right size
    dest.resize(source.getSize().y, source.getSize().x);

    for (unsigned int y = 0; y < source.getSize().y; y++) {
        for (unsigned int x = 0; x < source.getSize().x; x++) {
            // convert image to grayscale
            const sf::Color& c = source.getPixel(x,y);
            dest(y,x) = (c.r / 255.0f + c.g / 255.0f + c.b / 255.0f) / 3;
        }
    }

    // cout << "done." << endl;
}
///////////////////////////////////////////////////////////////////////////////
void ImageData::matrix_to_image(const Eigen::MatrixXf& source, sf::Image& dest)
{
    // cout << "converting matrix to image ...";

    // initialize and clear image
    dest.create(source.cols(), source.rows());

    float near = source.minCoeff();
    float far  = source.maxCoeff();

    // convert matrix data into grayscale image
    for (unsigned int y = 0; y < source.rows(); y++) {
        for (unsigned int x = 0; x < source.cols(); x++) {
            // scale values
            float v = (source(y,x)-near) / (far-near);
            dest.setPixel(x, y, sf::Color(v*255, v*255, v*255));
        }
    }

    // cout << "done." << endl;
}
///////////////////////////////////////////////////////////////////////////////
void ImageData::scale(float near, float far)
{
    data = data.array()*(far-near) + near;
    // no need to update image
}
///////////////////////////////////////////////////////////////////////////////
float ImageData::sampleValue(Eigen::Vector2f pos) const
{
    assert(pos.x() >= 0);
    assert(pos.y() >= 0);
    assert(pos.x() <= width-1);
    assert(pos.y() <= height-1);

    // remember: data[row, col] = data[y, x]!
    float v1 = data(floor(pos.y()), floor(pos.x()));
    float v2 = data( ceil(pos.y()), floor(pos.x()));
    float v3 = data(floor(pos.y()),  ceil(pos.x()));
    float v4 = data( ceil(pos.y()),  ceil(pos.x()));

    return (v1 + v2 + v3 + v4) / 4;
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void drawImageAt(const sf::Image& img, const sf::Vector2f& pos, sf::RenderTarget& target, sf::RenderStates states)
{
    sf::Texture t;
    t.loadFromImage(img);

    sf::Sprite s;
    s.setTexture(t);
    s.setPosition(pos.x, pos.y);

    target.draw(s, states);
}
///////////////////////////////////////////////////////////////////////////////
