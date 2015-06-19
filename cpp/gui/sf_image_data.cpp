#include "sf_image_data.h"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
void loadImageDataFromImage(ImageData& dest, const sf::Image& source_data)
{
    image_to_matrix(source_data, dest.data);

    // cout << "loaded image from image " << width << " x " << height << endl;
}
///////////////////////////////////////////////////////////////////////////////
void loadImageDataFromFile (ImageData& dest, const std::string source_file)
{
    sf::Image sf_image;
    sf_image.loadFromFile(source_file);

    image_to_matrix(sf_image, dest.data);

    // cout << "loaded image from file " << width << " x " << height << endl;
}
///////////////////////////////////////////////////////////////////////////////
void loadImageDataFromROSgrayscale(ImageData& dest, const sensor_msgs::Image& source_data)
{
    unsigned int width  = source_data.width;
    unsigned int height = source_data.height;

    dest.data.resize(height, width);

    // copy data
    for (unsigned int y = 0; y < height; y++) {
        for (unsigned int x = 0; x < width; x++) {
            dest.data(y, x) = source_data.data[(y*width+x)*3] / 255.0f;
        }
    }
}
///////////////////////////////////////////////////////////////////////////////
void loadImageDataFromROSdepthmap (ImageData& dest, const sensor_msgs::Image& source_data)
{
    unsigned int width  = source_data.width;
    unsigned int height = source_data.height;

    dest.data.resize(height, width);

    // copy data
    const float* fdata = reinterpret_cast<const float*>(source_data.data.data());

    for (unsigned int y = 0; y < height; y++) {
        for (unsigned int x = 0; x < width; x++) {
            //data(y, x) = source_data.data[(y*width+x)*3] / 255.0f;
            unsigned int i = y*width+x;

            // invalid data is exactly '1'
            // these are disparity values, not actual depth data!
            // TODO: use baseline value here!
            if (fdata[i] == 1.0f) {
                dest.data(y, x) = std::numeric_limits<float>::quiet_NaN();
            } else {
                //data(y, x) = 1.0f / fdata[i];
                dest.data(y, x) = fdata[i];
            }
        }
    }
}
///////////////////////////////////////////////////////////////////////////////
void loadImageDataFromROSraw(ImageData& dest, const sensor_msgs::Image& source_data)
{
    unsigned int width  = source_data.width;
    unsigned int height = source_data.height;

    dest.data.resize(height, width);

    // copy data
    for (unsigned int y = 0; y < height; y++) {
        for (unsigned int x = 0; x < width; x++) {
            unsigned const char v = source_data.data[(y*width+x)];
            if (v > 0 && v < 240)
                dest.data(y, x) = (float) v;
            else
                dest.data(y, x) = std::numeric_limits<float>::quiet_NaN();
        }
    }
}
///////////////////////////////////////////////////////////////////////////////
void image_to_matrix(const sf::Image& source, Eigen::MatrixXf& dest)
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
void matrix_to_image(const Eigen::MatrixXf& source, sf::Image& dest, const Colormap::Colormap& colormap)
{
    if (source.rows() == 0 || source.cols() == 0) {
        dest.create(100,100, sf::Color(0,0,255));
        return;
    }

    // initialize and clear image
    dest.create(source.cols(), source.rows());

    // matrix might contain NaNs
    float near = minNoNaN(source);
    float far  = maxNoNaN(source);

    // convert matrix data into grayscale image
    for (unsigned int y = 0; y < source.rows(); y++) {
        for (unsigned int x = 0; x < source.cols(); x++) {
            dest.setPixel(x, y, colormap(source(y,x), near, far));
        }
    }

    // cout << "done." << endl;
}
///////////////////////////////////////////////////////////////////////////////
void drawImageAt(sf::RenderTarget& target, const sf::Image& img, const sf::Vector2f& pos, const std::string& label, const sf::Font* font, const float& scale)
{
    sf::Texture t;
    t.loadFromImage(img);

    sf::Sprite s;
    s.setTexture(t);
    s.setPosition(pos.x, pos.y);
    s.setScale(scale, scale);

    target.draw(s);

    if (!label.empty() && font) {
        sf::Text t;
        t.setFont(*font);
        t.setCharacterSize(12);
        t.setString(label);
        t.setPosition(pos.x+2, pos.y+img.getSize().y*scale-t.getLocalBounds().height-4);
        target.draw(t);
    }
}
///////////////////////////////////////////////////////////////////////////////
void drawMatrixAt(sf::RenderTarget& target, const Eigen::MatrixXf& mat, const sf::Vector2f& pos, const std::string& label, const sf::Font* font, const Colormap::Colormap& colormap, const float& scale)
{
    sf::Image i;
    matrix_to_image(mat, i, colormap);

    drawImageAt(target, i, pos, label, font, scale);
}
///////////////////////////////////////////////////////////////////////////////
