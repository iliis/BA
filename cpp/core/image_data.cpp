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
void ImageData::loadFromMatrix(const Eigen::MatrixXf& source_data, const Colormap::Colormap& colormap)
{
    width  = source_data.cols();
    height = source_data.rows();

    // copy data
    data   = source_data;
    updateImageFromMatrix(colormap);

    //cout << "loaded image from matrix " << width << " x " << height << " (" << data.cols() << " x " << data.rows() << ")" << endl;
}
///////////////////////////////////////////////////////////////////////////////
void ImageData::loadFromROSgrayscale(const sensor_msgs::Image& source_data, const Colormap::Colormap& colormap)
{
    width  = source_data.width;
    height = source_data.height;

    data.resize(height, width);

    // copy data
    for (unsigned int y = 0; y < height; y++) {
        for (unsigned int x = 0; x < width; x++) {
            data(y, x) = source_data.data[(y*width+x)*3] / 255.0f;
        }
    }

    updateImageFromMatrix(colormap);
}
///////////////////////////////////////////////////////////////////////////////
void ImageData::loadFromROSdepthmap(const sensor_msgs::Image& source_data, const Colormap::Colormap& colormap)
{
    width  = source_data.width;
    height = source_data.height;

    data.resize(height, width);

    // copy data
    const float* fdata = reinterpret_cast<const float*>(source_data.data.data());

    for (unsigned int y = 0; y < height; y++) {
        for (unsigned int x = 0; x < width; x++) {
            //data(y, x) = source_data.data[(y*width+x)*3] / 255.0f;
            unsigned int i = y*width+x;

            // invalid data is exactly '1'
            // these are disparity values, not actual depth data!
            // TODO: use disparity value!
            if (fdata[i] == 1.0f)
                data(y, x) = std::numeric_limits<float>::quiet_NaN();
            else
                data(y, x) = 1.0f / fdata[i];
        }
    }

    updateImageFromMatrix(colormap);
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
void ImageData::updateImageFromMatrix(const Colormap::Colormap& colormap)
{
    matrix_to_image(data, sf_image, colormap);
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
void ImageData::matrix_to_image(const Eigen::MatrixXf& source, sf::Image& dest, const Colormap::Colormap& colormap)
{
    // cout << "converting matrix to image ...";

    // initialize and clear image
    dest.create(source.cols(), source.rows());

    float near = source.minCoeff();
    float far  = source.maxCoeff();

    // convert matrix data into grayscale image
    for (unsigned int y = 0; y < source.rows(); y++) {
        for (unsigned int x = 0; x < source.cols(); x++) {
            dest.setPixel(x, y, colormap(source(y,x), near, far));
        }
    }

    // cout << "done." << endl;
}
///////////////////////////////////////////////////////////////////////////////
void ImageData::normalizeTo(float near, float far)
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
    float v2 = data(floor(pos.y()),  ceil(pos.x()));
    float v3 = data( ceil(pos.y()), floor(pos.x()));
    float v4 = data( ceil(pos.y()),  ceil(pos.x()));

    // weight (how close to img[ceil(x), ceil(y)] are we?
    float sx = pos.x() - floor(pos.x());
    float sy = pos.y() - floor(pos.y());

    assert(sx >= 0); assert(sx < 1);
    assert(sy >= 0); assert(sy < 1);

    return v1 * ( (1-sx) * (1-sy) )
         + v2 * (    sx  * (1-sy) )
         + v3 * ( (1-sx) *    sy  )
         + v4 * (    sx  *    sy  );
}
///////////////////////////////////////////////////////////////////////////////
// WARNING: This function has a very slight bias, as points with integer
// coordinates (i.e. falling exactly on a pixel center) are not handled
// specially. Instead, the gradient towards the next (right/lower) pixel is
// taken.
Eigen::Matrix<float, 1, 2> ImageData::sampleDiff(Eigen::Vector2f pos) const
{
    float fx = floor(pos.x()), fy = floor(pos.y());

    if (fx > width-2) {
        cout << "fail: " << pos.transpose() << " --> " << fx << " " << fy << endl;
        cout << "W: " << width << " H: " << height << endl;
    }

    // make sure, all four neighbouring points lie inside the image
    assert(fx >= 0);
    assert(fy >= 0);
    assert(fx <= width-2);
    assert(fy <= height-2);

    // remember: data[row, col] = data[y, x]!
    float v1 = data(fy,   fx);
    float v2 = data(fy,   fx+1);
    float v3 = data(fy+1, fx);
    float v4 = data(fy+1, fx+1);

    // weight (how close to img[ceil(x), ceil(y)] are we?
    float sx = pos.x() - fx;
    float sy = pos.y() - fy;

    float dx1 = v2 - v1,  dy1 = v3 - v1;
    float dx2 = v4 - v3,  dy2 = v4 - v2;

    Eigen::Matrix<float, 1, 2> J;

    J(0) = dx1 * (1-sy)  +  dx2 * sy;
    J(1) = dy1 * (1-sx)  +  dy2 * sx;

    return J;
}
///////////////////////////////////////////////////////////////////////////////
void ImageData::downsample2(const Colormap::Colormap& colormap)
{
    for (int r = 0; r < data.rows()/2; r++) {
        for (int c = 0; c < data.cols()/2; c++) {
            data(r,c) = (data(2*r,   2*c)
                      +  data(2*r+1, 2*c)
                      +  data(2*r,   2*c+1)
                      +  data(2*r+1, 2*c+1)) / 4;
        }
    }

    width  /= 2;
    height /= 2;

    data.conservativeResize(height, width);

    this->updateImageFromMatrix(colormap);
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void drawImageAt(const sf::Image& img, const sf::Vector2f& pos, sf::RenderTarget& target, const std::string& label, const sf::Font* font)
{
    sf::Texture t;
    t.loadFromImage(img);

    sf::Sprite s;
    s.setTexture(t);
    s.setPosition(pos.x, pos.y);

    target.draw(s);

    if (!label.empty() && font) {
        sf::Text t;
        t.setFont(*font);
        t.setCharacterSize(12);
        t.setString(label);
        t.setPosition(pos.x+2, pos.y+img.getSize().y-t.getCharacterSize()-2);
        target.draw(t);
    }
}
///////////////////////////////////////////////////////////////////////////////
void drawMatrixAt(const Eigen::MatrixXf& mat, const sf::Vector2f& pos, sf::RenderTarget& target, const Colormap::Colormap& colormap, const std::string& label, const sf::Font* font)
{
    ImageData i;
    i.loadFromMatrix(mat, colormap);

    i.drawAt(target, pos);

    if (!label.empty() && font) {
        sf::Text t;
        t.setFont(*font);
        t.setCharacterSize(12);
        t.setString(label);
        t.setPosition(pos.x+2, pos.y+i.getHeight()-t.getCharacterSize()-2);
        target.draw(t);
    }
}
///////////////////////////////////////////////////////////////////////////////
