#include <SFML/Graphics.hpp>
#include <Eigen/Dense>

Eigen::MatrixXf* image_to_matrix(const sf::Image& image);
sf::Image*       matrix_to_image(const Eigen::MatrixXf& matrix);
