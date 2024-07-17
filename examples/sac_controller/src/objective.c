#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <Eigen.h>

double l(const Eigen::VectorXd&, const Eigen::VectorXd&){

} //cost function
double m(const Eigen::VectorXd&){

}; // terminal cost function

Eigen::VectorXd dldx(const Eigen::VectorXd&){

} //gradient of cost w/rt state

Eigen::VectorXd dmdx(const Eigen::VectorXd&){

} //gradient of terminal cost w/rt state

double cost2go(const std::vector<Eigen::VectorXd>& states, const std::vector<Eigen::VectorXd>& controls){

    return 0.0;
} // total cost-to-go