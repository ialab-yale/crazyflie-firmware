#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <Eigen.h>
#include "model.h"

using namespace Eigen;

Eigen::VectorXd model_f(const Eigen::VectorXd&, const Eigen::VectorXd&){

} //state transition
Eigen::MatrixXd model_dfdx(const Eigen::VectorXd&, const Eigen::VectorXd&){

} //Jacobian of state transition /w r.t. state
Eigen::MatrixXd model_dfdu(const Eigen::VectorXd&, const Eigen::VectorXd&){
    
} //jacobian of state transition w/ r.t. control