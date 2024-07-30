#ifndef MODEL_HPP
#define MODEL_HPP

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <vector>
#include <Eigen.h>

using namespace Eigen;

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int nx;
    int nu;
} Model;

Eigen::VectorXd model_f(const Eigen::VectorXd&, const Eigen::VectorXd&); //state transition

Eigen::MatrixXd model_ dfdx(const Eigen::VectorXd&, const Eigen::VectorXd&); //Jacobian of state transition /w r.t. state

Eigen::MatrixXd model_dfdu(const Eigen::VectorXd&, const Eigen::VectorXd&); //jacobian of state transition w/ r.t. control

#ifdef __cplusplus
}
#endif