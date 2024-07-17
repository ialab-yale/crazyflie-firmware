#ifndef OBJECTIVE_H
#define OBJECTIVE_H

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <Eigen.h>
#include <vector>

typedef struct {
    Eigen::MatrixXd R;
} Objective;

//functions methods decalred for derived class
double l(const Eigen::VectorXd&, const Eigen::VectorXd&); //cost function
double m(const Eigen::VectorXd&); // terminal cost function
Eigen::VectorXd dldx(const Eigen::VectorXd&); //gradient of cost w/rt state
Eigen::VectorXd dmdx(const Eigen::VectorXd&); //gradient of terminal cost w/rt state
double cost2go(const std::vector<Eigen::VectorXd>& states, const std::vector<Eigen::VectorXd>& controls); // total cost-to-go

#endif