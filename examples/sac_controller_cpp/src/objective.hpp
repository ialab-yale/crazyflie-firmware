#ifndef OBJECTIVE_HPP
#define OBJECTIVE_HPP

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <Eigen.h>
#include <vector>


#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    Eigen::MatrixXd R;
} Objective;

class Objective {
    public: 
        Eigen::MatrixXd R; //control inpts?
    Objective() {;}
    virtual ~Objective() {;}
    //virtual methods decalred for derived class
    virtual double l(const Eigen::VectorXd&, const Eigen::VectorXd&) = 0; //cost function
    virtual double m(const Eigen::VectorXd&) = 0; // terminal cost function
    virtual Eigen::VectorXd dldx(const Eigen::VectorXd&) = 0; //gradient of cost w/rt state
    virtual Eigen::VectorXd dmdx(const Eigen::VectorXd&) = 0; //gradient of terminal cost w/rt state
    virtual double cost2go(
                    const std::vector<Eigen::VectorXd>&, 
                    const std::vector<Eigen::VectorXd>&) = 0; // total cost-to-go
};

#ifdef __cplusplus
}
#endif