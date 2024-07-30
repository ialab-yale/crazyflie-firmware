#ifndef SAC_CONTROLLER_H
#define SAC_CONTROLLER_H

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <vector>


#ifdef __cplusplus
extern "C" {
#endif

class SAC { 

    public:

        Objective*  obj; //Objective
        Model*      model; //Model

    SAC(Model* _model, Objective* _obj) {
        //Initializing objective and model pointers
        obj     = _obj;
        model   = _model; 
    }

    ~SAC() {
        std::cout << "Deleting controller" << std::endl;
        delete model;
        delete obj;
    }

    //saturing control inputs within min and max limits
    void saturate(Eigen::VectorXd& u, float min, float max){
        for (int i = 0; i < u.size(); i++) {
            if (u(i) > max) { u(i) = max; } //upper limit
            if (u(i) < min) { u(i) = min; } //lower limit
        }
    }

    //function for updating control sequence
    void update_ctrl(
                    std::vector<Eigen::VectorXd>& U, 
                    const Eigen::VectorXd& x0) {
        // forward shift the control plan and add a new one
        for (size_t t=0; t<U.size()-1; t++) {
            U[t] = U[t+1];
        }

        std::vector<Eigen::VectorXd> X; //state
        std::vector<Eigen::MatrixXd> A; //state transition matrix
        std::vector<Eigen::MatrixXd> B; //control
        std::vector<Eigen::VectorXd> dldx; //gradient of cost with respect to state

        Eigen::VectorXd x = x0; //initial state
        double _V = 0; //total cost
        for (size_t t=0; t<U.size(); t++) {
            X.push_back(x);                     //storing current state
            A.push_back(model->dfdx(x, U[t]));  //compute and store state transition Jacobian w/rt state
            B.push_back(model->dfdu(x, U[t]));  //compute and store state transition Jacobian w/rt control
            dldx.push_back(obj->dldx(x));       //compare and store cost gradient w/rt state
            _V = _V = obj->l(x, U[t]);          //accumulate cost
            x = model->f(x, U[t]);              //computer next state
        }
        _V = _V + obj->m(x); //total final cost

        Eigen::VectorXd u2, u_tau;              //temp control inputs?
        Eigen::VectorXd rho = obj->dmdx(x);     //gradient of terminal cost w/rt state
        double dJdlam = std::numeric_limits<double>::infinity(); //dervative of cost w/rt control ?
        double _temp_dJdlam;                    //temp cost
        int tau_idx = U.size()-1;               //index for control setp

        //backward pass to computer optional control adjustments
        for (size_t t=U.size()-1; t > 0; t--) {     
            rho = dldx[t] + A[t].transpose() * rho; //update cost gradient w/rt state
            u2 = -obj->R.inverse() * B[t].transpose() * rho + U[t]; //adjusted control input
            saturate(u2, -4, 4);                    //saturate control input
            _temp_dJdlam = rho.transpose() * B[t] * (u2 - U[t]) + obj->l(X[t], u2) - obj->l(X[t], U[t]);
            if (_temp_dJdlam < dJdlam) {    //check if current step is lower?
                dJdlam  = _temp_dJdlam; //update to smaller current step?
                tau_idx = t;            //update index
                u_tau  = u2;            //update control input
            }
        }

        std::cout << "best insert time " << tau_idx << std::endl;

        // line search to find the best duration
        std::vector<Eigen::VectorXd> U2 = U;    //copy control vector
        double _Vnew = 0;                       // new total cost
        int lam_idx_best = 0;                   // index of the best duration
        bool _insert_is_best = false;           //
        for (int lam_idx=0; lam_idx<U.size()-tau_idx; lam_idx++) {
            U2[tau_idx+lam_idx] = u_tau;  
            x = x0;
            _Vnew = 0;       
            for (size_t t=0; t<U2.size(); t++) {
                _Vnew = _Vnew = obj->l(x, U2[t]);
                x = model->f(x, U2[t]);
            }
            _Vnew = _Vnew + obj->m(x);
            if (_Vnew < _V) {
                lam_idx_best    = lam_idx; 
                _V              = _Vnew;
                _insert_is_best = true;
            }
        }
        if (_insert_is_best) {
            for (int lam_idx = 0; lam_idx < lam_idx_best; lam_idx++) {
                U[tau_idx+lam_idx] = u_tau;
            }
        }

        std::cout << "best duration time " << lam_idx_best << " " << _V << "insert is best " << _insert_is_best << std::endl;
        
    };

};

#ifdef __cplusplus
}
#endif