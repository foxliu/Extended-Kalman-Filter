#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    /**
    TODO:
      * Calculate the RMSE here.
    */
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    unsigned long est_size = estimations.size();
    unsigned long ground_size = ground_truth.size();

    if (est_size != ground_size || est_size == 0) {
        cout << "Invalid estimation or ground_truth data" << endl;
        return rmse;
    }

    for (int i = 0; i < est_size; ++i) {
        VectorXd residual = estimations[i] - ground_truth[i];

        residual = residual.array() * residual.array();
        rmse += residual;
    }


    rmse = rmse / est_size;
    rmse = rmse.array().sqrt();
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
    /**
    TODO:
      * Calculate a Jacobian here.
    */

    // x_state length is 4, the member is: px, py, vx, vy
    MatrixXd Hj(3, 4);
    double px, py, vx, vy;
    px = x_state[0];
    py = x_state[1];
    vx = x_state[2];
    vy = x_state[3];

    double c1 = px * px + py * py;
    double c2 = sqrt(c1);
    double c3 = c1 * c2;

    if (c1 < 0.0001) {
        cout << "CalculateJacobian () - Error - Division by Zero" << endl;
        return Hj;
    }

    Hj << px / c2, py / c2, 0, 0,
            -py / c1, px / c1, 0, 0,
            py * (vx * py - vy * px) / c3, px * (vy * px - vx * py) / c3, px / c1, py / c1;
    return Hj;
}
