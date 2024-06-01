#include "lqr.h"

/* Discrete LQR */
Eigen::MatrixXf LQR(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B, const Eigen::MatrixXf &Q, const Eigen::MatrixXf &R, double eps, u_int max_iter) {
    Eigen::MatrixXf A_T = A.transpose();
    Eigen::MatrixXf B_T = B.transpose();

    Eigen::MatrixXf P = Q;
    Eigen::MatrixXf P_old = P;
    Eigen::MatrixXf delta = Eigen::MatrixXf::Zero(P.rows(), P.cols());

    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(A.rows(), A.cols());

    bool converged = false;

    for (u_int i = 0; i < max_iter; ++i) {

        P = A_T * P * A - A_T * P * B * (R + B_T * P * B).inverse() * B_T * P * A + Q;

        delta = P - P_old;
        if (fabs(delta.maxCoeff()) < eps) {
            converged = true;
            break;
        }
        P_old = P;
    }

    if (converged) {
        std::cout << "LQR: convergence reached.";
    } else {
        std::cout << "LQR: max iterations limit reached.";
    }

    K = (R + B_T * P * B).inverse() * (B_T * P * A);

    return K;
}