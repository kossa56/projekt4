#pragma once

#include <iostream>
#include <cmath>
#include <Eigen/Dense>

typedef unsigned int u_int;

/* Discrete LQR */
Eigen::MatrixXf LQR(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B, const Eigen::MatrixXf &Q, const Eigen::MatrixXf &R, double eps = 1e-5, u_int max_iter = 10000);