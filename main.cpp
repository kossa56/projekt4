#include <cmath>
#include <matplot/matplot.h>
#include <Eigen/Dense>
#include <iostream>

int main() {
    using namespace matplot;

    /* 1. Plot using std::vector, iota and transform */
    std::vector<double> t = iota(0, pi / 50, 10 * pi);
    std::vector<double> st = transform(t, [](auto x) { return sin(x); });
    std::vector<double> ct = transform(t, [](auto x) { return cos(x); });

    auto l_1 = plot3(st, ct, t);
    show();


    /* 2. Plot using Eigen elementwise operations */
    int n_points = 1000;
    Eigen::VectorXf x = Eigen::VectorXf::LinSpaced(n_points, 0, 10 * pi);
    Eigen::VectorXf sx = x.array().sin();
    Eigen::VectorXf cx = x.array().cos();

    auto l_2 = plot3(sx, cx, x);
    show();

    return 0;
}