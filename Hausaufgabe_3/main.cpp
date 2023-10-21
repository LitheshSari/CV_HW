//
// 2023 JiaoLoong CV enter examination
//
#include <iostream>
#include <chrono>
#include <cmath>
#include "kalman/kalman.h"

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

int main()
{
    int kCount = 0;
    srand(time(NULL));
    std::vector<double> data;

    freopen("../dollar.txt", "r", stdin);

    cv::Mat img;
    img = cv::Mat::zeros(cv::Size(800, 600), CV_8UC3);

    double temp;
    while (std::cin >> temp)
    {
        cv::circle(img, cv::Point(kCount * 20, temp * 200 - 1100), 2, cv::Scalar{0, 255, 0}, 2);
        data.push_back(temp);
        kCount++;
    }

    const int V_X = 2;
    const int V_Z = 1;

    Eigen::Matrix<double, V_X, 1> x_k1;
    Eigen::Matrix<double, V_X, V_X> A;
    Eigen::Matrix<double, V_Z, V_X> H;
    Eigen::Matrix<double, V_X, V_X> R;
    Eigen::Matrix<double, V_Z, V_Z> Q;

    x_k1 << data[0], 0;
    A << 1, 1,
        0, 1;
    H << 1, 0;

    R << 2, 0,
        0, 2;
    Q << 10;

    Kalman<V_Z, V_X> kalman_0(A, H, R, Q, x_k1);

    for (int i = 1; i < kCount; ++i)
    {
        double delta = data[i] - data[i - 1];
        Eigen::Matrix<double, V_Z, 1> z_k;
        z_k << data[i];
        x_k1 = kalman_0.update(z_k);

        // std::cout << "-------------" << std::endl;
        // std::cout << i << '\n'
        //<< x_k1.transpose() << std::endl;
    }

    cv::line(img,
             cv::Point(0, data[0] * 200 - 1100), cv::Point(kCount * 20, (data[0] + 30 * x_k1(1, 0)) * 200 - 1100),
             cv::Scalar{0, 255, 0});
    cv::imshow("result", img);
    cv::waitKey(-1);
    return 0;
}