#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Dense>
#include <iostream>

class Drawer
{
public:
    Drawer()
    {
        plane_ = cv::Mat::zeros(cv::Size(1300, 1000), CV_8UC1);
    }

    void imshow()
    {
        cv::imshow("result", plane_);
    }

    void drawPoint(Eigen::Vector3d point)
    {
        point /= point(2, 0);
        cv_point_.x = point(0, 0);
        cv_point_.y = point(1, 0);
        cv::circle(plane_, cv_point_, 2, cv::Scalar{255, 255, 255}, -1);
    }

    void imwrite()
    {
        cv::imwrite("JiaoLoong.jpg", plane_);
    }

private:
    cv::Mat plane_;
    cv::Point2d cv_point_;
};

class Camera_Sim
{
public:
    Camera_Sim()
        : transformation_matrix_(Eigen::Matrix4d::Zero())
    {
        inner_matrix_ << 400., 0., 190., 0.,
            0., 400., 160., 0.,
            0., 0., 1., 0.;
        Eigen::Quaterniond quaternion(-0.5, 0.5, 0.5, -0.5);
        Eigen::Matrix3d rotation_matrix = quaternion.matrix().transpose().cast<double>();
        transformation_matrix_(3, 3) = 1;
        transformation_matrix_.block(0, 0, 3, 3) = rotation_matrix;
        transformation_matrix_.block(0, 3, 3, 1) = -rotation_matrix * Eigen::Vector3d(2., 2., 2.);
        conbined_matrix_ = inner_matrix_ * transformation_matrix_;
    }

    void updateQuaternion(Eigen::Quaterniond quaternion)
    {
        Eigen::Matrix3d rotation_matrix = quaternion.matrix().transpose().cast<double>();
        transformation_matrix_.block(0, 0, 3, 3) = rotation_matrix;
        transformation_matrix_.block(0, 3, 3, 1) = -rotation_matrix * Eigen::Vector3d(2., 2., 2.);
        conbined_matrix_ = inner_matrix_ * transformation_matrix_;
    }

    Eigen::Matrix<double, 3, 4> getMatrix()
    {
        return conbined_matrix_;
    }

private:
    Eigen::Matrix<double, 3, 4> inner_matrix_;
    Eigen::Matrix4d transformation_matrix_;
    Eigen::Matrix<double, 3, 4> conbined_matrix_;
};

class Quaternion_Generator
{
public:
    Quaternion_Generator()
    {
        axis_ = Eigen::Vector3d(0.577, 0.577, -0.577);
    }
    void update(double step)
    {
        Eigen::Vector3d axis_temp = sin(step / 2) * axis_;
        quaternion_ = Eigen::Quaterniond(cos(step / 2), axis_temp.x(), axis_temp.y(), axis_temp.z());
    }
    Eigen::Quaterniond getQuaternion()
    {
        return quaternion_;
    }

private:
    Eigen::Quaterniond quaternion_;
    Eigen::Vector3d axis_;
};

int main(int argc, char **argv)
{
    cv::VideoWriter writer("JiaoLoong.avi",
                           cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                           50, cv::Size(1300, 1000), true);
    int kCount = 200;
    constexpr double PI = 3.14159265358;
    double target = -PI * 2 / 3;
    double step_length = target / kCount;
    Quaternion_Generator quad_generator;
    Camera_Sim camera_sim;

    for (int j = 0; j <= kCount; ++j)
    {
        Drawer drawer;
        double step = target / kCount * j;
        quad_generator.update(step);

        Eigen::Quaterniond quaternion = quad_generator.getQuaternion();
        camera_sim.updateQuaternion(quaternion);
        Eigen::Matrix<double, 3, 4> conbined_matrix = camera_sim.getMatrix();

        freopen("../points.txt", "r", stdin);
        int count;
        std::cin >> count;

        for (int i = 0; i < count; ++i)
        {
            double x, y, z;
            std::cin >> x >> y >> z;
            Eigen::Vector4d pos_vec;
            pos_vec << x, y, z, 1.;

            Eigen::Vector3d point = conbined_matrix * pos_vec;
            drawer.drawPoint(point);
        }

        drawer.imshow();
        cv::waitKey(50);
    }

    cv::waitKey(-1);
}