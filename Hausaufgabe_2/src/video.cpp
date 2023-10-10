#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Dense>
#include <iostream>

#define PI acos(-1)

cv::Mat img_show;

class Drawer
{
public:
    Drawer()
    {
        plane_ = cv::Mat::zeros(cv::Size(1200, 800), CV_8UC1);
    }

    void imshow()
    {
        cv::imshow("result", plane_);
    }

    cv::Point2d getPoint()
    {
        return cv_point_;
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
        rotation_matrix_ = quaternion.matrix().transpose().cast<double>();
        transformation_matrix_(3, 3) = 1;
        transformation_matrix_.block(0, 0, 3, 3) = rotation_matrix_;
        transformation_matrix_.block(0, 3, 3, 1) = -rotation_matrix_ * Eigen::Vector3d(2., 2., 2.);
        conbined_matrix_ = inner_matrix_ * transformation_matrix_;
    }

    void updateQuaternion(double angle)
    {
        Eigen::Quaterniond quaternion_cam(-0.5, 0.5, 0.5, -0.5);
        Eigen::Vector3d rotate_axis = quaternion_cam.matrix().cast<double>() * Eigen::Vector3d(0, 1., 0);
        rotate_axis = sin(angle / 2) * rotate_axis;

        Eigen::Quaterniond new_quaternion(cos(angle / 2), rotate_axis.x(), rotate_axis.y(), rotate_axis.z());
        Eigen::Matrix4d transformation_matrix = transformation_matrix_;
        Eigen::Matrix3d rotation_matrix = rotation_matrix_.cast<double>() *
                                          (new_quaternion.matrix().transpose().cast<double>());
        transformation_matrix.block(0, 0, 3, 3) = rotation_matrix;

        cam_matrix_ = inner_matrix_ * transformation_matrix.cast<double>();
    }

    Eigen::Matrix<double, 3, 4> getCamMatrix()
    {
        return cam_matrix_;
    }
    Eigen::Matrix<double, 3, 4> getMatrix()
    {
        return conbined_matrix_;
    }

private:
    Eigen::Matrix<double, 3, 4> inner_matrix_;
    Eigen::Matrix3d rotation_matrix_;
    Eigen::Matrix4d transformation_matrix_;
    Eigen::Matrix<double, 3, 4> conbined_matrix_;
    Eigen::Matrix<double, 3, 4> cam_matrix_;
};

int main(int argc, char **argv)
{

    int kCount = 200;
    double target = -PI / 2;
    double step_length = target / kCount;
    Camera_Sim camera_sim;
    cv::VideoWriter writer("JiaoLoong.avi",
                           cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                           50, cv::Size(1200, 800), false);
    for (int j = 0; j < kCount; ++j)
    {
        Drawer drawer;
        target -= step_length;
        camera_sim.updateQuaternion(target);

        Eigen::Matrix<double, 3, 4> conbined_matrix = camera_sim.getCamMatrix();

        freopen("../points.txt", "r", stdin);
        int count;
        std::cin >> count;

        img_show = cv::Mat::zeros(cv::Size(1200, 800), CV_8UC1);

        for (int i = 0; i < count; ++i)
        {
            double x, y, z;
            std::cin >> x >> y >> z;
            Eigen::Vector4d pos_vec;
            pos_vec << x, y, z, 1.;

            Eigen::Vector3d point = conbined_matrix * pos_vec;
            drawer.drawPoint(point);
            cv::Point2d point2d = drawer.getPoint();
            cv::circle(img_show, point2d, 2, cv::Scalar{255, 255, 255}, -1);
        }
        fclose(stdin);

        drawer.imshow();
        writer << img_show;
        cv::waitKey(50);
    }

    cv::waitKey(-1);
    writer.release();
}