#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <assert.h>

bool judgeContourByArea(const std::vector<cv::Point> &contour)
{
    if (cv::contourArea(contour) < 2000) // 舍弃小轮廓
        return false;
    double contour_area, rect_area;
    cv::RotatedRect rotate_rect = cv::minAreaRect(contour);
    rect_area = rotate_rect.size.area();
    contour_area = cv::contourArea(contour);
    if (rect_area > 1.4 * contour_area) // 轮廓面积约束
        return false;
    return true;
}

void dfs(cv::Mat &drawer,
         const std::vector<std::vector<cv::Point>> &contours,
         const std::vector<cv::Vec4i> &hierachy,
         const int &id,
         const int &depth)
{
    if (id == -1)
        return;
    static cv::Scalar COLOR_LIST[3] = {{220, 20, 20}, {20, 220, 20}, {20, 20, 220}};
    if (!judgeContourByArea(contours[id]))
        return;
    cv::Rect rect = cv::boundingRect(contours[id]);
    cv::RotatedRect rotate_rect = cv::minAreaRect(contours[id]);
    cv::Point2f vertices[4];
    rotate_rect.points(vertices); // 从RotatedRect类中提取出角点
    for (int i = 0; i < 4; i++)
    {
        cv::line(drawer, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 0, 255), 4);
    }
    // cv::drawContours(drawer, contours, id, COLOR_LIST[depth % 3], 2);
    for (int i = hierachy[id][2]; i + 1; i = hierachy[i][0])
    {
        dfs(drawer, contours, hierachy, i, depth + 1); // 向内部的子轮廓递归
    }
}

using namespace cv;
int main(int argc, char **argv)
{
    Mat src[5];
    Mat hsv[5];
    Mat morph_result[5];
    Mat result[5];

    static std::string index[5] = {"001", "002", "003", "004", "005"};
    for (int i = 0; i < 5; ++i)
    {
        src[i] = imread("../plates/" + index[i] + ".jpg");
        assert(src[i].channels() == 3);
        result[i] = src[i].clone();
    }

    for (int i = 0; i < 5; ++i)
    {
        Mat &mat = hsv[i];
        cvtColor(src[i], mat, COLOR_BGR2HSV);
        Mat hsv_part_blue;
        cv::inRange(mat, cv::Scalar(100, 140, 75), cv::Scalar(124, 255, 255), hsv_part_blue);

        cv::Mat hsv_result = hsv_part_blue;
        cv::Mat blured_img;
        cv::medianBlur(hsv_result, blured_img, 5);
        // cv::imshow("blured_img_" + index[i], blured_img);

        int morph_size_open = 2;
        int morph_size_close = 20;
        cv::Mat morphology_mat;
        cv::Mat element_open = cv::getStructuringElement(1, cv::Size(2 * morph_size_open + 1, 2 * morph_size_open + 1),
                                                         cv::Point(-1, -1));
        cv::morphologyEx(blured_img, morphology_mat, cv::MORPH_OPEN, element_open);
        // cv::imshow("morphlogy_open_" + index[i], morphology_mat);

        cv::Mat element_close = cv::getStructuringElement(1, cv::Size(2 * morph_size_close + 1, 2 * morph_size_close + 1),
                                                          cv::Point(-1, -1));
        cv::morphologyEx(morphology_mat, morph_result[i], cv::MORPH_CLOSE, element_close);
        // cv::imshow("result_" + index[i], morph_result[i]);
    }

    for (int i = 0; i < 5; ++i)
    {
        std::vector<std::vector<cv::Point>> contour;
        std::vector<cv::Vec4i> hierachy;
        cv::findContours(morph_result[i], contour, hierachy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
        // cv::Mat drawer = cv::Mat::zeros(cv::Size(morph_result[i].cols, morph_result[i].rows), CV_8UC3);
        // for (int j = 0; j + 1; j = hierachy[j][0])
        // dfs(drawer, contour, hierachy, j, 0);
        // cv::imshow("src" + index[i], drawer);
        for (int j = 0; j + 1; j = hierachy[j][0])
            dfs(result[i], contour, hierachy, j, 0);
        cv::imshow("result" + index[i], result[i]);
    }

    cv::waitKey(0);
}