#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <assert.h>

bool judgeContourByArea(const std::vector<cv::Point> &contour)
{
    if (cv::contourArea(contour) > 7000) // 舍弃小轮廓
        return true;
    return false;
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
    cv::drawContours(drawer, contours, id, COLOR_LIST[depth % 3], 3);
    cv::rectangle(drawer, rect, COLOR_LIST[2], 2);
    for (int i = hierachy[id][2]; i + 1; i = hierachy[i][0])
    {
        dfs(drawer, contours, hierachy, i, depth + 1); // 向内部的子轮廓递归
    }
}

int main(int argc, char **argv)
{
    cv::Mat src = cv::imread("../apple.png");
    assert(src.channels() == 3);

    cv::Mat hsv;
    cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
    cv::Mat hsv_part_red;
    cv::Mat hsv_part_green;
    cv::Mat hsv_part_orange;
    cv::Mat drawer = cv::Mat::zeros(cv::Size(src.cols, src.rows), CV_8UC3);

    // cv::inRange(hsv, cv::Scalar(0, 43, 46), cv::Scalar(25, 255, 255), hsv_part_red);
    cv::inRange(hsv, cv::Scalar(0, 140, 60), cv::Scalar(25, 255, 255), hsv_part_red);
    cv::inRange(hsv, cv::Scalar(60, 100, 60), cv::Scalar(140, 255, 255), hsv_part_green);
    cv::inRange(hsv, cv::Scalar(156, 43, 46), cv::Scalar(180, 255, 255),
                hsv_part_orange);
    cv::Mat hsv_result = hsv_part_orange | hsv_part_red - hsv_part_green;
    cv::Mat blured_img;
    cv::medianBlur(hsv_result, blured_img, 7);

    cv::imshow("hsv_result", hsv_result);
    cv::imshow("blured_img", blured_img);
    cv::Mat morphology_mat;
    int morph_size = 20;
    int morph_close_size = 20;
    cv::Mat element_open = cv::getStructuringElement(1, cv::Size(2 * morph_size + 1, 2 * morph_size + 1),
                                                     cv::Point(-1, -1));
    cv::morphologyEx(blured_img, morphology_mat, cv::MORPH_OPEN, element_open);

    cv::Mat element_close = cv::getStructuringElement(1, cv::Size(2 * morph_close_size + 1, 2 * morph_close_size + 1),
                                                      cv::Point(-1, -1));
    cv::Mat morphology_result;
    cv::morphologyEx(morphology_mat, morphology_result, cv::MORPH_CLOSE, element_close);
    cv::imshow("morphlogy_result", morphology_result);

    std::vector<std::vector<cv::Point>> contour;
    std::vector<cv::Vec4i> hierachy;
    cv::findContours(morphology_result, contour, hierachy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
    for (int i = 0; i + 1; i = hierachy[i][0])
        dfs(src, contour, hierachy, i, 0);
    cv::imshow("contour", src);

    cv::waitKey(0);

    return 0;
}