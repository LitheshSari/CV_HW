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

int main(int argc, char **argv)
{
}