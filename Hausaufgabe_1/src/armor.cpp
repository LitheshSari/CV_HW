#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <assert.h>
#include <unordered_map>

static cv::Scalar COLOR_LIST[3] = {{220, 20, 20}, {20, 220, 20}, {20, 20, 220}};

bool judgeContour(const std::vector<cv::Point> &contour, const cv::RotatedRect &rotate_rect)
{
    if (cv::contourArea(contour) < 350) // 舍弃小轮廓
        return false;
    if (cv::contourArea(contour) > 2000)
        return false;

    double length, width;
    cv::Point2f center;
    center = rotate_rect.center;
    if (center.y < 500)
        return false;

    double contour_area, rect_area;
    rect_area = rotate_rect.size.area();
    contour_area = cv::contourArea(contour);
    if (rect_area > 1.6 * contour_area) // 轮廓面积约束
        return false;
    return true;
}

/*
 * @brief recursive search
 */
void dfs(cv::Mat &drawer,
         const std::vector<std::vector<cv::Point>> &contours,
         const std::vector<cv::Vec4i> &hierachy,
         const int &id,
         const int &depth,
         std::unordered_map<int, cv::RotatedRect> &contours_map,
         std::vector<cv::Point2i> &armors)
{
    if (id == -1)
        return;
    // assuming that armor has only one layer
    if (depth > 0)
        return;
    cv::RotatedRect rotate_rect = cv::minAreaRect(contours[id]);
    if (!judgeContour(contours[id], rotate_rect))
        return;

    for (auto &it : contours_map)
    {
        const auto &cur_center = rotate_rect.center;
        cv::Point2d center = it.second.center;
        double length = sqrt((center.y - cur_center.y) * (center.y - cur_center.y) + (center.x - cur_center.x) * (center.x - cur_center.x));

        // Hardcode everywhere :(
        if (length < 100.0f && (center.y - cur_center.y) < 20.0f)
        {
            cv::line(drawer, center, cur_center, cv::Scalar(0, 0, 255), 4);
            armors.emplace_back(cv::Point2i(id, it.first));
        }
    }
    contours_map.emplace(std::make_pair(id, rotate_rect));

    cv::drawContours(drawer, contours, id, COLOR_LIST[depth % 3], 3);

    cv::Point2f vertices[4];
    rotate_rect.points(vertices); // 从RotatedRect类中提取出角点
    for (int i = 0; i < 4; ++i)
    {
        cv::line(drawer, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 0, 255), 4);
    }
    for (int i = hierachy[id][2]; i + 1; i = hierachy[i][0])
    {
        dfs(drawer, contours, hierachy, i, depth + 1, contours_map, armors); // 向内部的子轮廓递归
    }
}

void process_armor(cv::Mat &drawer,
                   std::unordered_map<int, cv::RotatedRect> &contours_map,
                   const std::vector<cv::Point2i> &armors)
{
    for (const auto &it : armors)
    {
        cv::RotatedRect &first_contour = contours_map[it.x];
        cv::RotatedRect &second_contour = contours_map[it.y];
        if (second_contour.center.x < first_contour.center.x)
        {
            std::swap(first_contour, second_contour);
        }
        cv::Point2f center_vector = second_contour.center - first_contour.center;
        std::vector<cv::Point2f> points;
        points.reserve(4);
        cv::Point2f left_vertices[4];
        cv::Point2f right_vertices[4];
        first_contour.points(left_vertices);
        second_contour.points(right_vertices);

        for (int i = 0; i < 4; ++i)
        {
            cv::Point2f left_vector = left_vertices[i] - first_contour.center;
            if (left_vector.x * center_vector.x + left_vector.y * center_vector.y < 0)
                points.emplace_back(left_vertices[i]);
            cv::Point2f right_vector = right_vertices[i] - second_contour.center;
            if (right_vector.x * center_vector.x + right_vector.y * center_vector.y > 0)
                points.emplace_back(right_vertices[i]);
        }

        for (int i = 0; i < 4; ++i)
        {
            for (int j = i; j < 4; ++j)
            {
                cv::line(drawer, points[i], points[j], cv::Scalar(0, 255, 0), 4);
            }
        }
    }
}

int main(int argc, char **argv)
{
    cv::VideoCapture capture("../armor.mp4");
    cv::Mat src;
    capture.read(src);
    cv::VideoWriter writer("../armor_output.avi",
                           cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                           50, cv::Size(src.cols, src.rows), true);
    while (!src.empty())
    {
        cv::Mat hsv, hsv_part_red, hsv_part_white;
        cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
        /*
         *  hsv combination
         */
        // cv::inRange(hsv, cv::Scalar(0, 135, 30), cv::Scalar(25, 255, 255), hsv_part_red);
        cv::inRange(hsv, cv::Scalar(0, 135, 65), cv::Scalar(25, 255, 255), hsv_part_red);
        // cv::inRange(hsv, cv::Scalar(0, 0, 221), cv::Scalar(180, 30, 255), hsv_part_white);
        cv::inRange(hsv, cv::Scalar(0, 0, 221), cv::Scalar(180, 50, 255), hsv_part_white);
        // cv::Mat hsv_result = hsv_part_red;
        cv::Mat hsv_result = hsv_part_white | hsv_part_red;
        cv::Mat blured_img;
        cv::medianBlur(hsv_result, blured_img, 3);

        /*
         *  Morphology
         */
        cv::Mat morphology_close;
        int morph_size_close = 10;
        cv::Mat element_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * morph_size_close + 1, 2 * morph_size_close + 1),
                                                          cv::Point(-1, -1));
        cv::morphologyEx(blured_img, morphology_close, cv::MORPH_CLOSE, element_close);

        cv::Mat morphology_open;
        int morph_size_open = 1;
        cv::Mat element_open = cv::getStructuringElement(1, cv::Size(2 * morph_size_open + 1, 2 * morph_size_open + 1),
                                                         cv::Point(-1, -1));
        cv::morphologyEx(morphology_close, morphology_open, cv::MORPH_OPEN, element_open);

        std::vector<std::vector<cv::Point>> contour;
        std::vector<cv::Vec4i> hierachy;

        /*
         *  contours
         */
        std::unordered_map<int, cv::RotatedRect> map_armor_light;
        std::vector<cv::Point2i> armors;

        cv::findContours(morphology_open, contour, hierachy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
        for (int j = 0; j + 1; j = hierachy[j][0])
            dfs(src, contour, hierachy, j, 0, map_armor_light, armors);

        process_armor(src, map_armor_light, armors);
        writer << src;
        cv::imshow("contour", src);

        cv::waitKey(10);
        capture.read(src);
    }
    writer.release();
    return 0;
}