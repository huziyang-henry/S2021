#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    auto size = control_points.size();
    if(size == 1)
    {
        return control_points[0];
    }

    std::vector<cv::Point2f> ps;
    for(int i = 0; i < size - 1; i++)
    {
        auto p = control_points[i] + t * (control_points[i + 1] - control_points[i]);
        ps.emplace_back(p);
    }

    return recursive_bezier(ps, t);

}

double distance(cv::Point2f p1, cv::Point2f p2)
{
    return cv::norm(p2 - p1);
}

void anti_aliasing(const cv::Point2f &point, cv::Mat &window)
{    
    int x, y;
    const int N = 3;
    if((N % 2) == 0)
    {
        x = std::round(point.x);
        y = std::round(point.y);
    }
    else
    {
        x = point.x;
        y = point.y;
    }

    auto min_d = sqrt(2) / 2;
    auto max_d = N * min_d;

    auto min_color = 255;
    auto max_color = 0;

    int startX = x - N / 2;
    int startY = y - N / 2;
    for(int i = 0; i < N; i++)
    {
        for(int j = 0; j < N; j++)
        {
            auto p = cv::Point2f(startX + i + 0.5f, startY + j + 0.5f);
            auto d = distance(point, p);

            float t = 0.0;
            if(d < min_d)
            {
                t = 0;
            }
            else if(d > max_d)
            {
                t = 1;
            }
            else
            {
                t = (d - min_d) / (max_d - min_d);
            }
            
            auto color = window.at<cv::Vec3b>(p.y, p.x)[1];
            color = std::max(min_color + t * (max_color - min_color), (float)color);
            window.at<cv::Vec3b>(p.y, p.x)[1] = color;
        }
    }
}

void bezier_anti_aliasing(const std::vector<cv::Point2f> &control_points, cv::Mat &window)
{
    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = recursive_bezier(control_points, t);
        anti_aliasing(point, window);
    }
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
    }
}


int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            // naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);

            // anti aliasing
            bezier_anti_aliasing(control_points, window);
            cv::imwrite("my_bezier_curve_aa.png", window);

            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
