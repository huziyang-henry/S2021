#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

float Angle2Radian(float angle)
{
    return angle * MY_PI / 180.;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix4f rotation;

    float a = Angle2Radian(angle);
    float cosA = cos(a);
    float sinA = sin(a);

    auto I = Eigen::Matrix3f::Identity();
    auto n = axis;
    auto nT = n.transpose();

    float x = n.x();
    float y = n.x();
    float z = n.x();

    Eigen::Matrix3f N;
    N <<  0, -z,  y,
          z,  0, -x,
         -y,  x,  0;

    Eigen::Matrix3f R = cosA * I + (1 - cosA) * n * nT + sinA * N;

    rotation << R(0, 0), R(0, 1), R(0, 2), 0, 
                R(1, 0), R(1, 1), R(1, 2), 0, 
                R(2, 0), R(2, 1), R(2, 2), 0, 
                0, 0, 0, 1;

    return rotation;
}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 
                 0, 1, 0, -eye_pos[1], 
                 0, 0, 1, -eye_pos[2], 
                 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    auto a = Angle2Radian(rotation_angle);
    auto sinA = sin(a);
    auto cosA = cos(a);

    Eigen::Matrix4f rotation;
    rotation << cosA, -sinA, 0, 0, 
                sinA, cosA, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
    
    model = rotation * model;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    float halfAngle = eye_fov / 2;
    float halfRadian = Angle2Radian(halfAngle);
    float tanA = tan(halfRadian);

    float t = zNear * tanA;
    float b = -t;
    float l = b * aspect_ratio;
    float r = -l;
    float n = -zNear;
    float f = -zFar;

    Eigen::Matrix4f M_scale;
    M_scale << 2 / (r - l), 0 ,0 ,0,
               0, 2 / (t - b), 0, 0,
               0, 0, 2 / (n - f), 0,
               0, 0, 0, 1;

    Eigen::Matrix4f M_translate;
    M_translate << 1, 0, 0, (r + l) / 2,
                   0, 1, 0, (t + b) / 2,
                   0, 0, 1, (n + f) / 2,
                   0, 0, 0, 1;

    Eigen::Matrix4f M_persp_ortho;
    M_persp_ortho << n, 0, 0, 0,
                     0, n, 0, 0,
                     0 ,0, n + f, -n * f,
                     0, 0, 1, 0;

    Eigen::Matrix4f M_ortho = M_scale * M_translate;
    projection =  M_ortho * M_persp_ortho;

    return projection;
}

int main(int argc, const char **argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3)
    {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4)
        {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imshow("image", image);
        key = cv::waitKey(10);
        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a')
        {
            angle += 10;
        }
        else if (key == 'd')
        {
            angle -= 10;
        }
    }

    return 0;
}
