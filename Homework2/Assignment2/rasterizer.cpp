// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    auto p = Vector3f(x, y, 0);
    auto a = Vector3f(_v[0].x(), _v[0].y(), 0);
    auto b = Vector3f(_v[1].x(), _v[1].y(), 0);
    auto c = Vector3f(_v[2].x(), _v[2].y(), 0);

    auto z1 = (p - a).cross(b - a).z();
    auto z2 = (p - b).cross(c - b).z();
    auto z3 = (p - c).cross(a - c).z();

    return (z1 >= 0 && z2 >= 0 && z3 >= 0) || (z1 <= 0 && z2 <= 0 && z3 <= 0);
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

static float computeZ(float x, float y, const Triangle& t, const std::array<Vector4f, 3>& v)
{
    auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    z_interpolated *= w_reciprocal;
    return z_interpolated;
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            // t.setVertex(i, v[i].head<3>());
            // t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    float minX = std::min({v[0].x(), v[1].x(), v[2].x()});
    float maxX = std::max({v[0].x(), v[1].x(), v[2].x()});
    
    float minY = std::min({v[0].y(), v[1].y(), v[2].y()});
    float maxY = std::max({v[0].y(), v[1].y(), v[2].y()});

    int aabb_minX = std::max((int)std::floor(minX), 0);
    int aabb_maxX = std::min((int)std::ceil(maxX), width);

    int aabb_minY = std::max((int)std::floor(minY), 0);
    int aabb_maxY = std::min((int)std::ceil(maxY), height);

    // iterate through the pixel and find if the current pixel is inside the triangle
    for(int i = aabb_minX; i < aabb_maxX; i++)
    {
        for(int j = aabb_minY; j < aabb_maxY; j++)
        {
            if(use_super_sampling)
            {
                super_sampling(i, j, t, v);
            }
            else
            {
                normal_sampling(i, j, t, v);
            }
        }
    }
}

void rst::rasterizer::normal_sampling(int i, int j, const Triangle& t, const std::array<Vector4f, 3>& v)
{
    float x = i + 0.5f;
    float y = j + 0.5f;
    bool isInside = insideTriangle(x, y, t.v);
    if(isInside)
    {
        auto z_interpolated = computeZ(x, y, t, v);
        auto z = std::abs(z_interpolated);

        auto index = get_index(i, j);
        if(depth_buf[index] > z)
        {
            depth_buf[index] = z;
            set_pixel(Vector3f(i, j, 1), t.getColor());
        }
    }
}

void rst::rasterizer::super_sampling(int i, int j, const Triangle& t, const std::array<Vector4f, 3>& v)
{
    int index = get_index(i, j);
    auto& super_sampling_depth = super_sampling_depth_buf[index];
    int count = 0;

    for(int ki = 0; ki < 2; ki++)
    {
        for(int kj = 0; kj < 2; kj++)
        {
            float x = i + 0.5f * ki + 0.25f;
            float y = j + 0.5f * kj + 0.25f;
            float k = 2 * ki + kj;

            bool isInside = insideTriangle(x, y, t.v);
            if(isInside)
            {
                float z_interpolated = computeZ(x, y, t, v);
                float z = std::abs(z_interpolated);

                if(super_sampling_depth[k] > z)
                {
                    super_sampling_depth[k] = z;
                    count ++;
                }
            }
        }
    }

    if(count > 0)
    {
        auto rate = count * 0.25f;
        auto cur_pixel = frame_buf[index];
        set_pixel(Vector3f(i, j, 1), t.getColor() * rate + cur_pixel);
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        float i = std::numeric_limits<float>::infinity();
        std::array<float, 4> v = {i, i, i, i};
        std::fill(depth_buf.begin(), depth_buf.end(), i);
        std::fill(super_sampling_depth_buf.begin(), super_sampling_depth_buf.end(), v);
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    super_sampling_depth_buf.resize(w * h);
    use_super_sampling = false;
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on