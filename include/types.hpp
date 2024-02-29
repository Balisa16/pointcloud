#pragma once

#include <GLFW/glfw3.h>
#include <pcl/point_types.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

namespace Calculate
{
    inline glm::vec3 transform(const glm::vec3 point,
                               const glm::vec3 in_translation,
                               const glm::quat in_rotation)
    {
        glm::mat4 t_mat = glm::translate(glm::mat4(1.0f), in_translation);

        glm::mat4 r_mat = glm::mat4_cast(in_rotation);

        glm::mat4 trans_mat = t_mat * r_mat;

        glm ::vec4 trans_vec = trans_mat * glm::vec4(point, 1.0f);

        return {trans_vec.x, trans_vec.y, trans_vec.z};
    }
}

struct ViewPoint
{
    float x, y, z, qw, qx, qy, qz;
};

struct CameraFrame
{
private:
    const int data_unit = 108;
    int maks_size = 1000;
    uint64_t _size;
    glm::vec3 last_position = {.0f, .0f, .0f};
    const GLfloat start_frame[48] = {
        0.f, 0.f, .0f, .15f, .15f, .20f,
        0.f, 0.f, .0f, .15f, -.15f, .20f,
        0.f, 0.f, .0f, -.15f, .15f, .20,
        0.f, 0.f, .0f, -.15f, -.15f, .20f,
        -.15f, -.15f, .20f, .15f, -.15f, .20f,
        .15f, -.15f, .20f, .15f, .15f, .20f,
        .15f, .15f, .20f, -.15f, .15f, .20,
        -.15f, .15f, .20, -.15f, -.15f, .20f};

public:
    GLfloat *data;

    void copy(CameraFrame &other)
    {
        data = other.data;
        _size = other._size;
        maks_size = other.maks_size;
        last_position = other.last_position;
    }

    void add(const glm::vec3 &new_pos, const glm::quat &orientation)
    {
        if (_size >= maks_size)
        {
            std::cerr << "Array reached limit. Rejected new point\n";
            return;
        }

        // Draw line from previous position to new position
        data[_size * data_unit] = last_position.x;
        data[_size * data_unit + 1] = last_position.y;
        data[_size * data_unit + 2] = last_position.z;
        data[_size * data_unit + 3] = 1.f; // red line

        data[_size * data_unit + 6] = new_pos.x;
        data[_size * data_unit + 7] = new_pos.y;
        data[_size * data_unit + 8] = new_pos.z;
        data[_size * data_unit + 9] = 1.f;

        glm::vec3 transf_point;
        for (size_t i = 0; i < 16; i++)
        {
            transf_point = Calculate::transform(
                glm::vec3(
                    start_frame[i * 3],
                    start_frame[i * 3 + 1],
                    start_frame[i * 3 + 2]),
                new_pos,
                orientation);
            data[_size * data_unit + (i + 2) * 6] = transf_point.x;
            data[_size * data_unit + (i + 2) * 6 + 1] = transf_point.y;
            data[_size * data_unit + (i + 2) * 6 + 2] = transf_point.z;
            data[_size * data_unit + (i + 2) * 6 + 3] = 1.f;
        }
        _size++;
        last_position = new_pos;
    }

    int unit() const
    {
        return data_unit;
    }

    bool resize(int new_size)
    {
        if (_size * data_unit >= new_size * data_unit)
        {
            std::cerr << "Current data too large and new size too small\n";
            return false;
        }

        maks_size = new_size;
        GLfloat *_temp_data = new GLfloat[maks_size * data_unit];
        for (int i = 0; i < _size * data_unit; i++)
            _temp_data[i] = data[i];
        delete[] data;
        data = _temp_data;
        return true;
    }

    int max_size()
    {
        return maks_size;
    }

    int size() const { return _size; }
    CameraFrame() : data(new GLfloat[maks_size * data_unit]), _size(0) {}
    virtual ~CameraFrame() { delete[] data; };
};

struct PCDFormat
{
    std::string fields[4];
    int size[4];
    char type[4];
    int count[4];
    uint64_t width, height;
    ViewPoint view_point;
    uint64_t num_points;
    std::string format;
    pcl::PointXYZRGB *points;
    GLfloat *gl_data;

    PCDFormat() : points(new pcl::PointXYZRGB[1]), gl_data(new GLfloat[1]) {}

    friend std::ostream &operator<<(std::ostream &os, const PCDFormat &data)
    {
        os << "Fields    : " << data.fields[0] << ' ' << data.fields[1] << ' ' << data.fields[2] << ' ' << data.fields[3]
           << "\nSize      : " << data.size[0] << ' ' << data.size[1] << ' ' << data.size[2] << ' ' << data.size[3]
           << "\nType      : " << data.size[0] << ' ' << data.size[1] << ' ' << data.size[2] << ' ' << data.size[3]
           << "\nCount     : " << data.count[0] << ' ' << data.count[1] << ' ' << data.count[2] << ' ' << data.count[3]
           << "\nWidth     : " << data.width
           << "\nHeight    : " << data.height
           << "\nViewpoint : " << data.view_point.x << ' ' << data.view_point.y << ' ' << data.view_point.z << ' ' << data.view_point.qw << ' ' << data.view_point.qx << ' ' << data.view_point.qy << ' ' << data.view_point.qz
           << "\nPoints    : " << data.num_points
           << "\nFormat    : " << data.format << '\n';
        return os;
    }

    void clear()
    {
        delete points;
        points = new pcl::PointXYZRGB[1];
        width = 0;
        height = 0;
        num_points = 0;
        format = "";
    }
};

struct Pointcloud
{
public:
    GLfloat *data;
    Pointcloud()
    {
        data = new GLfloat[data_limit * 6];
        _size = 0;
    }

    virtual ~Pointcloud()
    {
        if (data != nullptr)
            delete data;
    }

    void operator=(PCDFormat &new_data)
    {
        clear();
        uint64_t __cnt = 0;

        _size = new_data.num_points;
        if (_size > data_limit)
            _size = data_limit;

        for (uint64_t i = 0; i < new_data.num_points; ++i)
        {
            if (__cnt >= data_limit)
                __cnt = 0;
            data[__cnt * 6] = new_data.gl_data[i * 6];
            data[__cnt * 6 + 1] = new_data.gl_data[i * 6 + 1];
            data[__cnt * 6 + 2] = new_data.gl_data[i * 6 + 2];
            data[__cnt * 6 + 3] = new_data.gl_data[i * 6 + 3];
            data[__cnt * 6 + 4] = new_data.gl_data[i * 6 + 4];
            data[__cnt * 6 + 5] = new_data.gl_data[i * 6 + 5];
            __cnt++;
        }
    }

    void operator=(Pointcloud &new_data)
    {
        clear();
        uint64_t __cnt = 0;

        _size = new_data.size();
        if (_size > data_limit)
            _size = data_limit;

        for (uint64_t i = 0; i < new_data.size(); ++i)
        {
            if (__cnt >= data_limit)
                __cnt = 0;

            data[__cnt * 6] = new_data.data[i * 6];
            data[__cnt * 6 + 1] = new_data.data[i * 6 + 1];
            data[__cnt * 6 + 2] = new_data.data[i * 6 + 2];
            data[__cnt * 6 + 3] = new_data.data[i * 6 + 3];
            data[__cnt * 6 + 4] = new_data.data[i * 6 + 4];
            data[__cnt * 6 + 5] = new_data.data[i * 6 + 5];
            __cnt++;
        }
    }

    void operator+=(PCDFormat &new_data)
    {
        uint64_t __cnt = _size;

        _size += new_data.num_points;
        if (_size > data_limit)
            _size = data_limit;

        for (uint64_t i = 0; i < new_data.num_points; ++i)
        {
            if (__cnt >= data_limit)
                __cnt = 0;

            data[__cnt * 6] = new_data.gl_data[i * 6];
            data[__cnt * 6 + 1] = new_data.gl_data[i * 6 + 1];
            data[__cnt * 6 + 2] = new_data.gl_data[i * 6 + 2];
            data[__cnt * 6 + 3] = new_data.gl_data[i * 6 + 3];
            data[__cnt * 6 + 4] = new_data.gl_data[i * 6 + 4];
            data[__cnt * 6 + 5] = new_data.gl_data[i * 6 + 5];
            __cnt++;
        }
    }

    void operator+=(Pointcloud &new_data)
    {
        uint64_t __cnt = _size;

        _size += new_data.size();
        if (_size > data_limit)
            _size = data_limit;

        for (uint64_t i = 0; i < new_data.size(); ++i)
        {
            if (__cnt >= data_limit)
                __cnt = 0;

            data[__cnt * 6] = new_data.data[i * 6];
            data[__cnt * 6 + 1] = new_data.data[i * 6 + 1];
            data[__cnt * 6 + 2] = new_data.data[i * 6 + 2];
            data[__cnt * 6 + 3] = new_data.data[i * 6 + 3];
            data[__cnt * 6 + 4] = new_data.data[i * 6 + 4];
            data[__cnt * 6 + 5] = new_data.data[i * 6 + 5];
            __cnt++;
        }
    }

    Pointcloud operator+(const Pointcloud &Pointcloud)
    {
        uint64_t __cnt = _size;

        _size += Pointcloud._size;
        if (_size > data_limit)
            _size = data_limit;

        for (uint64_t i = 0; i < Pointcloud._size; ++i)
        {
            if (__cnt >= data_limit)
                __cnt = 0;

            data[__cnt * 6] = Pointcloud.data[i * 6];
            data[__cnt * 6 + 1] = Pointcloud.data[i * 6 + 1];
            data[__cnt * 6 + 2] = Pointcloud.data[i * 6 + 2];
            data[__cnt * 6 + 3] = Pointcloud.data[i * 6 + 3];
            data[__cnt * 6 + 4] = Pointcloud.data[i * 6 + 4];
            data[__cnt * 6 + 5] = Pointcloud.data[i * 6 + 5];
            __cnt++;
        }

        return *this;
    }

    void clear()
    {
        delete data;

        data = new GLfloat[data_limit * 6];
        _size = 0;
    }

    uint64_t size() const
    {
        return _size;
    }

    void resize(uint64_t maks_data)
    {
        data_limit = maks_data;
        clear();
    }

    GLfloat *get_data()
    {
        return data;
    }

    uint64_t limit() const
    {
        return data_limit;
    }

private:
    uint64_t data_limit = 100000000;
    uint64_t _size;
};