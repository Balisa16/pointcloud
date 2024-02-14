#pragma once

#include <GLFW/glfw3.h>
#include <pcl/point_types.h>

struct ViewPoint
{
    float x, y, z, qw, qx, qy, qz;
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

    PCDFormat() : points(new pcl::PointXYZRGB[0]), gl_data(new GLfloat[0]) {}

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
        points = new pcl::PointXYZRGB[0];
        width = 0;
        height = 0;
        num_points = 0;
        format = "";
    }
};

struct Buffer
{
public:
    GLfloat *data;
    Buffer() : data(new GLfloat[data_start + data_limit * 6]),
               _size(0)
    {
        for (int i = 0; i < data_start; i++)
            data[i] = camera_frame_lines[i];
    }
    void operator=(PCDFormat &new_data)
    {
        clear();
        uint64_t __cnt = data_start_6;
        for (uint64_t i = 0; i < new_data.num_points; ++i)
        {
            buff_check(_size, __cnt);
            data[__cnt * 6] = new_data.gl_data[i * 6];
            data[__cnt * 6 + 1] = new_data.gl_data[i * 6 + 1];
            data[__cnt * 6 + 2] = new_data.gl_data[i * 6 + 2];
            data[__cnt * 6 + 3] = new_data.gl_data[i * 6 + 3];
            data[__cnt * 6 + 4] = new_data.gl_data[i * 6 + 4];
            data[__cnt * 6 + 5] = new_data.gl_data[i * 6 + 5];
            __cnt++;
        }
        if (__cnt - data_start_6 < data_limit)
            _size = __cnt - data_start_6 - 1;
    }

    void operator=(Buffer &new_data)
    {
        clear();
        uint64_t __cnt = data_start_6;
        for (uint64_t i = 0; i < new_data.size(); ++i)
        {
            buff_check(_size, __cnt);

            data[__cnt * 6] = new_data.data[i * 6];
            data[__cnt * 6 + 1] = new_data.data[i * 6 + 1];
            data[__cnt * 6 + 2] = new_data.data[i * 6 + 2];
            data[__cnt * 6 + 3] = new_data.data[i * 6 + 3];
            data[__cnt * 6 + 4] = new_data.data[i * 6 + 4];
            data[__cnt * 6 + 5] = new_data.data[i * 6 + 5];
            __cnt++;
        }

        if (__cnt - data_start_6 < data_limit)
            _size = __cnt - data_start_6 - 1;
    }

    void operator+=(PCDFormat &new_data)
    {
        uint64_t __cnt = _size + data_start_6;
        for (uint64_t i = 0; i < new_data.num_points; ++i)
        {

            buff_check(_size, __cnt);
            data[(__cnt + i) * 6] = new_data.gl_data[i * 6];
            data[(__cnt + i) * 6 + 1] = new_data.gl_data[i * 6 + 1];
            data[(__cnt + i) * 6 + 2] = new_data.gl_data[i * 6 + 2];
            data[(__cnt + i) * 6 + 3] = new_data.gl_data[i * 6 + 3];
            data[(__cnt + i) * 6 + 4] = new_data.gl_data[i * 6 + 4];
            data[(__cnt + i) * 6 + 5] = new_data.gl_data[i * 6 + 5];
            __cnt++;
        }
        if (__cnt - data_start_6 < data_limit)
            _size = __cnt - data_start_6 - 1;
    }

    void operator+=(Buffer &new_data)
    {
        uint64_t __cnt = _size + data_start_6;
        for (uint64_t i = 0; i < new_data.size(); ++i)
        {

            buff_check(_size, __cnt);
            data[(__cnt + i) * 6] = new_data.data[i * 6];
            data[(__cnt + i) * 6 + 1] = new_data.data[i * 6 + 1];
            data[(__cnt + i) * 6 + 2] = new_data.data[i * 6 + 2];
            data[(__cnt + i) * 6 + 3] = new_data.data[i * 6 + 3];
            data[(__cnt + i) * 6 + 4] = new_data.data[i * 6 + 4];
            data[(__cnt + i) * 6 + 5] = new_data.data[i * 6 + 5];
            __cnt++;
        }
        std::cout << "Added new data\n";
        if (__cnt - data_start_6 < data_limit)
            _size = __cnt - data_start_6 - 1;
    }

    Buffer operator+(const Buffer &buffer)
    {
        Buffer result;
        result = *this;

        // Copy data from the second buffer
        uint64_t __cnt = result._size + data_start_6;
        for (uint64_t i = 0; i < buffer._size; ++i)
        {
            buff_check(_size, __cnt);
            result.data[(__cnt + i) * 6] = buffer.data[i * 6];
            result.data[(__cnt + i) * 6 + 1] = buffer.data[i * 6 + 1];
            result.data[(__cnt + i) * 6 + 2] = buffer.data[i * 6 + 2];
            result.data[(__cnt + i) * 6 + 3] = buffer.data[i * 6 + 3];
            result.data[(__cnt + i) * 6 + 4] = buffer.data[i * 6 + 4];
            result.data[(__cnt + i) * 6 + 5] = buffer.data[i * 6 + 5];
            __cnt++;
        }

        if (__cnt - data_start_6 < data_limit)
            result._size = __cnt - data_start_6 - 1;

        return result;
    }

    void clear()
    {
        if (data != nullptr)
            delete[] data;

        data = new GLfloat[data_start + data_limit * 6];

        // Fill camera frame lines
        for (int i = 0; i < data_start; i++)
            data[i] = camera_frame_lines[i];
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

    uint32_t start() const
    {
        return data_start;
    }

private:
    uint64_t data_limit = 10000000;
    uint64_t _size;

    const uint32_t data_start = 96;
    const uint32_t data_start_6 = 96 / 6;

    void buff_check(uint64_t &size, uint64_t &counter)
    {
        if (counter + data_start_6 >= data_limit)
        {
            size = data_limit;
            std::cout << "Memory overflow\n";
            counter = 0;
        }
    }

    const GLfloat camera_frame_lines[96] = {
        0.f, 0.f, .0f, 1.f, .0f, .0f, .15f, .15f, .20f, 1.f, .0f, .0f,
        0.f, 0.f, .0f, 1.f, .0f, .0f, .15f, -.15f, .20f, 1.f, .0f, .0f,
        0.f, 0.f, .0f, 1.f, .0f, .0f, -.15f, .15f, .20, 1.f, .0f, .0f,
        0.f, 0.f, .0f, 1.f, .0f, .0f, -.15f, -.15f, .20f, 1.f, .0f, .0f,
        -.15f, -.15f, .20f, 1.f, .0f, .0f, .15f, -.15f, .20f, 1.f, .0f, .0f,
        .15f, -.15f, .20f, 1.f, .0f, .0f, .15f, .15f, .20f, 1.f, .0f, .0f,
        .15f, .15f, .20f, 1.f, .0f, .0f, -.15f, .15f, .20, 1.f, .0f, .0f,
        -.15f, .15f, .20, 1.f, .0f, .0f, -.15f, -.15f, .20f, 1.f, .0f, .0f};
};
