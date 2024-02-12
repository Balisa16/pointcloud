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