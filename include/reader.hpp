#pragma once

#include <pcl/point_types.h>
#include <boost/filesystem.hpp>
#include <string>
#include <fstream>
#include <sstream>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <pcl/point_types.h>

void convert_rgb(const uint32_t &rgb, pcl::PointXYZRGB &point)
{
    point.a = (rgb >> 24) & 0xFF;
    point.b = (rgb >> 16) & 0xFF;
    point.g = (rgb >> 8) & 0xFF;
    point.r = rgb & 0xFF;
}

#ifndef PCDData
#define PCDData
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
#endif

class PCDParser
{
public:
    PCDParser(const std::string &filepath)
    {
        operator=(filepath);
    }
    ~PCDParser()
    {
        delete data.points;
    }

    void operator=(const std::string &filepath)
    {
        file_path = filepath;
        parse();
    }

    PCDFormat &get_data()
    {
        return data;
    }

private:
    PCDFormat data;
    bool
    parse()
    {
        std::ifstream file(file_path);
        if (!file.is_open())
        {
            std::cerr << "Error opening file: " << file_path << std::endl;
            return false;
        }

        std::string line;
        while (std::getline(file, line))
        {
            if (line.find("FIELDS") != std::string::npos)
            {
                parseFields(line);
            }
            else if (line.find("SIZE") != std::string::npos)
            {
                parseSize(line);
            }
            else if (line.find("TYPE") != std::string::npos)
            {
                parseType(line);
            }
            else if (line.find("COUNT") != std::string::npos)
            {
                parseCount(line);
            }
            else if (line.find("WIDTH") != std::string::npos)
            {
                parseWidth(line);
            }
            else if (line.find("HEIGHT") != std::string::npos)
            {
                parseHeight(line);
            }
            else if (line.find("VIEWPOINT") != std::string::npos)
            {
                parseViewPoint(line);
            }
            else if (line.find("POINTS") != std::string::npos)
            {
                parseNumPoints(line);
            }
            else if (line.find("DATA") != std::string::npos)
            {
                std::istringstream iss(line);
                std::string token;
                iss >> token;
                if (token == "DATA")
                    iss >> data.format;
                parse_cloud(file);
                break; // Assuming DATA is the last line we need
            }
        }

        if (file.is_open())
            file.close();
        return true;
    }

    void parseFields(const std::string &line)
    {
        std::istringstream iss(line);
        std::string token;
        iss >> token;
        if (token != "FIELDS")
            return;
        for (int i = 0; i < 4; ++i)
            iss >> data.fields[i];
    }

    void parseSize(const std::string &line)
    {
        std::istringstream iss(line);
        std::string token;
        iss >> token;
        for (int i = 0; i < 4; ++i)
            iss >> data.size[i];
    }

    void parseType(const std::string &line)
    {
        std::istringstream iss(line);
        std::string token;
        iss >> token;
        for (int i = 0; i < 4; ++i)
            iss >> data.type[i];
    }

    void parseCount(const std::string &line)
    {
        std::istringstream iss(line);
        std::string token;
        iss >> token;
        for (int i = 0; i < 4; ++i)
            iss >> data.count[i];
    }

    void parseWidth(const std::string &line)
    {
        std::istringstream iss(line);
        std::string token;
        iss >> token >> data.width >> token;
    }

    void parseHeight(const std::string &line)
    {
        std::istringstream iss(line);
        std::string token;
        iss >> token >> data.height >> token;
    }

    void parseViewPoint(const std::string &line)
    {
        std::istringstream iss(line);
        std::string token;
        iss >> token >> data.view_point.x >> data.view_point.y >> data.view_point.z >> data.view_point.qw >> data.view_point.qx >> data.view_point.qy >> data.view_point.qz;
    }

    void parseNumPoints(const std::string &line)
    {
        std::istringstream iss(line);
        std::string token;
        iss >> token >> data.num_points >> token;
    }

    void parse_cloud(std::ifstream &file)
    {
        data.points = new pcl::PointXYZRGB[data.num_points];

        pcl::PointXYZRGB point;
        for (uint64_t i = 0; i < data.num_points; ++i)
        {
            std::string line;
            if (std::getline(file, line))
            {
                std::istringstream iss(line);
                uint32_t rgb;
                iss >> point.x >> point.y >> point.z >> rgb;
                convert_rgb(rgb, point);
                data.points[i] = point;
            }
            else
            {
                std::cerr << "Error reading data line " << i + 1 << std::endl;
                break;
            }
        }
    }

    std::string file_path;
};