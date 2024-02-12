#pragma once

#include <pcl/point_types.h>
#include <GLFW/glfw3.h>
#include <boost/filesystem.hpp>
#include <string>
#include <fstream>
#include <sstream>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <types.hpp>
#include <transform.hpp>

void convert_rgb(const uint32_t &rgb, pcl::PointXYZRGB &point)
{
    point.a = (rgb >> 24) & 0xFF;
    point.b = (rgb >> 16) & 0xFF;
    point.g = (rgb >> 8) & 0xFF;
    point.r = rgb & 0xFF;
}
class PCDReader
{
public:
    PCDReader(const std::string &filepath)
    {
        operator=(filepath);
    }
    ~PCDReader()
    {
        delete data.points;
    }

    void operator=(const std::string &filepath)
    {
        file_path = filepath;
        parse();
    }

    void operator+=(const char *filepath)
    {
        PCDReader new_reader(filepath);
        PCDFormat new_data = new_reader.get_data();

        uint64_t new_size = data.num_points + new_data.num_points;
        pcl::PointXYZRGB *temp_points = new pcl::PointXYZRGB[new_size];
        GLfloat *temp_gl_data = new GLfloat[new_size * 6];
        for (uint64_t i = 0; i < data.num_points; ++i)
        {
            temp_points[i] = data.points[i];
            temp_gl_data[i * 6] = data.gl_data[i * 6];
            temp_gl_data[i * 6 + 1] = data.gl_data[i * 6 + 1];
            temp_gl_data[i * 6 + 2] = data.gl_data[i * 6 + 2];
            temp_gl_data[i * 6 + 3] = data.gl_data[i * 6 + 3];
            temp_gl_data[i * 6 + 4] = data.gl_data[i * 6 + 4];
            temp_gl_data[i * 6 + 5] = data.gl_data[i * 6 + 5];
        }
        for (uint64_t i = 0; i < new_data.num_points; i++)
        {
            temp_points[data.num_points + i] = new_data.points[i];
            temp_gl_data[(data.num_points + i) * 6] = new_data.gl_data[i * 6];
            temp_gl_data[(data.num_points + i) * 6 + 1] = new_data.gl_data[i * 6 + 1];
            temp_gl_data[(data.num_points + i) * 6 + 2] = new_data.gl_data[i * 6 + 2];
            temp_gl_data[(data.num_points + i) * 6 + 3] = new_data.gl_data[i * 6 + 3];
            temp_gl_data[(data.num_points + i) * 6 + 4] = new_data.gl_data[i * 6 + 4];
            temp_gl_data[(data.num_points + i) * 6 + 5] = new_data.gl_data[i * 6 + 5];
        }

        delete data.points;
        delete data.gl_data;
        data.points = temp_points;
        data.gl_data = temp_gl_data;
        data.num_points += new_data.num_points;
        data.height = 1;
        data.width = data.num_points;
    }

    PCDFormat &get_data()
    {
        return data;
    }

private:
    bool parse()
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
        if (data.num_points == 0)
            return;
        if (data.gl_data != nullptr)
            delete[] data.gl_data;
        if (data.points != nullptr)
            delete[] data.points;
        data.gl_data = new GLfloat[data.num_points * 6];
        data.points = new pcl::PointXYZRGB[data.num_points];

        bool is_need_transform = false;
        Transform transf = data.view_point;
        if (std::fabs(data.view_point.x) > .0f || std::fabs(data.view_point.y) > .0f || std::fabs(data.view_point.z) > .0f ||
            std::fabs(data.view_point.qw) > .0f || std::fabs(data.view_point.qx) > .0f || std::fabs(data.view_point.qy) > .0f || std::fabs(data.view_point.qz) > .0f)
            is_need_transform = true;
        for (uint64_t i = 0; i < data.num_points; ++i)
        {
            std::string line;
            if (std::getline(file, line))
            {
                std::istringstream iss(line);
                uint32_t rgb;
                iss >> point.x >> point.y >> point.z >> rgb;
                convert_rgb(rgb, point);
                if (is_need_transform)
                    transf.transform(point, point);
                data.gl_data[i * 6] = point.x;
                data.gl_data[i * 6 + 1] = point.y;
                data.gl_data[i * 6 + 2] = point.z;
                data.gl_data[i * 6 + 3] = point.r / 255.f;
                data.gl_data[i * 6 + 4] = point.g / 255.f;
                data.gl_data[i * 6 + 5] = point.b / 255.f;
                data.points[i] = point;
            }
            else
            {
                std::cerr << "Error reading data line " << i + 1 << std::endl;
                break;
            }
        }
    }

private:
    std::string file_path;
    PCDFormat data;
};