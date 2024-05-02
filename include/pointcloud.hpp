#pragma once

#include <pcl/io/pcd_io.h>
// #include <pcl/io/ply_io.h>
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

class PCD
{
public:
    PCD(std::string filename)
    {
        operator=(filename);
    };

    void operator=(std::string filename)
    {
        this->filename = filename;
        if (!boost::filesystem::exists(filename))
        {
            std::cout << "File doesn't exist. Throw by pointcloud reader.\n";
            exit(EXIT_FAILURE);
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB> &get()
    {
        pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename, pointcloud);
        return pointcloud;
    }

private:
    std::string filename;
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
};