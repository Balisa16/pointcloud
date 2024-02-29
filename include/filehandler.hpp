#pragma once

#include <boost/filesystem.hpp>
#include <future>
#include <iostream>
#include <string>
#include <vector>
#include <types.hpp>
#include <reader.hpp>

class FileHandler
{
public:
    static FileHandler &get_instance()
    {
        static FileHandler instance;
        return instance;
    }

    static bool read(std::string foldername)
    {
        return get_instance().Iread(foldername);
    }

    static void sync()
    {
        get_instance().Isync();
    }

    static bool is_new_data()
    {
        return get_instance().new_data;
    }

    static void get_data(Pointcloud &data)
    {
        data = get_instance()._temp_buff;
        get_instance().new_data = false;
    }

    static void get_camera_frame(CameraFrame &cam_frame)
    {
        cam_frame.copy(get_instance().camera_frame);
    }

private:
    bool is_run_task = false, new_data = false;
    std::future<int> task_result;
    std::string folder_name = "";
    std::vector<std::string> file_list;
    Pointcloud _temp_buff;

    CameraFrame camera_frame;

    bool Iread(std::string foldername)
    {
        auto collect_file = [&]()
        {
            for (boost::filesystem::directory_iterator it(folder_name); it != boost::filesystem::directory_iterator(); ++it)
                if (boost::filesystem::is_regular_file(it->status()) && it->path().extension() == ".pcd")
                    file_list.push_back(it->path().filename().string());
            std::cout << "Found " << file_list.size() << " pcd files" << std::endl;
        };

        if (!boost::filesystem::exists(foldername))
        {
            std::string start_path = foldername;

            // Try parent folder
            for (size_t i = 0; i < 2; i++)
            {
                start_path = "../" + start_path;
                std::cout << "Check : " << start_path;
                if (boost::filesystem::exists(start_path))
                {
                    std::cout << " [Valid]\n";
                    folder_name = start_path;
                    collect_file();
                    return true;
                }
                std::cout << " [Invalid]\n";
            }

            // Try current child folder
            boost::filesystem::path current_path = boost::filesystem::current_path();
            for (const auto &entry : boost::filesystem::directory_iterator(current_path))
                if (boost::filesystem::is_directory(entry.path()))
                {
                    if (boost::filesystem::is_directory(std::string(entry.path().string() + "/" + foldername)))
                    {
                        folder_name = entry.path().string() + "/" + foldername;
                        std::cout << "Used : " << folder_name << std::endl;
                        collect_file();
                        return true;
                    }
                }

            std::cout << "File doesn't exist.\n";
            return false;
        }

        folder_name = foldername;
        collect_file();
        return true;
    }

    int load_file(std::string filename)
    {
        std::cout << "Load : " << filename << std::flush;
        PCDReader parser(folder_name + '/' + filename);

        new_data = true;
        PCDFormat _pcd_format = parser.get_data();
        _temp_buff = _pcd_format;

        camera_frame.add(
            {_pcd_format.view_point.x,
             _pcd_format.view_point.y,
             _pcd_format.view_point.z},
            {_pcd_format.view_point.qw,
             _pcd_format.view_point.qx,
             _pcd_format.view_point.qy,
             _pcd_format.view_point.qz});

        std::cout << " [OK]" << std::endl;
        return 0;
    }

    void Isync()
    {
        // Pass if data is old data is still exist
        if (new_data)
            return;

        if (folder_name == "" || std::empty(folder_name))
        {
            std::cout << "Please set the folder name first" << std::endl;
            return;
        }

        if (!is_run_task && file_list.size())
        {
            std::string file_name = file_list.back();

            // Remove the first element
            file_list.erase(file_list.end());
            task_result = std::async(std::launch::async, &FileHandler::load_file, this, file_name);
            is_run_task = true;
        }

        if (task_result.valid() && task_result.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
            is_run_task = false;
    }

    FileHandler(const FileHandler &) = delete;
    FileHandler(FileHandler &&) = delete;
    FileHandler &operator=(const FileHandler &) = delete;
    FileHandler &operator=(FileHandler &&) = delete;

    FileHandler() = default;
    ~FileHandler() = default;
};