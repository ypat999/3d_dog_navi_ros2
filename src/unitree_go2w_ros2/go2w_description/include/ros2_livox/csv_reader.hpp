//
// Created by lfc on 2021/3/1.
//

#ifndef SRC_GAZEBO_CSV_READER_HPP
#define SRC_GAZEBO_CSV_READER_HPP

#include <fstream>
#include <string>
#include <vector>
#include <iostream>
#include <algorithm>

class CsvReader
{
public:
    static bool ReadCsvFile(const std::string &file_name, std::vector<std::vector<double>> &datas)
    {
        // 使用二进制模式打开文件以提高读取效率
        std::ifstream file_stream(file_name, std::ios::in | std::ios::binary);
        if (!file_stream.is_open())
        {
            std::cerr << "cannot read csv file: " << file_name << "\n";
            return false;
        }

        // 跳过文件头
        std::string line;
        if (!std::getline(file_stream, line))
        {
            std::cerr << "empty csv file: " << file_name << "\n";
            return false;
        }

        // 预分配内存空间，假设平均每行有2个数据，初始预分配1000行
        datas.reserve(1600000);
        
        // 读取文件内容到缓冲区，减少磁盘IO
        std::string file_content;
        file_content.reserve(4096 * 4096); // 预分配1MB缓冲区
        
        // 读取剩余文件内容
        while (std::getline(file_stream, line))
        {
            // 跳过空行
            line.erase(std::remove_if(line.begin(), line.end(), ::isspace), line.end());
            if (line.empty() || line[0] == '#') // 跳过空行和注释行
                continue;

            // 创建临时数据行，预分配2个元素空间（角度和时间）
            std::vector<double> data;
            data.reserve(2);

            // 手动解析CSV行，避免std::stringstream的额外开销
            size_t start = 0;
            size_t end = line.find(',');
            while (end != std::string::npos)
            {
                try
                {
                    data.push_back(std::stod(line.substr(start, end - start)));
                }
                catch (const std::exception &e)
                {
                    std::cerr << "cannot convert value: " << line.substr(start, end - start) << " in line: " << line << "\n";
                    break;
                }
                start = end + 1;
                end = line.find(',', start);
            }
            
            // 解析最后一个值
            try
            {
                data.push_back(std::stod(line.substr(start)));
            }
            catch (const std::exception &e)
            {
                std::cerr << "cannot convert value: " << line.substr(start) << " in line: " << line << "\n";
                continue;
            }

            // 确保每行有3个数据（时间、方位角和天顶角）
            if (data.size() == 3)
            {
                datas.push_back(std::move(data)); // 使用移动语义，避免拷贝
            }
            else
            {
                std::cerr << "invalid line format: " << line << "\n";
            }
        }

        std::cerr << "data size:" << datas.size() << "\n";
        return !datas.empty();
    }
};

#endif // SRC_GAZEBO_CSV_READER_HPP
