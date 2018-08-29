#pragma once

#include <fstream>
#include <iostream>
#include <vector>

/**
 * This class handles writing data to files like a log system.
 */
class FileWriter
{
public:
    static std::ofstream m_file;

    template <typename T>
    static void Write(const std::string &file_name, T data)
    {
        return;
        m_file.open("/home/albert/Documents/FILES/" + file_name, std::ios::app);
        if (m_file.is_open())
        {
            m_file << data << "\n";
        }
        else
        {
            std::cout << "Couldn't open File " <<  file_name << " with FileWriter!\n";
            exit(1);
        }
        m_file.close();
    }

    template <typename T>
    static void WriteArray(const std::string &file_name, std::vector<T> data)
    {
        return;
        m_file.open("/home/albert/Documents/FILES/" + file_name, std::ios::app);
        if (m_file.is_open())
        {
            for (auto& item : data)
                m_file << item->weight << "\n";
        }
        else
        {
            std::cout << "Couldn't open File " << file_name << " with FileWriter!\n";
            exit(1);
        }

        m_file.close();
    }
};
