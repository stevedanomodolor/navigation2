
#include <string>
#include <iostream>
#include <fstream>
class plannerDebugUtils

{
    public:
    plannerDebugUtils()
    {

    }
    ~plannerDebugUtils()
    {

    }
    void set_file_name(int test_num)
    {          
            file_name_ = "/nav2_work/results/result_" + std::to_string(test_num) + ".txt";
            std::cout << "Setting file name to: " << file_name_ << std::endl;

    }
    bool open_stream()
    {
        stream_open_ = false;
        std::cout << "Opening file: here " << file_name_ << std::endl;
        file_stream_.open(file_name_);
        std::cout << "Filded stream opened " << file_name_ << std::endl;

        if(!file_stream_.is_open())
        {
            std::cout << "Error opening file: " << file_name_ << std::endl;
            return false;
        }
        stream_open_ = true;
        return true;
    }

    void close_stream()
    {
        file_stream_.close();
    }

    void write_to_stream(const double &x, const double &y, const double &theta)
    {
        file_stream_ << x << " " << y << " " << theta << std::endl;
    }
    bool stream_is_open()
    {
        return stream_open_;
    }


    private:
    std::string file_name_;
    std::ofstream file_stream_;
    bool stream_open_;
};