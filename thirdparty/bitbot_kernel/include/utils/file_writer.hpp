#pragma once

#include "readerwriterqueue/readerwriterqueue.h"
#include "bitbot_kernel/utils/priority.h"

#include <fstream>
#include <iostream>
#include <thread>
#include <vector>

class FileWriter
{
public:
  FileWriter(std::string filename)
    : file_(filename, std::ios::out)
    , queue_(1024)
  {
    if(file_.bad())
    {
      std::cerr << "failed to open " << filename << std::endl;
    }
    else
    {
      write_thread_ = std::thread(&FileWriter::WriteFile, this);
    }
  }

  ~FileWriter()
  {
    run_ = false;
    write_thread_.join();
  }

  void Write(std::string str)
  {
    queue_.enqueue(std::vector<char>(str.begin(), str.end()));
  }

  void Write(std::vector<char> data)
  {
    queue_.enqueue(data);
  }

  void Write(const char* data, size_t size)
  {
    queue_.enqueue(std::vector<char>(data, data+size));
  }

private:

  void WriteFile()
  {
    setProcessHighPriority(18);
    std::vector<char> temp;

    while (run_)
    {
      while(queue_.try_dequeue(temp))
      {
        file_.write(temp.data(), temp.size());
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    while(queue_.try_dequeue(temp))
    {
      file_.write(temp.data(), temp.size());
    }
  }

  std::ofstream file_;
  moodycamel::ReaderWriterQueue<std::vector<char>> queue_;
  std::thread write_thread_;
  bool run_ = true;
};

