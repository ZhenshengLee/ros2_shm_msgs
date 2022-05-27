// Copyright (c) 2022, ZhenshengLee.
// All rights reserved.

#ifndef SHM_MSGS__ARRAY_HELPER_HPP_
#define SHM_MSGS__ARRAY_HELPER_HPP_

#include "shm_msgs/msg/string.hpp"
#include "shm_msgs/msg/header.hpp"
#include "std_msgs/msg/header.hpp"

#include <string>
#include <memory>
#include <utility>
#include <cstring>
#include <memory>

// the conversion from the shm_str(std::array of char) to std::string

namespace shm_msgs
{
    // the operator "="
    static inline bool is_equal(shm_msgs::msg::String shm_str, const std::string& std_str)
    {
        int size = (uint8_t)std::max(std_str.size(), (size_t)shm_str.size);
        if(size > shm_msgs::msg::String::MAX_SIZE)
        {
            throw std::runtime_error("std_str is too big, please check!");
        }
        return !std::memcmp(shm_str.data.data(), std_str.data(), size);
    }
    static inline bool is_unequal(shm_msgs::msg::String shm_str, const std::string& std_str)
    {
        return !is_equal(shm_str, std_str);
    }

    // shm_str setter and getter
    static inline void set_str(shm_msgs::msg::String& shm_str, const std::string& std_str)
    {
        std::memcpy(shm_str.data.data(), std_str.data(), std_str.size());
        shm_str.size = std_str.size();
    }
    // https://stackoverflow.com/questions/32787502/put-bytes-from-unsigned-char-array-to-stdstring-using-memcpy-function
    static inline std::string get_str(shm_msgs::msg::String& shm_str)
    {
        std::string std_str;
        std_str.resize(shm_str.size);
        std::memcpy((char*)std_str.data(), shm_str.data.data(), shm_str.size);
        return std_str;
    }
    static inline std::string get_str(const shm_msgs::msg::String& shm_str)
    {
        std::string std_str;
        std_str.resize(shm_str.size);
        std::memcpy((char*)std_str.data(), shm_str.data.data(), shm_str.size);
        return std_str;
    }

    // shm_header getter and setter
    static inline std_msgs::msg::Header get_header(shm_msgs::msg::Header& shm_header)
    {
        std_msgs::msg::Header std_header;
        std_header.stamp = shm_header.stamp;
        std_header.frame_id = get_str(shm_header.frame_id);
        return std_header;
    }
    static inline std_msgs::msg::Header get_header(const shm_msgs::msg::Header& shm_header)
    {
        std_msgs::msg::Header std_header;
        std_header.stamp = shm_header.stamp;
        std_header.frame_id = get_str(shm_header.frame_id);
        return std_header;
    }
    static inline void set_header(shm_msgs::msg::Header& shm_header, std_msgs::msg::Header& std_header)
    {
        shm_header.stamp = std_header.stamp;
        set_str(shm_header.frame_id, std_header.frame_id);
    }
    static inline void set_header(shm_msgs::msg::Header& shm_header, const std_msgs::msg::Header& std_header)
    {
        shm_header.stamp = std_header.stamp;
        set_str(shm_header.frame_id, std_header.frame_id);
    }

    // add
}  // namespace shm_msgs

#endif  // SHM_MSGS__ARRAY_HELPER_HPP_
