// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// This file is originally ported from ROS1:
// https://github.com/ros/common_msgs/blob/89069bc/shm_msgs/include/shm_msgs/fill_image.h

#ifndef SHM_MSGS__ARRAY_HELPER_HPP_
#define SHM_MSGS__ARRAY_HELPER_HPP_

#include "shm_msgs/msg/string.hpp"
#include <string>
#include <memory>
#include <utility>
#include <cstring>
#include <memory>

namespace shm_msgs
{

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

    static inline void set_str(shm_msgs::msg::String& shm_str, const std::string& std_str)
    {
        std::memcpy(shm_str.data.data(), std_str.data(), std_str.size());
        shm_str.size = std_str.size();
    }

    static inline std::string get_str(shm_msgs::msg::String& shm_str)
    {
        std::string std_str;
        std::memcpy((char*)std_str.data(), shm_str.data.data(), shm_str.size);
    }

}  // namespace shm_msgs

#endif  // SHM_MSGS__ARRAY_HELPER_HPP_
