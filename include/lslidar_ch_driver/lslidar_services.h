/******************************************************************************
 * This file is part of lslidar_ch_driver.
 *
 * Copyright 2022 LeiShen Intelligent Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef LSLIDAR_CH_SERVICES_H
#define LSLIDAR_CH_SERVICES_H

#include <atomic>
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <regex>
#include <cstdio>
#include <cstring>
#include <string>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

#include "lslidar_log.h"
#include "lslidar_ch_driver/msg/lslidar_packet.hpp"
#include "lslidar_ch_driver/srv/ip_and_port.hpp"
#include "lslidar_ch_driver/srv/motor_control.hpp"
#include "lslidar_ch_driver/srv/motor_speed.hpp"
#include "lslidar_ch_driver/srv/power_control.hpp"
#include "lslidar_ch_driver/srv/rfd_removal.hpp"
#include "lslidar_ch_driver/srv/tail_removal.hpp"
#include "lslidar_ch_driver/srv/time_mode.hpp"

// DIFOP 设备包
// UCWP  配置包

namespace lslidar_ch_driver {

    class LslidarServices {
    public:
        LslidarServices() {}

        virtual ~LslidarServices() = default;

        // 获取设备包数据
        int getDifopPacket(lslidar_ch_driver::msg::LslidarPacket::UniquePtr &pkt);

        // 设置配置包包头
        static void setUcwpPacketHeader(unsigned char *ucwp_data);

        // 设置配置包
        bool setUcwpData(unsigned char *ucwp_data);

        // 发送配置包配置雷达
        bool sendUcwpPacketTolidar(unsigned char *ucwp_data) const;

        virtual void setTimeModeBytes(unsigned char* ucwp_data, int time_mode);

        std::string getTimeModeString(int time_mode);

        bool checkAndSetIp(const std::string& ip, unsigned char* ucwp_data, int index, bool is_lidar_ip);

        bool checkAndSetPort(int port, unsigned char* ucwp_data, int index, bool is_data_port);

        // 设置雷达IP 端口
        // 出错保护: 禁止将雷达ip与当前雷达目的ip设置相同，数据端口与当前设备端口设置相同，反之亦然
        virtual bool setIpAndPort(std::shared_ptr<lslidar_ch_driver::srv::IpAndPort::Request> req,
                                  std::shared_ptr<lslidar_ch_driver::srv::IpAndPort::Response> res);

        // 设置雷达转速
        virtual bool setMotorSpeed(std::shared_ptr<lslidar_ch_driver::srv::MotorSpeed::Request> req,
                                   std::shared_ptr<lslidar_ch_driver::srv::MotorSpeed::Response> res);

        // 设置雷达授时模式
        virtual bool setTimeMode(std::shared_ptr<lslidar_ch_driver::srv::TimeMode::Request> req,
                                 std::shared_ptr<lslidar_ch_driver::srv::TimeMode::Response> res);

    protected:
        mutable std::mutex difop_data_mutex;
        std::atomic<bool> difop_valid{false};
        unsigned char difop_data[1206];
        std::string lslidar_ip;
    };

    class LslidarCxServices : public LslidarServices {
    public:
        LslidarCxServices() {}

        // 机械式雷达设置 IP 端口
        bool setIpAndPort(std::shared_ptr<lslidar_ch_driver::srv::IpAndPort::Request> req,
                          std::shared_ptr<lslidar_ch_driver::srv::IpAndPort::Response> res) override;

        // 机械式雷达设置转速
        bool setMotorSpeed(std::shared_ptr<lslidar_ch_driver::srv::MotorSpeed::Request> req,
                           std::shared_ptr<lslidar_ch_driver::srv::MotorSpeed::Response> res) override;

        // 机械式雷达控制电机启停
        bool setMotorControl(std::shared_ptr<lslidar_ch_driver::srv::MotorControl::Request> req,
                             std::shared_ptr<lslidar_ch_driver::srv::MotorControl::Response> res);

        // 机械式雷达控制雷达上下电
        bool setPowerControl(std::shared_ptr<lslidar_ch_driver::srv::PowerControl::Request> req,
                             std::shared_ptr<lslidar_ch_driver::srv::PowerControl::Response> res);

        // 机械式雷达去雨雾尘
        bool setRfdRemoval(std::shared_ptr<lslidar_ch_driver::srv::RfdRemoval::Request> req,
                           std::shared_ptr<lslidar_ch_driver::srv::RfdRemoval::Response> res);

        // 机械式雷达设置去除拖尾
        bool setTailRemoval(std::shared_ptr<lslidar_ch_driver::srv::TailRemoval::Request> req,
                            std::shared_ptr<lslidar_ch_driver::srv::TailRemoval::Response> res);

        // 机械式雷达设置授时模式
        bool setTimeMode(std::shared_ptr<lslidar_ch_driver::srv::TimeMode::Request> req,
                         std::shared_ptr<lslidar_ch_driver::srv::TimeMode::Response> res) override;

        void setTimeModeBytes(unsigned char* ucwp_data, int time_mode) override;

        int GetCxFpgaVersion(unsigned char* ucwp_data);
    };

} // namespace lslidar_ch_driver

#endif // LSLIDAR_CH_SERVICES_H
