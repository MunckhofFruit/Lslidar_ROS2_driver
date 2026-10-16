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

#include "lslidar_ch_driver/lslidar_services.h"

namespace lslidar_ch_driver {
    // 获取设备包
    int LslidarServices::getDifopPacket(lslidar_ch_driver::msg::LslidarPacket::UniquePtr &pkt) {
        if (pkt->data[0] == 0xA5 && pkt->data[1] == 0xFF && pkt->data[2] == 0x00 && pkt->data[3] == 0x5A) {
            {
                std::lock_guard<std::mutex> lock(difop_data_mutex);
                mempcpy(difop_data, pkt->data.data(), 1206);
                lslidar_ip = std::to_string(difop_data[10]) + "." + std::to_string(difop_data[11]) + "." +
                             std::to_string(difop_data[12]) + "." + std::to_string(difop_data[13]);
            }

            difop_valid.store(true);
            return 0;
        } else {
            LS_ERROR << "Can not get dev packet! Set failed !!!" << LS_END;
            difop_valid.store(false);
            return -1;
        }
    }

    void LslidarServices::setUcwpPacketHeader(unsigned char *ucwp_data) {
        ucwp_data[0] = 0xAA;
        ucwp_data[1] = 0x00;
        ucwp_data[2] = 0xFF;
        ucwp_data[3] = 0x11;
        ucwp_data[4] = 0x22;
        ucwp_data[5] = 0x22;
        ucwp_data[6] = 0xAA;
        ucwp_data[7] = 0xAA;
    }

    bool LslidarServices::setUcwpData(unsigned char *ucwp_data) {
        if (!difop_valid.load()) {
            LS_ERROR << "No valid Difop packet available. Set failed!" << LS_END;
            return false;
        }

        {
            std::lock_guard<std::mutex> lock(difop_data_mutex);
            mempcpy(ucwp_data, difop_data, 1206);
        }

        difop_valid.store(false);
        setUcwpPacketHeader(ucwp_data);

        return true;
    }

    bool LslidarServices::sendUcwpPacketTolidar(unsigned char *ucwp_data) const {
        std::string target_ip;
        {
            std::lock_guard<std::mutex> lock(difop_data_mutex);
            target_ip = lslidar_ip;
        }
        int socketid;
        sockaddr_in addrSrv{};
        socketid = socket(2, 2, 0);
        addrSrv.sin_addr.s_addr = inet_addr(target_ip.c_str());
        addrSrv.sin_family = AF_INET;
        addrSrv.sin_port = htons(2368);
        sendto(socketid, (const char *) ucwp_data, 1206, 0, (struct sockaddr *) &addrSrv, sizeof(addrSrv));
        close(socketid);

        return true;
    }

    bool LslidarServices::checkAndSetIp(const std::string& ip, unsigned char* ucwp_data, int index, bool is_lidar_ip) {
        // 检查IP
        unsigned short first_value, second_value, third_value, end_value;
        if (sscanf(ip.c_str(), "%hu.%hu.%hu.%hu", &first_value, &second_value, &third_value, &end_value) != 4) {
            return false;
        }

        std::string disable_ip;
        if (is_lidar_ip) {
            disable_ip = std::to_string(ucwp_data[14]) + "." + std::to_string(ucwp_data[15]) + "." +
                         std::to_string(ucwp_data[16]) + "." + std::to_string(ucwp_data[17]);
            if (first_value >= 224) {
                LS_ERROR << "Parameter error, please check the input LiDAR ip parameters." << LS_END;
                return false;
            }
        } else {
            disable_ip = std::to_string(ucwp_data[10]) + "." + std::to_string(ucwp_data[11]) + "." +
                         std::to_string(ucwp_data[12]) + "." + std::to_string(ucwp_data[13]);
            if (first_value >= 240) {
                LS_ERROR << "Parameter error, please check the input destination ip parameters." << LS_END;
                return false;
            }
        }

        if (first_value == 0 || first_value == 127 || first_value >= 255 || disable_ip == ip) {
            LS_ERROR << "Parameter error, please check the input ip parameters " << ip.c_str() << LS_END;
            return false;
        }

        // 设置IP
        ucwp_data[index] = first_value;
        ucwp_data[index + 1] = second_value;
        ucwp_data[index + 2] = third_value;
        ucwp_data[index + 3] = end_value;

        return true;
    }

    bool LslidarServices::checkAndSetPort(int port, unsigned char* ucwp_data, int index, bool is_data_port) {
        // 检查端口
        int disable_port = ucwp_data[index] * 256 + ucwp_data[index];

        if (port < 1025 || port > 65535 || port == disable_port) {
            LS_ERROR << "Parameter error, please check the input port parameters." << LS_END;
            return false;
        }

        // 设置端口
        if (is_data_port) {
            ucwp_data[24] = port / 256;
            ucwp_data[25] = port % 256;
        } else {
            ucwp_data[26] = port / 256;
            ucwp_data[27] = port % 256;
        }

        return true;
    }

    bool LslidarServices::setIpAndPort(std::shared_ptr<lslidar_ch_driver::srv::IpAndPort::Request> req,
                                       std::shared_ptr<lslidar_ch_driver::srv::IpAndPort::Response> res) {
        unsigned char ucwp_data[1206];
        if (!setUcwpData(ucwp_data)) {
            res->result = false;
            return false;
        }

        if (!checkAndSetIp(req->lidar_ip, ucwp_data, 10, true)) {
            res->result = false;
            return false;
        }

        if (!checkAndSetIp(req->destination_ip, ucwp_data, 14, false)) {
            res->result = false;
            return false;
        }

        if (!checkAndSetPort(req->data_port, ucwp_data, 26, true)) {
            res->result = false;
            return false;
        }

        if (!checkAndSetPort(req->dev_port, ucwp_data, 24, false)) {
            res->result = false;
            return false;
        }

        sendUcwpPacketTolidar(ucwp_data);

        LS_MSG << "----------- Successfully! -----------" << LS_END;
        LS_MSG << "LiDAR IP: " << req->lidar_ip << LS_END;
        LS_MSG << "destination IP: " << req->destination_ip << LS_END;
        LS_MSG << "data port: " << req->data_port << LS_END;
        LS_MSG << "dev port: " << req->dev_port << LS_END;
        LS_MSG << "Please restart the ROS driver." << LS_END;
        LS_MSG << "" << LS_END;
        res->result = true;

        return true;
    }

    bool LslidarServices::setMotorSpeed(std::shared_ptr<lslidar_ch_driver::srv::MotorSpeed::Request> req,
                                        std::shared_ptr<lslidar_ch_driver::srv::MotorSpeed::Response> res) {
        unsigned char ucwp_data[1206];
        if (!setUcwpData(ucwp_data)) {
            res->result = false;
            return false;
        }

        if (req->motor_speed == 5) {
            ucwp_data[8] = 0x01;
            ucwp_data[9] = 0x2c;
        } else if (req->motor_speed == 10) {
            ucwp_data[8] = 0x02;
            ucwp_data[9] = 0x58;
        } else if (req->motor_speed == 20) {
            ucwp_data[8] = 0x04;
            ucwp_data[9] = 0xB0;
        } else if (req->motor_speed == 30) {
            ucwp_data[8] = 0x07;
            ucwp_data[9] = 0x08;
        } else if (req->motor_speed == 40) {
            ucwp_data[8] = 0x09;
            ucwp_data[9] = 0x60;
        } else if (req->motor_speed == 50) {
            ucwp_data[8] = 0x0B;
            ucwp_data[9] = 0xB8;
        } else if (req->motor_speed == 60) {
            ucwp_data[8] = 0x0E;
            ucwp_data[9] = 0x10;
        } else {
            LS_ERROR << "Parameter error, please check the input parameters." << LS_END;
            res->result = false;
            return false;
        }

        sendUcwpPacketTolidar(ucwp_data);
        LS_MSG << "Successfully set the LiDAR speed to " << ((ucwp_data[8] << 8) | ucwp_data[9]) << LS_END;
        LS_MSG << "" << LS_END;
        res->result = true;

        return true;
    }

    void LslidarServices::setTimeModeBytes(unsigned char* ucwp_data, int time_mode) {
        ucwp_data[44] = static_cast<unsigned char>(time_mode);
    }

    std::string LslidarServices::getTimeModeString(int time_mode) {
        switch (time_mode) {
            case 0:
                return "GPS";
            case 1:
                return "PTP L2";
            case 2:
                return "NTP";
            case 3:
                return "PTP UDPv4";
            case 4:
                return "E2E L2";
            case 5:
                return "E2E UDPv4";
            default:
                return "Unknown";
        }
    }

    bool LslidarServices::setTimeMode(std::shared_ptr<lslidar_ch_driver::srv::TimeMode::Request> req,
                                      std::shared_ptr<lslidar_ch_driver::srv::TimeMode::Response> res) {
        if (req->time_mode > 5) {
            LS_ERROR << "Parameter error, please check the input parameters." << LS_END;
            res->result = false;
            return false;
        }

        unsigned char ucwp_data[1206];
        if (!setUcwpData(ucwp_data)) {
            res->result = false;
            return false;
        }

        setTimeModeBytes(ucwp_data, req->time_mode);

        if (req->time_mode == 2) {
            std::string ntp_ip = req->ntp_ip;
            std::regex ipv4(
                    "\\b(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\b");
            if (!regex_match(ntp_ip, ipv4)) {
                LS_ERROR << "Parameter error, please check the input parameters." << LS_END;
                res->result = false;
                return true;
            }
            unsigned short first_value, second_value, third_value, end_value;
            sscanf(ntp_ip.c_str(), "%hu.%hu.%hu.%hu", &first_value, &second_value, &third_value, &end_value);
            ucwp_data[28] = first_value;
            ucwp_data[29] = second_value;
            ucwp_data[30] = third_value;
            ucwp_data[31] = end_value;
        }

        sendUcwpPacketTolidar(ucwp_data);

        std::string time_mode_str = getTimeModeString(req->time_mode);
        LS_MSG << time_mode_str << " mode setting successful!" << LS_END;
        LS_MSG << "" << LS_END;
        res->result = true;

        return true;
    }

    /////////////////////////////// 机械式 ///////////////////////////////
    bool LslidarCxServices::setIpAndPort(std::shared_ptr<lslidar_ch_driver::srv::IpAndPort::Request> req,
                                         std::shared_ptr<lslidar_ch_driver::srv::IpAndPort::Response> res) {
        return LslidarServices::setIpAndPort(req, res);
    }

    bool LslidarCxServices::setMotorControl(std::shared_ptr<lslidar_ch_driver::srv::MotorControl::Request> req,
                                            std::shared_ptr<lslidar_ch_driver::srv::MotorControl::Response> res) {
        unsigned char ucwp_data[1206];
        if (!setUcwpData(ucwp_data)) {
            res->result = false;
            return false;
        }

        if (req->motor_control == 1) {
            ucwp_data[8] = 0x02;
            ucwp_data[9] = 0x58;
            ucwp_data[41] = 0x00;
        } else if (req->motor_control == 0) {
            ucwp_data[41] = 0x01;
        } else {
            LS_ERROR << "Parameter error, please check the input parameters." << LS_END;
            res->result = false;
            return false;
        }

        sendUcwpPacketTolidar(ucwp_data);
        std::string status = (req->motor_control == 1) ? "rotation" : "stopped";
        LS_MSG << "Set successfully! LiDAR " << status << "." << LS_END;
        LS_MSG << "" << LS_END;
        res->result = true;

        return true;
    }

    bool LslidarCxServices::setMotorSpeed(std::shared_ptr<lslidar_ch_driver::srv::MotorSpeed::Request> req,
                                          std::shared_ptr<lslidar_ch_driver::srv::MotorSpeed::Response> res) {
        return LslidarServices::setMotorSpeed(req, res);
    }

    int LslidarCxServices::GetCxFpgaVersion(unsigned char* ucwp_data) {
        int fpga_version = 0;
        if (ucwp_data[1196] == 0x03 || (ucwp_data[1196] == 0x02 && ucwp_data[1197] >> 4 == 8)) {
            fpga_version = 3;
        } else if ((ucwp_data[1202] >= 0x64) && (ucwp_data[1203] & 0xF0) == 5) {
            fpga_version = 5;
        } else if ((ucwp_data[1198] & 0x0F) == 4) {
            fpga_version = 4;
        }

        return fpga_version;
    }

    bool LslidarCxServices::setPowerControl(std::shared_ptr<lslidar_ch_driver::srv::PowerControl::Request> req,
                                            std::shared_ptr<lslidar_ch_driver::srv::PowerControl::Response> res) {
        unsigned char ucwp_data[1206];
        if (!setUcwpData(ucwp_data)) {
            res->result = false;
            return false;
        }

        int fpga_version = GetCxFpgaVersion(ucwp_data);
        if (fpga_version == 0) {
            LS_ERROR << "LiDAR FPGA error !!!" << LS_END;
            res->result = false;
            return false;
        }

        if (req->power_control == 1) {
            if (fpga_version == 5) {
                ucwp_data[48] = 0xBB;
            } else if (fpga_version == 4) {
                ucwp_data[50] = 0xBB;
            } else if (fpga_version == 3) {
                ucwp_data[8] = 0x02;
                ucwp_data[9] = 0x58;
                ucwp_data[45] = 0x00;
            }
        } else if (req->power_control == 0) {
            if (fpga_version == 5) {
                ucwp_data[48] = 0xAA;
            } else if (fpga_version == 4) {
                ucwp_data[50] = 0xAA;
            } else if (fpga_version == 3) {
                ucwp_data[45] = 0x01;
            }
        }

        sendUcwpPacketTolidar(ucwp_data);
        std::string status = (req->power_control == 1) ? "on" : "off";
        LS_MSG << "Set successfully! LiDAR power " << status << "." << LS_END;
        LS_MSG << "" << LS_END;
        res->result = true;

        return true;
    }

    bool LslidarCxServices::setRfdRemoval(std::shared_ptr<lslidar_ch_driver::srv::RfdRemoval::Request> req,
                                          std::shared_ptr<lslidar_ch_driver::srv::RfdRemoval::Response> res) {
        unsigned char ucwp_data[1206];
        if (!setUcwpData(ucwp_data)) {
            res->result = false;
            return false;
        }

        int fpga_version = GetCxFpgaVersion(ucwp_data);
        if (fpga_version <= 3) {
            LS_WARN << "The " << fpga_version << ".0 version of LiDAR does not have this function!" << LS_END;
            res->result = false;
            return false;
        }

        if (req->rfd_removal <= 3) {
            if (fpga_version == 5) {
                ucwp_data[49] = static_cast<unsigned char>(req->rfd_removal);
            } else if (fpga_version == 4) {
                ucwp_data[110] = static_cast<unsigned char>(req->rfd_removal);
            }
        } else {
            LS_ERROR << "Parameter error, please check the input parameters." << LS_END;
            res->result = false;
            return false;
        }

        sendUcwpPacketTolidar(ucwp_data);
        LS_MSG << "Set successfully, Remove rain, fog, and dust, level: " << static_cast<int>(req->rfd_removal) << LS_END;
        LS_MSG << "" << LS_END;
        res->result = true;

        return true;
    }

    bool LslidarCxServices::setTailRemoval(std::shared_ptr<lslidar_ch_driver::srv::TailRemoval::Request> req,
                                           std::shared_ptr<lslidar_ch_driver::srv::TailRemoval::Response> res) {
        unsigned char ucwp_data[1206];
        if (!setUcwpData(ucwp_data)) {
            res->result = false;
            return false;
        }

        int fpga_version = GetCxFpgaVersion(ucwp_data);
        if (fpga_version <= 3) {
            LS_WARN << "The " << fpga_version << ".0 version of LiDAR does not have this function!" << LS_END;
            res->result = false;
            return false;
        }

        if (req->tail_removal <= 10) {
            if (fpga_version == 5) {
                ucwp_data[51] = static_cast<unsigned char>(req->tail_removal);
            } else if (fpga_version == 4) {
                ucwp_data[111] = static_cast<unsigned char>(req->tail_removal);
            }
        } else {
            LS_ERROR << "Parameter error, please check the input parameters." << LS_END;
            res->result = false;
            return false;
        }

        sendUcwpPacketTolidar(ucwp_data);
        LS_MSG << "Set successfully, Remove trail point. level: " << static_cast<int>(req->tail_removal) << LS_END;
        LS_MSG << "" << LS_END;
        res->result = true;

        return true;
    }

    void LslidarCxServices::setTimeModeBytes(unsigned char* ucwp_data, int time_mode) {
        ucwp_data[45] = static_cast<unsigned char>(time_mode);
    }

    bool LslidarCxServices::setTimeMode(std::shared_ptr<lslidar_ch_driver::srv::TimeMode::Request> req,
                                        std::shared_ptr<lslidar_ch_driver::srv::TimeMode::Response> res) {
        return LslidarServices::setTimeMode(req, res);
    }

} // namespace lslidar_ch_driver
