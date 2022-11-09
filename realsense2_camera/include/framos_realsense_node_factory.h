// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 Intel Corporation. All Rights Reserved

#pragma once

#include "../include/realsense_node_factory.h"

namespace realsense2_camera
{
    class FramosRealSenseNodeFactory : public nodelet::Nodelet
    {
    public:
        FramosRealSenseNodeFactory();
        virtual ~FramosRealSenseNodeFactory();

    private:
        std::string api_version_to_string(int version);
        void closeDevice();
        void StartDevice();
        void change_device_callback(rs2::event_information& info);
        void getDevice(rs2::device_list list);
        virtual void onInit() override;
        void initialize(const ros::WallTimerEvent &ignored);
        void tryGetLogSeverity(rs2_log_severity& severity) const;
        void reset();
        bool handleReset(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
        static std::string parse_usb_port(std::string line);
        bool toggle_sensor_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
        void setRs2CliArgs();
        void setDeviceFilterList();
        uint64_t serialFromString(const std::string& serial);
        uint32_t ipFromString(const std::string& ip);
        bool isNumber(const std::string& str);

        rs2::device _device;
        std::shared_ptr<InterfaceRealSenseNode> _realSenseNode;
        rs2::context* _ctx;
        std::string _serial_no;
        std::string _usb_port_id;
        std::string _device_type;
        bool _initial_reset;
        std::thread _query_thread;
        bool _is_alive;
        ros::ServiceServer toggle_sensor_srv;
        ros::WallTimer _init_timer;
        ros::ServiceServer _reset_srv;
        std::string _ip_address;

    };
}//end namespace
