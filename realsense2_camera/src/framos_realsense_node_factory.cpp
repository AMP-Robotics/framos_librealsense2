// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include "../include/framos_realsense_node_factory.h"
#include "../include/base_realsense_node.h"
#include "../include/t265_realsense_node.h"
#include <iostream>
#include <map>
#include <mutex>
#include <condition_variable>
#include <signal.h>
#include <thread>
#include <sys/time.h>
#include <regex>

using namespace realsense2_camera;

#define REALSENSE_ROS_EMBEDDED_VERSION_STR (VAR_ARG_STRING(VERSION: REALSENSE_ROS_MAJOR_VERSION.REALSENSE_ROS_MINOR_VERSION.REALSENSE_ROS_PATCH_VERSION))
constexpr auto realsense_ros_camera_version = REALSENSE_ROS_EMBEDDED_VERSION_STR;

PLUGINLIB_EXPORT_CLASS(realsense2_camera::FramosRealSenseNodeFactory, nodelet::Nodelet)

std::string FramosRealSenseNodeFactory::api_version_to_string(int version)
{
	std::ostringstream ss;
	if (version / 10000 == 0)
		ss << version;
	else
		ss << (version / 10000) << "." << (version % 10000) / 100 << "." << (version % 100);
	return ss.str();
}

FramosRealSenseNodeFactory::FramosRealSenseNodeFactory():
    _is_alive(true)
{
    _ctx = nullptr;

    rs2_error* e = nullptr;
	std::string running_librealsense_version(api_version_to_string(rs2_get_api_version(&e)));
    ROS_INFO("RealSense ROS v%s", REALSENSE_ROS_VERSION_STR);
    ROS_INFO("Built with LibRealSense v%s", RS2_API_VERSION_STR);
	ROS_INFO_STREAM("Running with LibRealSense v" << running_librealsense_version);
	if (RS2_API_VERSION_STR != running_librealsense_version)
	{
		ROS_WARN("***************************************************");
		ROS_WARN("** running with a different librealsense version **");
		ROS_WARN("** than the one the wrapper was compiled with!   **");
		ROS_WARN("***************************************************");
	}

    auto severity = rs2_log_severity::RS2_LOG_SEVERITY_WARN;
    tryGetLogSeverity(severity);
    if (rs2_log_severity::RS2_LOG_SEVERITY_DEBUG == severity)
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    rs2::log_to_console(severity);
}

FramosRealSenseNodeFactory::~FramosRealSenseNodeFactory()
{
    _is_alive = false;
    if (_query_thread.joinable())
    {
        _query_thread.join();
    }

    if (_ctx != nullptr) {
        delete _ctx;
        _ctx = nullptr;
    }
}

std::string FramosRealSenseNodeFactory::parse_usb_port(std::string line)
{
    std::string port_id;
    std::regex self_regex("(?:[^ ]+/usb[0-9]+[0-9./-]*/){0,1}([0-9.-]+)(:){0,1}[^ ]*", std::regex_constants::ECMAScript);
    std::smatch base_match;
    bool found = std::regex_match(line, base_match, self_regex);
    if (found)
    {
        port_id = base_match[1].str();
        if (base_match[2].str().size() == 0)    //This is libuvc string. Remove counter is exists.
        {
            std::regex end_regex = std::regex(".+(-[0-9]+$)", std::regex_constants::ECMAScript);
            bool found_end = std::regex_match(port_id, base_match, end_regex);
            if (found_end)
            {
                port_id = port_id.substr(0, port_id.size() - base_match[1].str().size());
            }
        }
    }
    return port_id;
}

void FramosRealSenseNodeFactory::setRs2CliArgs()
{
    if (_ctx != nullptr) {
        ROS_ERROR("rs2_d400e_set_cli_args must be called before rs2::context initialization!");
        return;
    }
    
    std::string args;
    ros::NodeHandle privateNh = getPrivateNodeHandle();
    privateNh.param("dev_filter", args, std::string(""));

    std::vector<char*> cstrs;
    rs2::d400e::cli_arg carg;

    carg.argc = 0;
    if (args.size() > 0) {
        carg.argc++;
        cstrs.push_back((char*)(&args[0]));
    }

    for (size_t i = 1; i < args.size(); i++) {
        if (args[i] == ' ') {
            if ((i + 1) < args.size()) {
                carg.argc++;
                cstrs.push_back((char*)(&args[i + 1]));
            }
        }
    }
    carg.argv = const_cast<const char**>(cstrs.data());

    if (args.size() > 0) {
        rs2::d400e::set_cli_args(carg);
    }
}

void FramosRealSenseNodeFactory::setDeviceFilterList()
{
    std::vector<rs2_d400e_filter> filter_list;
    rs2_d400e_filter filter;
    filter.serial_range.begin = 0;
    filter.serial_range.end = 0;
    filter.ip_range.begin = 0;
    filter.ip_range.end = 0;
            
    if (_serial_no.size() > 0) {
        uint64_t serial = serialFromString(_serial_no);
        filter.serial_range.begin = serial;
        filter.serial_range.end = serial;
        filter_list.push_back(filter);
    }
    else if (_ip_address.size() > 0) {
        uint32_t ip = ipFromString(_ip_address);
        filter.ip_range.begin = ip;
        filter.ip_range.end = ip;
        filter_list.push_back(filter);
    }
    
    if (filter_list.size() > 0) {
        rs2::d400e::set_device_filter_list(filter_list);
    }
}

uint64_t FramosRealSenseNodeFactory::serialFromString(const std::string& serial)
{
    const uint8_t octet_count = 6;
    uint64_t output = 0;
    uint32_t octet[octet_count] = { 0 };

    // ex. 6CD446000000 == 12
    if (serial.size() == 12) {

        for (uint8_t i = 0; i < octet_count; i++) {
            try {
                octet[i] = std::stoul((serial.substr((i * 2), 2)).c_str(), nullptr, 16);
            }
            catch (std::invalid_argument& e) {                
                ROS_ERROR_STREAM("FRAMOS - Device filtering [serial]: " << e.what());
                exit(1);
            }
            catch (std::out_of_range& e) {
                ROS_ERROR_STREAM("FRAMOS - Device filtering [serial]: " << e.what());
                exit(1);
            }
        }

        int8_t index = 0;
        for (int8_t i = 40; i >= 0; i -= 8) {
            output |= ((uint64_t)octet[index++]) << i;
        }
    }
    else {
        ROS_ERROR_STREAM("FRAMOS - Invalid serial number format in device filtering file [invalid serial number size].");
        exit(1);
    }

    return output;
}

uint32_t FramosRealSenseNodeFactory::ipFromString(const std::string& ip)
{
    uint32_t output = 0;

    size_t n = std::count(ip.begin(), ip.end(), '.');
    if (n == 3) {

        std::string ip_tmp = ip;
        ip_tmp.erase(std::remove(ip_tmp.begin(), ip_tmp.end(), '.'), ip_tmp.end());

        if (isNumber(ip_tmp)) {
            char ch;
            uint32_t a, b, c, d;
            std::stringstream s(ip);
            s >> a >> ch >> b >> ch >> c >> ch >> d;
            output = ((a & 0x000000FF) << 24) | ((b & 0x000000FF) << 16) | ((c & 0x000000FF) << 8) | ((d & 0x000000FF) << 0);
        }
        else {
            ROS_ERROR_STREAM("FRAMOS - Device filtering - invalid ip address format [not a number].");
            exit(1);
        }
    }
    else {
        ROS_ERROR_STREAM("FRAMOS - Device filtering - invalid ip address format.");
        exit(1);
    }

    return output;
}

bool FramosRealSenseNodeFactory::isNumber(const std::string& str)
{
    std::string::const_iterator it = str.begin();
    while (it != str.end() && std::isdigit(*it)) {
        ++it;
    }

    return (!str.empty() && it == str.end());
}

void FramosRealSenseNodeFactory::getDevice(rs2::device_list list)
{

    if (_ctx == nullptr) {
        ROS_FATAL("rs2::context nullptr!");
        return;
    }

    if (!_device)
    {
        if (0 == list.size())
        {
            ROS_WARN("No RealSense devices were found!");
        }
        else
        {
            bool found = false;
            ROS_INFO_STREAM(" ");
            for (size_t count = 0; count < list.size(); count++)
            {
                rs2::device dev;
                try
                {
                    dev = list[count];
                }
                catch(const std::exception& ex)
                {
                    ROS_WARN_STREAM("Device " << count+1 << "/" << list.size() << " failed with exception: " << ex.what());
                    continue;
                }
                std::string ip;
                bool found_ip = false;

                auto sn = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                found_ip = dev.supports(RS2_CAMERA_INFO_IP_ADDRESS);
                if (found_ip) ip = dev.get_info(RS2_CAMERA_INFO_IP_ADDRESS);
                ROS_INFO_STREAM("Device with serial number " << sn << " was found."<<std::endl);

                if (found_ip) ROS_INFO_STREAM("Device with ip address " << ip << " was found."<<std::endl);
                std::string pn = dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT);
                std::string name = dev.get_info(RS2_CAMERA_INFO_NAME);
                ROS_INFO_STREAM("Device with physical ID " << pn << " was found.");
                std::vector<std::string> results;
                ROS_INFO_STREAM("Device with name " << name << " was found.");
                std::string port_id = parse_usb_port(pn);
                if (port_id.empty() && !found_ip)
                {
                    std::stringstream msg;
                    msg << "Error extracting usb port from device with physical ID: " << pn << std::endl << "Please report on github issue at https://github.com/IntelRealSense/realsense-ros";
                    if (_usb_port_id.empty())
                    {
                        ROS_WARN_STREAM(msg.str());
                    }
                    else
                    {
                        ROS_ERROR_STREAM(msg.str());
                        ROS_ERROR_STREAM("Please use serial number instead of usb port.");
                    }
                }
                else if (!port_id.empty() && !found_ip)
                {
                    ROS_INFO_STREAM("Device with port number " << port_id << " was found.");					
                }
                bool found_device_type(true);
                if (!_device_type.empty())
                {
                    std::smatch match_results;
                    std::regex device_type_regex(_device_type.c_str(), std::regex::icase);
                    found_device_type = std::regex_search(name, match_results, device_type_regex);
                }

                if ((_ip_address.empty() || (ip == _ip_address && found_ip)) && (_serial_no.empty() || sn == _serial_no) && (_usb_port_id.empty() || port_id == _usb_port_id) && found_device_type)
                {
                    _device = dev;
                    _serial_no = sn;
                    _ip_address = ip;
                    found = true;
                    break;
                }
            }
            if (!found)
            {
                // T265 could be caught by another node.
                std::string msg ("The requested device with ");
                bool add_and(false);
                if (!_serial_no.empty())
                {
                    msg += "serial number " + _serial_no;
                    add_and = true;
                }
                if (!_ip_address.empty())
                {
                    if (add_and)
                    {
                        msg += " and ";
                    }
                    msg += " ip address " + _ip_address;
                    add_and = true;
                }
                if (!_usb_port_id.empty())
                {
                    if (add_and)
                    {
                        msg += " and ";
                    }
                    msg += "usb port id " + _usb_port_id;
                    add_and = true;
                }
                if (!_device_type.empty())
                {
                    if (add_and)
                    {
                        msg += " and ";
                    }
                    msg += "device name containing " + _device_type;
                }
                msg += " is NOT found. Will Try again.";
                ROS_ERROR_STREAM(msg);
            }
            else
			{
				if (_device.supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
				{
					std::string usb_type = _device.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR);
					ROS_INFO_STREAM("Device USB type: " << usb_type);
					if (usb_type.find("2.") != std::string::npos)
					{
						ROS_WARN_STREAM("Device " << _serial_no << " is connected using a " << usb_type << " port. Reduced performance is expected.");
					}
				}
			}
        }
    }

    bool remove_tm2_handle(_device && RS_T265_PID != std::stoi(_device.get_info(RS2_CAMERA_INFO_PRODUCT_ID), 0, 16));
    if (remove_tm2_handle)
    {
        _ctx->unload_tracking_module();
    }

    if (_device && _initial_reset)
    {
        _initial_reset = false;
        try
        {
            ROS_INFO("Resetting device...");
            _device.hardware_reset();
            _device = rs2::device();         
        }
        catch(const std::exception& ex)
        {
            ROS_WARN_STREAM("An exception has been thrown: " << ex.what());
        }
    }
}

void FramosRealSenseNodeFactory::change_device_callback(rs2::event_information& info)
{
    if (info.was_removed(_device))
    {
        ROS_ERROR("The device has been disconnected!");
        reset();
    }
}

bool FramosRealSenseNodeFactory::toggle_sensor_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  if (req.data)
    ROS_INFO_STREAM("toggling sensor : ON");
  else
    ROS_INFO_STREAM("toggling sensor : OFF");
  _realSenseNode->toggleSensors(req.data);
  res.success=true;
  return true;
}

void FramosRealSenseNodeFactory::onInit()
{
	auto nh = getNodeHandle();
	_init_timer = nh.createWallTimer(ros::WallDuration(0.01), &FramosRealSenseNodeFactory::initialize, this, true);
}

void FramosRealSenseNodeFactory::initialize(const ros::WallTimerEvent &ignored)
{
    try
    {
#ifdef BPDEBUG
        std::cout << "Attach to Process: " << getpid() << std::endl;
        std::cout << "Press <ENTER> key to continue." << std::endl;
        std::cin.get();
#endif

        setRs2CliArgs();
        setDeviceFilterList();

        if (_ctx == nullptr) {
            _ctx = new rs2::context();
        }
        _device = rs2::device();

        ros::NodeHandle nh = getNodeHandle();
        auto privateNh = getPrivateNodeHandle();
        privateNh.param("serial_no", _serial_no, std::string(""));
        privateNh.param("usb_port_id", _usb_port_id, std::string(""));
        privateNh.param("device_type", _device_type, std::string(""));
        privateNh.param("ip_address", _ip_address, std::string(""));

        if (!toggle_sensor_srv)
		{
			toggle_sensor_srv = nh.advertiseService("enable", &FramosRealSenseNodeFactory::toggle_sensor_callback, this);
		}
        std::string rosbag_filename("");
        privateNh.param("rosbag_filename", rosbag_filename, std::string(""));

        if (!rosbag_filename.empty())
        {
            {
                ROS_INFO_STREAM("publish topics from rosbag file: " << rosbag_filename.c_str());
                auto pipe = std::make_shared<rs2::pipeline>();
                rs2::config cfg;
                cfg.enable_device_from_file(rosbag_filename.c_str(), false);
                cfg.enable_all_streams();
                pipe->start(cfg); //File will be opened in read mode at this point
                _device = pipe->get_active_profile().get_device();
                _serial_no = _device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            }
            if (_device)
            {
                StartDevice();
            }
        }
        else
        {
            privateNh.param("initial_reset", _initial_reset, false);

            _is_alive = true;
            _query_thread = std::thread([=]()
                        {
							double reconnect_timeout;
							privateNh.param("reconnect_timeout", reconnect_timeout, 6.0);
							double wait_for_device_timeout;
							privateNh.param("wait_for_device_timeout", wait_for_device_timeout, -1.0);
							std::chrono::milliseconds timespan(static_cast<int>(reconnect_timeout*1e3));
							ros::Time first_try_time = ros::Time::now();
                            while (_is_alive && !_device)
                            {
                                getDevice(_ctx->query_devices());
                                if (_device)
                                {
                                    std::function<void(rs2::event_information&)> change_device_callback_function = [this](rs2::event_information& info){change_device_callback(info);};
                                    _ctx->set_devices_changed_callback(change_device_callback_function);
                                    StartDevice();
                                }
                                else
                                {
									std::chrono::milliseconds actual_timespan(timespan);
									if (wait_for_device_timeout > 0)
									{
										auto time_to_timeout(wait_for_device_timeout - (ros::Time::now() - first_try_time).toSec());
										if (time_to_timeout < 0)
										{
											ROS_ERROR_STREAM("wait for device timeout of " << wait_for_device_timeout << " secs expired");
											exit(1);
										}
										else
										{
											double max_timespan_secs(std::chrono::duration_cast<std::chrono::seconds>(timespan).count());
											actual_timespan = std::chrono::milliseconds (static_cast<int>(std::min(max_timespan_secs, time_to_timeout) * 1e3));
										}
									}
									std::this_thread::sleep_for(actual_timespan);
                                }
                            }
                        });
                        if (!_reset_srv)
			{
				_reset_srv = privateNh.advertiseService("reset", &FramosRealSenseNodeFactory::handleReset, this);
			}
        }
    }
    catch(const std::exception& ex)
    {
        ROS_ERROR_STREAM("An exception has been thrown: " << ex.what());
        exit(1);
    }
    catch(...)
    {
        ROS_ERROR_STREAM("Unknown exception has occured!");
        exit(1);
    }
}

void FramosRealSenseNodeFactory::StartDevice()
{
	if (_realSenseNode) _realSenseNode.reset();
	try
	{
		ros::NodeHandle nh = getNodeHandle();
		ros::NodeHandle privateNh = getPrivateNodeHandle();
		// TODO
		std::string pid_str(_device.get_info(RS2_CAMERA_INFO_PRODUCT_ID));
		uint16_t pid = std::stoi(pid_str, 0, 16);
		switch(pid)
		{
		case SR300_PID:
		case SR300v2_PID:
		case RS400_PID:
		case RS405_PID:
		case RS410_PID:
		case RS460_PID:
		case RS415_PID:
		case RS420_PID:
		case RS420_MM_PID:
		case RS430_PID:
		case RS430_MM_PID:
		case RS430_MM_RGB_PID:
		case RS435_RGB_PID:
		case RS435i_RGB_PID:
		case RS455_PID:
		case RS465_PID:
		case RS_USB2_PID:
		case RS_L515_PID_PRE_PRQ:
		case RS_L515_PID:
		case RS_L535_PID:
			_realSenseNode = std::shared_ptr<BaseRealSenseNode>(new BaseRealSenseNode(nh, privateNh, _device, _serial_no));
			break;
		case RS_T265_PID:
			_realSenseNode = std::shared_ptr<T265RealsenseNode>(new T265RealsenseNode(nh, privateNh, _device, _serial_no));
			break;
		default:
			ROS_FATAL_STREAM("Unsupported device!" << " Product ID: 0x" << pid_str);
			ros::shutdown();
			exit(1);
		}
		assert(_realSenseNode);
		_realSenseNode->publishTopics();
	}
	catch (const rs2::error& e)
	{
		ROS_ERROR_STREAM("Exception: " << e.what());
	}
}

void FramosRealSenseNodeFactory::reset()
{
	_is_alive = false;
	if (_query_thread.joinable())
	{
		_query_thread.join();
	}

	try
	{
		_realSenseNode.reset();
		if (_device)
		{
			_device.hardware_reset();
			_device = rs2::device();
		}
	}
	catch (const rs2::error& e)
	{
		ROS_ERROR_STREAM("Exception: " << e.what());
	}

	_init_timer = getNodeHandle().createWallTimer(ros::WallDuration(1.0), &FramosRealSenseNodeFactory::initialize, this, true);
}

bool FramosRealSenseNodeFactory::handleReset(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	reset();
	return true;
}

void FramosRealSenseNodeFactory::tryGetLogSeverity(rs2_log_severity& severity) const
{
    static const char* severity_var_name = "LRS_LOG_LEVEL";
    auto content = getenv(severity_var_name);

    if (content)
    {
        std::string content_str(content);
        std::transform(content_str.begin(), content_str.end(), content_str.begin(), ::toupper);

        for (uint32_t i = 0; i < RS2_LOG_SEVERITY_COUNT; i++)
        {
            auto current = std::string(rs2_log_severity_to_string((rs2_log_severity)i));
            std::transform(current.begin(), current.end(), current.begin(), ::toupper);
            if (content_str == current)
            {
                severity = (rs2_log_severity)i;
                break;
            }
        }
    }
}
