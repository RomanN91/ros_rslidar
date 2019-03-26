/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the RILIDAR 3D LIDARs
 */
#include "rsdriver.h"
// #include <rslidar_msgs/rslidarScan.h>

namespace rslidar_driver
{
rslidarDriver::rslidarDriver(const LidarConfig& config/*ros::NodeHandle node, ros::NodeHandle private_nh*/)
{
  // use private node handle to get parameters
  /// private_nh.param("frame_id", config_.frame_id, std::string("rslidar"));
  config_.frame_id = config.frame_id;

  /*std::string tf_prefix = tf::getPrefixParam(private_nh);
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
  config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);*/

  // get model name, validate string, determine packet rate
  /// private_nh.param("model", config_.model, std::string("RS16"));
  config_.model = config.model;
  double packet_rate;  // packet frequency (Hz)
  std::string model_full_name;

  // product model
  if (config_.model == "RS16")
  {
    packet_rate = 840;
    model_full_name = "RS-LiDAR-16";
  }
  else if (config_.model == "RS32")
  {
    packet_rate = 1690;
    model_full_name = "RS-LiDAR-32";
  }
  else
  {
    // ROS_ERROR_STREAM("unknown LIDAR model: " << config_.model);
    std::cerr << "unknown LIDAR model: " << config_.model << std::endl;
    packet_rate = 2600.0;
  }
  std::string deviceName(std::string("Robosense ") + model_full_name);

  /// private_nh.param("rpm", config_.rpm, 600.0);
  config_.rpm = config.rpm;
  double frequency = (config_.rpm / 60.0);  // expected Hz rate

  // default number of packets for each scan is a single revolution
  // (fractions rounded up)

  int npackets = (int)ceil(packet_rate / frequency);
  // private_nh.param("npackets", config_.npackets, npackets);
  if (config.npackets == 0) {
      config_.npackets = npackets;
  } else {
      config_.npackets = config.npackets;
  }
  // ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");
  std::cout << "publishing " << config_.npackets << " packets per scan" << std::endl;

  std::string dump_file;
  // private_nh.param("pcap", dump_file, std::string(""));
  dump_file = config.pcap;

  int msop_udp_port;
  // private_nh.param("msop_port", msop_udp_port, (int)MSOP_DATA_PORT_NUMBER);
  msop_udp_port = config.msop_port;
  int difop_udp_port;
  // private_nh.param("difop_port", difop_udp_port, (int)DIFOP_DATA_PORT_NUMBER);
  difop_udp_port = config.difop_port;

  double cut_angle;
  // private_nh.param("cut_angle", cut_angle, -0.01);
  cut_angle = config.cut_angle;
  if (cut_angle < 0.0)
  {
    // ROS_INFO_STREAM("Cut at specific angle feature deactivated.");
    std::cout << "Cut at specific angle feature deactivated." << std::endl;
  }
  else if (cut_angle < 360)
  {
//    ROS_INFO_STREAM("Cut at specific angle feature activated. "
//                    "Cutting rslidar points always at "
//                    << cut_angle << " degree.");
    std::cout << "Cut at specific angle feature activated. "
                 "Cutting rslidar points always at "
                 << cut_angle << " degree." << std::endl;
  }
  else
  {
//    ROS_ERROR_STREAM("cut_angle parameter is out of range. Allowed range is "
//                     << "between 0.0 and 360 negative values to deactivate this feature.");
    std::cout << "cut_angle parameter is out of range. Allowed range is "
              << "between 0.0 and 360 negative values to deactivate this feature." << std::endl;
    cut_angle = -0.01;
  }

  // Convert cut_angle from radian to one-hundredth degree,
  // which is used in rslidar packets
  config_.cut_angle = static_cast<int>(cut_angle * 100);

  // Initialize dynamic reconfigure
  /// srv_ = boost::make_shared<dynamic_reconfigure::Server<rslidar_driver::rslidarNodeConfig> >(private_nh);
  //// dynamic_reconfigure::Server<rslidar_driver::rslidarNodeConfig>::CallbackType f;
  //// f = boost::bind(&rslidarDriver::callback, this, _1, _2);
  //// srv_->setCallback(f);  // Set callback function und call initially

  // initialize diagnostics
  //// diagnostics_.setHardwareID(deviceName);
  const double diag_freq = packet_rate / config_.npackets;
  diag_max_freq_ = diag_freq;
  diag_min_freq_ = diag_freq;
  // ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

  /// using namespace diagnostic_updater;
//  diag_topic_.reset(new TopicDiagnostic("rslidar_packets", diagnostics_,
//                                        FrequencyStatusParam(&diag_min_freq_, &diag_max_freq_, 0.1, 10),
//                                        TimeStampStatusParam()));

  // open rslidar input device or file
  if (dump_file != "")  // have PCAP file?
  {
    // read data from packet capture file
    /// msop_input_.reset(new rslidar_driver::InputPCAP(private_nh, msop_udp_port, packet_rate, dump_file));
    /// difop_input_.reset(new rslidar_driver::InputPCAP(private_nh, difop_udp_port, packet_rate, dump_file));
    msop_input_.reset(new rslidar_driver::InputPCAP(config, config.device_ip, msop_udp_port, packet_rate, dump_file));
    difop_input_.reset(new rslidar_driver::InputPCAP(config, config.device_ip, difop_udp_port, packet_rate, dump_file));
  }
  else
  {
    // read data from live socket
    /// msop_input_.reset(new rslidar_driver::InputSocket(private_nh, msop_udp_port));
    /// difop_input_.reset(new rslidar_driver::InputSocket(private_nh, difop_udp_port));

    msop_input_.reset(new rslidar_driver::InputSocket(config.device_ip, msop_udp_port));
    difop_input_.reset(new rslidar_driver::InputSocket(config.device_ip, difop_udp_port));
  }

  // raw packet output topic
  /// msop_output_ = node.advertise<rslidar_msgs::rslidarScan>("rslidar_packets", 10);
  /// difop_output_ = node.advertise<rslidar_msgs::rslidarPacket>("rslidar_packets_difop", 10);
  difop_thread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&rslidarDriver::difopPoll, this)));
}



/** poll the device
 *
 *  @returns true unless end of file reached
 */
boost::shared_ptr<Scan> rslidarDriver::poll(void)
{
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  /// rslidar_msgs::rslidarScanPtr scan(new rslidar_msgs::rslidarScan);
  boost::shared_ptr<Scan> scan(new Scan);

  // Since the rslidar delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.
  if (config_.cut_angle >= 0)  // Cut at specific angle feature enabled
  {
    // scan->packets.reserve(config_.npackets);
    // rslidar_msgs::rslidarPacket tmp_packet;

    scan->packets.reserve(config_.npackets);
    LidarPacket tmp_packet;
    while (true)
    {
      while (true)
      {
        int rc = msop_input_->getPacket(&tmp_packet, config_.time_offset);
        if (rc == 0)
          break;  // got a full packet?
        if (rc < 0)
          // return false;  // end of file reached?
            return nullptr;
      }
      scan->packets.push_back(tmp_packet);

      static int ANGLE_HEAD = -36001;  // note: cannot be set to -1, or stack smashing
      static int last_azimuth = ANGLE_HEAD;

      int azimuth = 256 * tmp_packet.data[44] + tmp_packet.data[45];
      // int azimuth = *( (u_int16_t*) (&tmp_packet.data[azimuth_data_pos]));

      // Handle overflow 35999->0
      if (azimuth < last_azimuth)
      {
        last_azimuth -= 36000;
      }
      // Check if currently passing cut angle
      if (last_azimuth != ANGLE_HEAD && last_azimuth < config_.cut_angle && azimuth >= config_.cut_angle)
      {
        last_azimuth = azimuth;
        break;  // Cut angle passed, one full revolution collected
      }
      last_azimuth = azimuth;
    }
  }
  else  // standard behaviour
  {
    scan->packets.resize(config_.npackets);
    for (int i = 0; i < config_.npackets; ++i)
    {
      while (true)
      {
        // keep reading until full packet received
        int rc = msop_input_->getPacket(&scan->packets[i], config_.time_offset);
        if (rc == 0)
          break;  // got a full packet?
        if (rc < 0)
          // return false;  // end of file reached?
            return nullptr;
      }
    }
  }

  // publish message using time of last packet read
  // ROS_DEBUG("Publishing a full rslidar scan.");
  std::cout << "Publishing a full rslidar scan." << std::endl;
  scan->header.stamp = scan->packets.back().stamp;
  scan->header.frame_id = config_.frame_id;
  // msop_output_.publish(scan);

  // notify diagnostics that a message has been published, updating its status
  // diag_topic_->tick(scan->header.stamp);
  // diagnostics_.update();

  return scan;
  // return true;
}

boost::shared_ptr<Scan> rslidarDriver::difopPoll(void)
{
  // reading and publishing scans as fast as possible.
  /// rslidar_msgs::rslidarPacketPtr difop_packet_ptr(new rslidar_msgs::rslidarPacket);
  boost::shared_ptr<Scan> scan(new Scan);
  // while (true)  // ros::ok()
  {
    // keep reading
    /// rslidar_msgs::rslidarPacket difop_packet_msg;
    LidarPacket difop_packet;
    // boost::shared_ptr<LidarPacket> disfop_packet(new LidarPacket);
    int rc = difop_input_->getPacket(&difop_packet, config_.time_offset);
    if (rc == 0)
    {
      std::cout << "Publishing a difop data." << std::endl;
      // ROS_DEBUG("Publishing a difop data.");
      ///*difop_packet_ptr = difop_packet_msg;
      /// difop_output_.publish(difop_packet_ptr);
      scan->packets.push_back(difop_packet);
      return scan;
    }
    if (rc < 0)
        return nullptr;
      // return;  // end of file reached?

    // ros::spinOnce();
  }
}

/*void rslidarDriver::callback(rslidar_driver::rslidarNodeConfig& config, uint32_t level)
{
  ROS_INFO("Reconfigure Request");
  config_.time_offset = config.time_offset;
}*/
}
