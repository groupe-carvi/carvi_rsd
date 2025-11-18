#pragma once

#include "cxx.h"

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/common/error_code.hpp>
#include <rs_driver/driver/driver_param.hpp>

// Forward declarations for types from rs_driver (global namespace)
struct PointXYZIRT;
template <typename T>
class PointCloudT;

namespace carvi_rsd
{

struct PackedPointXYZIRT
{
  float x;
  float y;
  float z;
  uint8_t intensity;
  uint16_t ring;
  double timestamp;
};

class PointCloudBuffer
{
public:
  PointCloudBuffer();
  ~PointCloudBuffer();

  // Delete copy and move operations
  PointCloudBuffer(const PointCloudBuffer&) = delete;
  PointCloudBuffer& operator=(const PointCloudBuffer&) = delete;
  PointCloudBuffer(PointCloudBuffer&&) = delete;
  PointCloudBuffer& operator=(PointCloudBuffer&&) = delete;

  size_t len() const;
  const PackedPointXYZIRT* data() const;
  uint32_t height() const;
  uint32_t width() const;
  bool is_dense() const;
  double timestamp() const;
  uint32_t seq() const;
  const std::string& frame_id() const;
  void clear();

  void assign_from(const PointCloudT<PointXYZIRT>& cloud);

private:
  std::vector<PackedPointXYZIRT> points_;
  uint32_t height_;
  uint32_t width_;
  bool is_dense_;
  double timestamp_;
  uint32_t seq_;
  std::string frame_id_;
};

class Driver
{
public:
  Driver();
  ~Driver();

  // Delete copy and move operations
  Driver(const Driver&) = delete;
  Driver& operator=(const Driver&) = delete;
  Driver(Driver&&) = delete;
  Driver& operator=(Driver&&) = delete;

  bool init(const robosense::lidar::RSDriverParam& param);
  bool start();
  void stop();
  bool poll_point_cloud(uint64_t timeout_ms, PointCloudBuffer& out);
  bool get_temperature(float& temp);
  bool get_device_info(robosense::lidar::DeviceInfo& info);
  bool get_device_status(robosense::lidar::DeviceStatus& status);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

std::unique_ptr<Driver> driver_create();
std::unique_ptr<PointCloudBuffer> point_cloud_buffer_create();

std::unique_ptr<robosense::lidar::RSDriverParam> driver_param_create();
robosense::lidar::RSInputParam& driver_param_input(robosense::lidar::RSDriverParam& param);
robosense::lidar::RSDecoderParam& driver_param_decoder(robosense::lidar::RSDriverParam& param);
robosense::lidar::RSTransformParam& decoder_param_transform(robosense::lidar::RSDecoderParam& param);

void driver_param_set_lidar_type(robosense::lidar::RSDriverParam& param, robosense::lidar::LidarType type);
robosense::lidar::LidarType driver_param_get_lidar_type(const robosense::lidar::RSDriverParam& param);

void driver_param_set_input_type(robosense::lidar::RSDriverParam& param, robosense::lidar::InputType type);
robosense::lidar::InputType driver_param_get_input_type(const robosense::lidar::RSDriverParam& param);

void driver_param_set_frame_id(robosense::lidar::RSDriverParam& param, rust::Str frame_id);
const std::string& driver_param_get_frame_id(const robosense::lidar::RSDriverParam& param);

void input_param_set_host_address(robosense::lidar::RSInputParam& param, rust::Str addr);
const std::string& input_param_get_host_address(const robosense::lidar::RSInputParam& param);

void input_param_set_group_address(robosense::lidar::RSInputParam& param, rust::Str addr);
const std::string& input_param_get_group_address(const robosense::lidar::RSInputParam& param);

void input_param_set_msop_port(robosense::lidar::RSInputParam& param, uint16_t port);
uint16_t input_param_get_msop_port(const robosense::lidar::RSInputParam& param);

void input_param_set_difop_port(robosense::lidar::RSInputParam& param, uint16_t port);
uint16_t input_param_get_difop_port(const robosense::lidar::RSInputParam& param);

void input_param_set_imu_port(robosense::lidar::RSInputParam& param, uint16_t port);
uint16_t input_param_get_imu_port(const robosense::lidar::RSInputParam& param);

void input_param_set_user_layer_bytes(robosense::lidar::RSInputParam& param, uint16_t bytes);
uint16_t input_param_get_user_layer_bytes(const robosense::lidar::RSInputParam& param);

void input_param_set_tail_layer_bytes(robosense::lidar::RSInputParam& param, uint16_t bytes);
uint16_t input_param_get_tail_layer_bytes(const robosense::lidar::RSInputParam& param);

void input_param_set_socket_recv_buf(robosense::lidar::RSInputParam& param, uint32_t bytes);
uint32_t input_param_get_socket_recv_buf(const robosense::lidar::RSInputParam& param);

void input_param_set_pcap_path(robosense::lidar::RSInputParam& param, rust::Str path);
const std::string& input_param_get_pcap_path(const robosense::lidar::RSInputParam& param);

void input_param_set_pcap_repeat(robosense::lidar::RSInputParam& param, bool repeat);
bool input_param_get_pcap_repeat(const robosense::lidar::RSInputParam& param);

void input_param_set_pcap_rate(robosense::lidar::RSInputParam& param, float rate);
float input_param_get_pcap_rate(const robosense::lidar::RSInputParam& param);

void input_param_set_use_vlan(robosense::lidar::RSInputParam& param, bool enabled);
bool input_param_get_use_vlan(const robosense::lidar::RSInputParam& param);

void decoder_param_set_min_distance(robosense::lidar::RSDecoderParam& param, float value);
float decoder_param_get_min_distance(const robosense::lidar::RSDecoderParam& param);

void decoder_param_set_max_distance(robosense::lidar::RSDecoderParam& param, float value);
float decoder_param_get_max_distance(const robosense::lidar::RSDecoderParam& param);

void decoder_param_set_use_lidar_clock(robosense::lidar::RSDecoderParam& param, bool value);
bool decoder_param_get_use_lidar_clock(const robosense::lidar::RSDecoderParam& param);

void decoder_param_set_dense_points(robosense::lidar::RSDecoderParam& param, bool value);
bool decoder_param_get_dense_points(const robosense::lidar::RSDecoderParam& param);

void decoder_param_set_ts_first_point(robosense::lidar::RSDecoderParam& param, bool value);
bool decoder_param_get_ts_first_point(const robosense::lidar::RSDecoderParam& param);

void decoder_param_set_wait_for_difop(robosense::lidar::RSDecoderParam& param, bool value);
bool decoder_param_get_wait_for_difop(const robosense::lidar::RSDecoderParam& param);

void decoder_param_set_config_from_file(robosense::lidar::RSDecoderParam& param, bool value);
bool decoder_param_get_config_from_file(const robosense::lidar::RSDecoderParam& param);

void decoder_param_set_angle_path(robosense::lidar::RSDecoderParam& param, rust::Str path);
const std::string& decoder_param_get_angle_path(const robosense::lidar::RSDecoderParam& param);

void decoder_param_set_start_angle(robosense::lidar::RSDecoderParam& param, float angle);
float decoder_param_get_start_angle(const robosense::lidar::RSDecoderParam& param);

void decoder_param_set_end_angle(robosense::lidar::RSDecoderParam& param, float angle);
float decoder_param_get_end_angle(const robosense::lidar::RSDecoderParam& param);

void decoder_param_set_split_frame_mode(robosense::lidar::RSDecoderParam& param, robosense::lidar::SplitFrameMode mode);
robosense::lidar::SplitFrameMode decoder_param_get_split_frame_mode(const robosense::lidar::RSDecoderParam& param);

void decoder_param_set_split_angle(robosense::lidar::RSDecoderParam& param, float angle);
float decoder_param_get_split_angle(const robosense::lidar::RSDecoderParam& param);

void decoder_param_set_num_blks_split(robosense::lidar::RSDecoderParam& param, uint16_t num);
uint16_t decoder_param_get_num_blks_split(const robosense::lidar::RSDecoderParam& param);

void transform_param_set_x(robosense::lidar::RSTransformParam& param, float value);
float transform_param_get_x(const robosense::lidar::RSTransformParam& param);

void transform_param_set_y(robosense::lidar::RSTransformParam& param, float value);
float transform_param_get_y(const robosense::lidar::RSTransformParam& param);

void transform_param_set_z(robosense::lidar::RSTransformParam& param, float value);
float transform_param_get_z(const robosense::lidar::RSTransformParam& param);

void transform_param_set_roll(robosense::lidar::RSTransformParam& param, float value);
float transform_param_get_roll(const robosense::lidar::RSTransformParam& param);

void transform_param_set_pitch(robosense::lidar::RSTransformParam& param, float value);
float transform_param_get_pitch(const robosense::lidar::RSTransformParam& param);

void transform_param_set_yaw(robosense::lidar::RSTransformParam& param, float value);
float transform_param_get_yaw(const robosense::lidar::RSTransformParam& param);

std::string driver_version();

}  // namespace carvi_rsd
