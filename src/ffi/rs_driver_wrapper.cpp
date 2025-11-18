#include "rs_driver_wrapper.hpp"

#include <algorithm>
#include <chrono>
#include <utility>

// Include full rs_driver headers for implementation
#include "rs_driver/msg/point_cloud_msg.hpp"

using robosense::lidar::InputType;
using robosense::lidar::LidarType;
using robosense::lidar::RSDecoderParam;
using robosense::lidar::RSDriverParam;
using robosense::lidar::RSInputParam;
using robosense::lidar::RSTransformParam;
using robosense::lidar::SplitFrameMode;
// Note: PointXYZIRT and PointCloudT are from point_cloud_msg.hpp

namespace carvi_rsd
{

PointCloudBuffer::PointCloudBuffer()
  : height_(0)
  , width_(0)
  , is_dense_(false)
  , timestamp_(0.0)
  , seq_(0)
{
}

PointCloudBuffer::~PointCloudBuffer() = default;

size_t PointCloudBuffer::len() const
{
  return points_.size();
}

const PackedPointXYZIRT* PointCloudBuffer::data() const
{
  return points_.data();
}

uint32_t PointCloudBuffer::height() const
{
  return height_;
}

uint32_t PointCloudBuffer::width() const
{
  return width_;
}

bool PointCloudBuffer::is_dense() const
{
  return is_dense_;
}

double PointCloudBuffer::timestamp() const
{
  return timestamp_;
}

uint32_t PointCloudBuffer::seq() const
{
  return seq_;
}

const std::string& PointCloudBuffer::frame_id() const
{
  return frame_id_;
}

void PointCloudBuffer::clear()
{
  points_.clear();
  height_ = 0U;
  width_ = 0U;
  is_dense_ = false;
  timestamp_ = 0.0;
  seq_ = 0U;
  frame_id_.clear();
}

void PointCloudBuffer::assign_from(const PointCloudT<PointXYZIRT>& cloud)
{
  points_.resize(cloud.points.size());
  std::transform(
    cloud.points.begin(), cloud.points.end(), points_.begin(),
    [](const PointXYZIRT& src) {
      PackedPointXYZIRT dst{};
      dst.x = src.x;
      dst.y = src.y;
      dst.z = src.z;
      dst.intensity = src.intensity;
      dst.ring = src.ring;
      dst.timestamp = src.timestamp;
      return dst;
    });
  height_ = cloud.height;
  width_ = cloud.width;
  is_dense_ = cloud.is_dense;
  timestamp_ = cloud.timestamp;
  seq_ = cloud.seq;
  frame_id_ = cloud.frame_id;
}

// Driver::Impl - private implementation
struct Driver::Impl
{
  using Cloud = PointCloudT<PointXYZIRT>;

  Impl()
    : running_(false)
    , initialized_(false)
  {
  }

  std::shared_ptr<Cloud> allocate_cloud()
  {
    return std::make_shared<Cloud>();
  }

  void on_cloud_ready(std::shared_ptr<Cloud> cloud)
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      ready_queue_.push_back(std::move(cloud));
    }
    queue_cv_.notify_one();
  }

  robosense::lidar::LidarDriver<Cloud> driver_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<std::shared_ptr<Cloud>> ready_queue_;
  bool running_;
  bool initialized_;
};

Driver::Driver()
  : impl_(std::make_unique<Impl>())
{
  impl_->driver_.regPointCloudCallback(
    [this]() { return this->impl_->allocate_cloud(); },
    [this](std::shared_ptr<Impl::Cloud> cloud) { this->impl_->on_cloud_ready(std::move(cloud)); });
}

Driver::~Driver()
{
  stop();
}

bool Driver::init(const RSDriverParam& param)
{
  impl_->initialized_ = impl_->driver_.init(param);
  return impl_->initialized_;
}

bool Driver::start()
{
  if (!impl_->initialized_)
  {
    return false;
  }
  impl_->running_ = impl_->driver_.start();
  return impl_->running_;
}

void Driver::stop()
{
  impl_->driver_.stop();
  {
    std::lock_guard<std::mutex> lock(impl_->queue_mutex_);
    impl_->running_ = false;
    impl_->ready_queue_.clear();
  }
  impl_->queue_cv_.notify_all();
}

bool Driver::poll_point_cloud(uint64_t timeout_ms, PointCloudBuffer& out)
{
  std::unique_lock<std::mutex> lock(impl_->queue_mutex_);
  auto predicate = [this]() { return !impl_->ready_queue_.empty() || !impl_->running_; };

  if (timeout_ms == 0U)
  {
    impl_->queue_cv_.wait(lock, predicate);
  }
  else
  {
    if (!impl_->queue_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), predicate))
    {
      return false;
    }
  }

  if (impl_->ready_queue_.empty())
  {
    return false;
  }

  auto cloud = impl_->ready_queue_.front();
  impl_->ready_queue_.pop_front();
  lock.unlock();

  if (!cloud)
  {
    return false;
  }

  out.assign_from(*cloud);
  return true;
}

bool Driver::get_temperature(float& temp)
{
  return impl_->driver_.getTemperature(temp);
}

bool Driver::get_device_info(robosense::lidar::DeviceInfo& info)
{
  return impl_->driver_.getDeviceInfo(info);
}

bool Driver::get_device_status(robosense::lidar::DeviceStatus& status)
{
  return impl_->driver_.getDeviceStatus(status);
}

std::unique_ptr<Driver> driver_create()
{
  return std::make_unique<Driver>();
}

std::unique_ptr<PointCloudBuffer> point_cloud_buffer_create()
{
  return std::make_unique<PointCloudBuffer>();
}

std::unique_ptr<RSDriverParam> driver_param_create()
{
  return std::make_unique<RSDriverParam>();
}

RSInputParam& driver_param_input(RSDriverParam& param)
{
  return param.input_param;
}

RSDecoderParam& driver_param_decoder(RSDriverParam& param)
{
  return param.decoder_param;
}

RSTransformParam& decoder_param_transform(RSDecoderParam& param)
{
  return param.transform_param;
}

void driver_param_set_lidar_type(RSDriverParam& param, LidarType type)
{
  param.lidar_type = type;
}

LidarType driver_param_get_lidar_type(const RSDriverParam& param)
{
  return param.lidar_type;
}

void driver_param_set_input_type(RSDriverParam& param, InputType type)
{
  param.input_type = type;
}

InputType driver_param_get_input_type(const RSDriverParam& param)
{
  return param.input_type;
}

void driver_param_set_frame_id(RSDriverParam& param, rust::Str frame_id)
{
  param.frame_id.assign(frame_id.data(), frame_id.size());
}

const std::string& driver_param_get_frame_id(const RSDriverParam& param)
{
  return param.frame_id;
}

void input_param_set_host_address(RSInputParam& param, rust::Str addr)
{
  param.host_address.assign(addr.data(), addr.size());
}

const std::string& input_param_get_host_address(const RSInputParam& param)
{
  return param.host_address;
}

void input_param_set_group_address(RSInputParam& param, rust::Str addr)
{
  param.group_address.assign(addr.data(), addr.size());
}

const std::string& input_param_get_group_address(const RSInputParam& param)
{
  return param.group_address;
}

void input_param_set_msop_port(RSInputParam& param, uint16_t port)
{
  param.msop_port = port;
}

uint16_t input_param_get_msop_port(const RSInputParam& param)
{
  return param.msop_port;
}

void input_param_set_difop_port(RSInputParam& param, uint16_t port)
{
  param.difop_port = port;
}

uint16_t input_param_get_difop_port(const RSInputParam& param)
{
  return param.difop_port;
}

void input_param_set_imu_port(RSInputParam& param, uint16_t port)
{
  param.imu_port = port;
}

uint16_t input_param_get_imu_port(const RSInputParam& param)
{
  return param.imu_port;
}

void input_param_set_user_layer_bytes(RSInputParam& param, uint16_t bytes)
{
  param.user_layer_bytes = bytes;
}

uint16_t input_param_get_user_layer_bytes(const RSInputParam& param)
{
  return param.user_layer_bytes;
}

void input_param_set_tail_layer_bytes(RSInputParam& param, uint16_t bytes)
{
  param.tail_layer_bytes = bytes;
}

uint16_t input_param_get_tail_layer_bytes(const RSInputParam& param)
{
  return param.tail_layer_bytes;
}

void input_param_set_socket_recv_buf(RSInputParam& param, uint32_t bytes)
{
  param.socket_recv_buf = bytes;
}

uint32_t input_param_get_socket_recv_buf(const RSInputParam& param)
{
  return param.socket_recv_buf;
}

void input_param_set_pcap_path(RSInputParam& param, rust::Str path)
{
  param.pcap_path.assign(path.data(), path.size());
}

const std::string& input_param_get_pcap_path(const RSInputParam& param)
{
  return param.pcap_path;
}

void input_param_set_pcap_repeat(RSInputParam& param, bool repeat)
{
  param.pcap_repeat = repeat;
}

bool input_param_get_pcap_repeat(const RSInputParam& param)
{
  return param.pcap_repeat;
}

void input_param_set_pcap_rate(RSInputParam& param, float rate)
{
  param.pcap_rate = rate;
}

float input_param_get_pcap_rate(const RSInputParam& param)
{
  return param.pcap_rate;
}

void input_param_set_use_vlan(RSInputParam& param, bool enabled)
{
  param.use_vlan = enabled;
}

bool input_param_get_use_vlan(const RSInputParam& param)
{
  return param.use_vlan;
}

void decoder_param_set_min_distance(RSDecoderParam& param, float value)
{
  param.min_distance = value;
}

float decoder_param_get_min_distance(const RSDecoderParam& param)
{
  return param.min_distance;
}

void decoder_param_set_max_distance(RSDecoderParam& param, float value)
{
  param.max_distance = value;
}

float decoder_param_get_max_distance(const RSDecoderParam& param)
{
  return param.max_distance;
}

void decoder_param_set_use_lidar_clock(RSDecoderParam& param, bool value)
{
  param.use_lidar_clock = value;
}

bool decoder_param_get_use_lidar_clock(const RSDecoderParam& param)
{
  return param.use_lidar_clock;
}

void decoder_param_set_dense_points(RSDecoderParam& param, bool value)
{
  param.dense_points = value;
}

bool decoder_param_get_dense_points(const RSDecoderParam& param)
{
  return param.dense_points;
}

void decoder_param_set_ts_first_point(RSDecoderParam& param, bool value)
{
  param.ts_first_point = value;
}

bool decoder_param_get_ts_first_point(const RSDecoderParam& param)
{
  return param.ts_first_point;
}

void decoder_param_set_wait_for_difop(RSDecoderParam& param, bool value)
{
  param.wait_for_difop = value;
}

bool decoder_param_get_wait_for_difop(const RSDecoderParam& param)
{
  return param.wait_for_difop;
}

void decoder_param_set_config_from_file(RSDecoderParam& param, bool value)
{
  param.config_from_file = value;
}

bool decoder_param_get_config_from_file(const RSDecoderParam& param)
{
  return param.config_from_file;
}

void decoder_param_set_angle_path(RSDecoderParam& param, rust::Str path)
{
  param.angle_path.assign(path.data(), path.size());
}

const std::string& decoder_param_get_angle_path(const RSDecoderParam& param)
{
  return param.angle_path;
}

void decoder_param_set_start_angle(RSDecoderParam& param, float angle)
{
  param.start_angle = angle;
}

float decoder_param_get_start_angle(const RSDecoderParam& param)
{
  return param.start_angle;
}

void decoder_param_set_end_angle(RSDecoderParam& param, float angle)
{
  param.end_angle = angle;
}

float decoder_param_get_end_angle(const RSDecoderParam& param)
{
  return param.end_angle;
}

void decoder_param_set_split_frame_mode(RSDecoderParam& param, SplitFrameMode mode)
{
  param.split_frame_mode = mode;
}

SplitFrameMode decoder_param_get_split_frame_mode(const RSDecoderParam& param)
{
  return param.split_frame_mode;
}

void decoder_param_set_split_angle(RSDecoderParam& param, float angle)
{
  param.split_angle = angle;
}

float decoder_param_get_split_angle(const RSDecoderParam& param)
{
  return param.split_angle;
}

void decoder_param_set_num_blks_split(RSDecoderParam& param, uint16_t num)
{
  param.num_blks_split = num;
}

uint16_t decoder_param_get_num_blks_split(const RSDecoderParam& param)
{
  return param.num_blks_split;
}

void transform_param_set_x(RSTransformParam& param, float value)
{
  param.x = value;
}

float transform_param_get_x(const RSTransformParam& param)
{
  return param.x;
}

void transform_param_set_y(RSTransformParam& param, float value)
{
  param.y = value;
}

float transform_param_get_y(const RSTransformParam& param)
{
  return param.y;
}

void transform_param_set_z(RSTransformParam& param, float value)
{
  param.z = value;
}

float transform_param_get_z(const RSTransformParam& param)
{
  return param.z;
}

void transform_param_set_roll(RSTransformParam& param, float value)
{
  param.roll = value;
}

float transform_param_get_roll(const RSTransformParam& param)
{
  return param.roll;
}

void transform_param_set_pitch(RSTransformParam& param, float value)
{
  param.pitch = value;
}

float transform_param_get_pitch(const RSTransformParam& param)
{
  return param.pitch;
}

void transform_param_set_yaw(RSTransformParam& param, float value)
{
  param.yaw = value;
}

float transform_param_get_yaw(const RSTransformParam& param)
{
  return param.yaw;
}

std::string driver_version()
{
  return robosense::lidar::getDriverVersion();
}

}  // namespace carvi_rsd
