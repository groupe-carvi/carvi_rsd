mod bindings;

use autocxx::prelude::*;
use bindings::carvi_rsd;
use std::fmt;
use std::pin::Pin;
use std::time::Duration;

pub use bindings::{
    carvi_rsd::PackedPointXYZIRT, robosense::lidar::DeviceInfo, robosense::lidar::DeviceStatus,
    robosense::lidar::InputType, robosense::lidar::LidarType, robosense::lidar::SplitFrameMode,
};

pub type Result<T> = std::result::Result<T, DriverError>;

#[derive(Debug)]
pub struct DriverError {
    message: String,
}

impl DriverError {
    fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
        }
    }
}

impl fmt::Display for DriverError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.message)
    }
}

impl std::error::Error for DriverError {}

#[derive(Debug, Clone)]
pub struct PointXYZIRT {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: u8,
    pub ring: u16,
    pub timestamp: f64,
}

#[derive(Debug, Clone)]
pub struct PointCloud {
    pub height: u32,
    pub width: u32,
    pub is_dense: bool,
    pub timestamp: f64,
    pub sequence: u32,
    pub frame_id: String,
    pub points: Vec<PointXYZIRT>,
}

pub struct Driver {
    inner: UniquePtr<carvi_rsd::Driver>,
    buffer: UniquePtr<carvi_rsd::PointCloudBuffer>,
}

impl Driver {
    pub fn new() -> Result<Self> {
        let inner = carvi_rsd::driver_create();
        let buffer = carvi_rsd::point_cloud_buffer_create();
        Ok(Self { inner, buffer })
    }

    pub fn init(&mut self, config: &DriverConfig) -> Result<()> {
        let success = {
            self.inner.pin_mut().init(
                config
                    .raw
                    .as_ref()
                    .ok_or_else(|| DriverError::new("invalid config"))?,
            )
        };
        if success {
            Ok(())
        } else {
            Err(DriverError::new("failed to initialize driver"))
        }
    }

    pub fn start(&mut self) -> Result<()> {
        if self.inner.pin_mut().start() {
            Ok(())
        } else {
            Err(DriverError::new("failed to start driver"))
        }
    }

    pub fn stop(&mut self) {
        {
            self.inner.pin_mut().stop();
        }
    }

    pub fn get_temperature(&mut self) -> Result<f32> {
        let mut temp = 0.0f32;
        if unsafe {
            self.inner
                .pin_mut()
                .get_temperature(Pin::new_unchecked(&mut temp))
        } {
            Ok(temp)
        } else {
            Err(DriverError::new("unable to fetch temperature"))
        }
    }

    pub fn get_device_info(&mut self) -> Result<DeviceInfo> {
        let mut info = unsafe {
            let uninit: std::mem::MaybeUninit<DeviceInfo> = std::mem::MaybeUninit::uninit();
            uninit.assume_init()
        };
        if unsafe {
            self.inner
                .pin_mut()
                .get_device_info(Pin::new_unchecked(&mut info))
        } {
            Ok(info)
        } else {
            Err(DriverError::new("unable to fetch device info"))
        }
    }

    pub fn get_device_status(&mut self) -> Result<DeviceStatus> {
        let mut status = unsafe {
            let uninit: std::mem::MaybeUninit<DeviceStatus> = std::mem::MaybeUninit::uninit();
            uninit.assume_init()
        };
        if unsafe {
            self.inner
                .pin_mut()
                .get_device_status(Pin::new_unchecked(&mut status))
        } {
            Ok(status)
        } else {
            Err(DriverError::new("unable to fetch device status"))
        }
    }

    pub fn poll_point_cloud(&mut self, timeout: Duration) -> Result<Option<PointCloud>> {
        let timeout_ms = timeout.as_millis().try_into().unwrap_or(u64::MAX);
        let has_frame = {
            self.inner
                .pin_mut()
                .poll_point_cloud(timeout_ms, self.buffer.pin_mut())
        };
        if !has_frame {
            return Ok(None);
        }

        let buffer = self
            .buffer
            .as_ref()
            .ok_or_else(|| DriverError::new("buffer unavailable"))?;
        let points = unsafe {
            let len = buffer.len();
            let raw = std::slice::from_raw_parts(buffer.data(), len);
            raw.iter().map(PointXYZIRT::from).collect::<Vec<_>>()
        };

        let frame_id = buffer.frame_id().to_string_lossy().into_owned();
        Ok(Some(PointCloud {
            height: buffer.height(),
            width: buffer.width(),
            is_dense: buffer.is_dense(),
            timestamp: buffer.timestamp(),
            sequence: buffer.seq(),
            frame_id,
            points,
        }))
    }
}

impl From<&PackedPointXYZIRT> for PointXYZIRT {
    fn from(value: &PackedPointXYZIRT) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
            intensity: value.intensity,
            ring: value.ring,
            timestamp: value.timestamp,
        }
    }
}

pub struct DriverConfig {
    raw: UniquePtr<bindings::robosense::lidar::RSDriverParam>,
}

impl Default for DriverConfig {
    fn default() -> Self {
        Self::new()
    }
}

impl DriverConfig {
    pub fn new() -> Self {
        Self {
            raw: carvi_rsd::driver_param_create(),
        }
    }

    pub fn lidar_type(&mut self, ty: LidarType) -> &mut Self {
        carvi_rsd::driver_param_set_lidar_type(self.raw.pin_mut(), ty);
        self
    }

    pub fn input_type(&mut self, ty: InputType) -> &mut Self {
        carvi_rsd::driver_param_set_input_type(self.raw.pin_mut(), ty);
        self
    }

    pub fn frame_id(&mut self, id: &str) -> &mut Self {
        carvi_rsd::driver_param_set_frame_id(self.raw.pin_mut(), id);
        self
    }

    pub fn input<'a>(&'a mut self) -> DriverInput<'a> {
        let pin = { carvi_rsd::driver_param_input(self.raw.pin_mut()) };
        DriverInput { raw: pin }
    }

    pub fn decoder<'a>(&'a mut self) -> DriverDecoder<'a> {
        let pin = { carvi_rsd::driver_param_decoder(self.raw.pin_mut()) };
        DriverDecoder { raw: pin }
    }

    pub fn as_ptr(&self) -> Option<&bindings::robosense::lidar::RSDriverParam> {
        self.raw.as_ref()
    }
}

pub struct DriverInput<'a> {
    raw: Pin<&'a mut bindings::robosense::lidar::RSInputParam>,
}

impl<'a> DriverInput<'a> {
    pub fn host_address(&mut self, addr: &str) {
        carvi_rsd::input_param_set_host_address(self.raw.as_mut(), addr);
    }

    pub fn group_address(&mut self, addr: &str) {
        carvi_rsd::input_param_set_group_address(self.raw.as_mut(), addr);
    }

    pub fn msop_port(&mut self, port: u16) {
        carvi_rsd::input_param_set_msop_port(self.raw.as_mut(), port);
    }

    pub fn difop_port(&mut self, port: u16) {
        carvi_rsd::input_param_set_difop_port(self.raw.as_mut(), port);
    }

    pub fn imu_port(&mut self, port: u16) {
        carvi_rsd::input_param_set_imu_port(self.raw.as_mut(), port);
    }

    pub fn user_layer_bytes(&mut self, bytes: u16) {
        carvi_rsd::input_param_set_user_layer_bytes(self.raw.as_mut(), bytes);
    }

    pub fn tail_layer_bytes(&mut self, bytes: u16) {
        carvi_rsd::input_param_set_tail_layer_bytes(self.raw.as_mut(), bytes);
    }

    pub fn socket_recv_buf(&mut self, bytes: u32) {
        carvi_rsd::input_param_set_socket_recv_buf(self.raw.as_mut(), bytes);
    }

    pub fn pcap_path(&mut self, path: &str) {
        carvi_rsd::input_param_set_pcap_path(self.raw.as_mut(), path);
    }

    pub fn pcap_repeat(&mut self, repeat: bool) {
        carvi_rsd::input_param_set_pcap_repeat(self.raw.as_mut(), repeat);
    }

    pub fn pcap_rate(&mut self, rate: f32) {
        carvi_rsd::input_param_set_pcap_rate(self.raw.as_mut(), rate);
    }

    pub fn use_vlan(&mut self, enabled: bool) {
        carvi_rsd::input_param_set_use_vlan(self.raw.as_mut(), enabled);
    }
}

pub struct DriverDecoder<'a> {
    raw: Pin<&'a mut bindings::robosense::lidar::RSDecoderParam>,
}

impl<'a> DriverDecoder<'a> {
    pub fn min_distance(&mut self, value: f32) {
        carvi_rsd::decoder_param_set_min_distance(self.raw.as_mut(), value);
    }

    pub fn max_distance(&mut self, value: f32) {
        carvi_rsd::decoder_param_set_max_distance(self.raw.as_mut(), value);
    }

    pub fn use_lidar_clock(&mut self, enabled: bool) {
        carvi_rsd::decoder_param_set_use_lidar_clock(self.raw.as_mut(), enabled);
    }

    pub fn dense_points(&mut self, enabled: bool) {
        carvi_rsd::decoder_param_set_dense_points(self.raw.as_mut(), enabled);
    }

    pub fn timestamp_first_point(&mut self, enabled: bool) {
        carvi_rsd::decoder_param_set_ts_first_point(self.raw.as_mut(), enabled);
    }

    pub fn wait_for_difop(&mut self, enabled: bool) {
        carvi_rsd::decoder_param_set_wait_for_difop(self.raw.as_mut(), enabled);
    }

    pub fn config_from_file(&mut self, enabled: bool) {
        carvi_rsd::decoder_param_set_config_from_file(self.raw.as_mut(), enabled);
    }

    pub fn angle_path(&mut self, path: &str) {
        carvi_rsd::decoder_param_set_angle_path(self.raw.as_mut(), path);
    }

    pub fn start_angle(&mut self, angle: f32) {
        carvi_rsd::decoder_param_set_start_angle(self.raw.as_mut(), angle);
    }

    pub fn end_angle(&mut self, angle: f32) {
        carvi_rsd::decoder_param_set_end_angle(self.raw.as_mut(), angle);
    }

    pub fn split_frame_mode(&mut self, mode: SplitFrameMode) {
        carvi_rsd::decoder_param_set_split_frame_mode(self.raw.as_mut(), mode);
    }

    pub fn split_angle(&mut self, angle: f32) {
        carvi_rsd::decoder_param_set_split_angle(self.raw.as_mut(), angle);
    }

    pub fn num_blocks(&mut self, count: u16) {
        carvi_rsd::decoder_param_set_num_blks_split(self.raw.as_mut(), count);
    }

    pub fn transform(&mut self) -> TransformParams<'_> {
        let pin = { carvi_rsd::decoder_param_transform(self.raw.as_mut()) };
        TransformParams { raw: pin }
    }
}

pub struct TransformParams<'a> {
    raw: Pin<&'a mut bindings::robosense::lidar::RSTransformParam>,
}

impl<'a> TransformParams<'a> {
    pub fn x(&mut self, value: f32) {
        carvi_rsd::transform_param_set_x(self.raw.as_mut(), value);
    }
    pub fn y(&mut self, value: f32) {
        carvi_rsd::transform_param_set_y(self.raw.as_mut(), value);
    }
    pub fn z(&mut self, value: f32) {
        carvi_rsd::transform_param_set_z(self.raw.as_mut(), value);
    }
    pub fn roll(&mut self, value: f32) {
        carvi_rsd::transform_param_set_roll(self.raw.as_mut(), value);
    }
    pub fn pitch(&mut self, value: f32) {
        carvi_rsd::transform_param_set_pitch(self.raw.as_mut(), value);
    }
    pub fn yaw(&mut self, value: f32) {
        carvi_rsd::transform_param_set_yaw(self.raw.as_mut(), value);
    }
}

pub fn driver_version() -> String {
    carvi_rsd::driver_version().to_string_lossy().into_owned()
}
