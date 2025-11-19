# RoboSense Airy LiDAR Examples

## Airy Rerun Example

This example demonstrates how to capture point cloud data from a RoboSense Airy lidar and visualize it in real-time using [Rerun](https://rerun.io/).

### Prerequisites

1. **Hardware**: RoboSense Airy LiDAR connected to your network
2. **Network Configuration**: Ensure your computer can receive UDP packets from the lidar
   - Default MSOP port: 6699
   - Default DIFOP port: 7788
   - Configure your lidar's destination IP to your computer's IP address

3. **Rerun Viewer**: Install the Rerun viewer to visualize the data
   ```bash
   cargo install rerun-cli
   ```

### Running the Example

**Run the example**:
```bash
cargo run --example airy_rerun --features web_viewer
```

This will:
1. Start a gRPC server for data streaming
2. Automatically open a web browser with the Rerun viewer
3. Begin streaming point cloud data from the Airy lidar

If the web browser doesn't open automatically, navigate to the URL shown in the console output (typically `http://localhost:9090`).

### Configuration

The example uses the following default configuration:

- **LiDAR Type**: RSAIRY
- **Input Type**: ONLINE_LIDAR (live data from sensor)
- **MSOP Port**: 6699
- **DIFOP Port**: 7788
- **Host Address**: 0.0.0.0 (listen on all interfaces)
- **Min Distance**: 0.2 meters
- **Max Distance**: 200.0 meters

You can modify these parameters in the source code (`examples/airy_rerun.rs`) to match your setup.

### What You'll See

The example will:
1. Initialize the driver and configure it for the Airy lidar
2. Attempt to retrieve device information (serial number, firmware version)
3. Display the current sensor temperature
4. Continuously poll for point cloud data
5. Log each frame to Rerun with:
   - 3D point positions
   - Intensity-based coloring (grayscale)
   - Frame sequence and timestamp information

In the Rerun viewer, you'll see:
- **lidar/points**: The complete point cloud colored by intensity

### Troubleshooting

**No point cloud received:**
- Verify the lidar is powered and connected to your network
- Check that the destination IP on the lidar is set to your computer's IP
- Ensure no firewall is blocking UDP ports 6699 and 7788
- Use `tcpdump` or Wireshark to verify packets are arriving:
  ```bash
  sudo tcpdump -i <interface> port 6699 or port 7788
  ```

**Device info not available:**
- This is normal if DIFOP packets haven't been received yet
- The driver will still work and produce point clouds

**Build errors:**
- Ensure the rs-driver C++ library is properly built
- Check that all dependencies are installed (see main README)

### Advanced Usage

**Custom Network Configuration:**

Modify the input parameters in the example:

```rust
config
    .input()
    .msop_port(6699)           // Change if using non-default port
    .difop_port(7788)          // Change if using non-default port
    .host_address("192.168.1.100");  // Bind to specific interface
```

**Point Cloud Filtering:**

Adjust the decoder distance filters:

```rust
config
    .decoder()
    .min_distance(1.0)         // Filter points closer than 1m
    .max_distance(50.0);       // Filter points farther than 50m
```

**Coordinate Transform:**

Apply a transform to the point cloud:

```rust
config
    .decoder()
    .transform()
    .x(0.0)
    .y(0.0)
    .z(1.5)    // Offset 1.5m in Z
    .roll(0.0)
    .pitch(0.0)
    .yaw(0.0);
```

### Performance Tips

- The example logs every point in every frame, which can be data-intensive
- For better performance with high-frequency lidars, consider:
  - Downsampling the point cloud before logging
  - Logging only every Nth frame
  - Using the `dense_points(true)` option to filter invalid points

### License

This example is provided under the same license as the carvi_rsd crate.
