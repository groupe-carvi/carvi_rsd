use anyhow::{Context, Result};
use carvi_rsd::{Driver, DriverConfig, InputType, LidarType};
use rerun::external::glam;
use std::time::Duration;

fn main() -> Result<()> {
    // Start a gRPC server and use it as log sink
    let rec = rerun::RecordingStreamBuilder::new("RoboSense Airy LiDAR")
        .serve_grpc()
        .context("Failed to start gRPC server")?;

    // Connect the web viewer to the gRPC server and open it in the browser
    #[cfg(feature = "web_viewer")]
    let _server_guard = rerun::serve_web_viewer(rerun::web_viewer::WebViewerConfig {
        connect_to: vec!["rerun+http://localhost/proxy".to_owned()],
        ..Default::default()
    })?;

    #[cfg(not(feature = "web_viewer"))]
    println!("Note: Enable 'web_viewer' feature to automatically open the web browser");
    println!("Web viewer available at: http://localhost:9090 (or check console output)");
    println!("Initializing RoboSense Airy LiDAR driver...");

    // Create and configure the driver for Airy lidar
    let mut config = DriverConfig::new();
    config
        .lidar_type(LidarType::RSAIRY)
        .input_type(InputType::ONLINE_LIDAR)
        .frame_id("lidar_frame");

    // Configure input parameters
    {
        let mut input = config.input();
        input.msop_port(6699); // Default MSOP port for RoboSense
        input.difop_port(7788); // Default DIFOP port for RoboSense
        input.host_address("0.0.0.0"); // Listen on all interfaces
        input.group_address("0.0.0.0"); // No multicast
    }

    // Configure decoder parameters
    {
        let mut decoder = config.decoder();
        decoder.min_distance(0.2); // Minimum distance in meters
        decoder.max_distance(200.0); // Maximum distance in meters
        decoder.use_lidar_clock(false); // Use host clock
        decoder.dense_points(false); // Include invalid points
        decoder.wait_for_difop(true); // Wait for device info packet
    }

    // Create and initialize the driver
    let mut driver = Driver::new().context("Failed to create driver")?;
    driver
        .init(&config)
        .context("Failed to initialize driver")?;

    println!("Starting driver...");
    driver.start().context("Failed to start driver")?;

    // Try to get device info
    match driver.get_device_info() {
        Ok(_info) => {
            println!("Device Info retrieved successfully");
        }
        Err(e) => println!("Warning: Could not get device info: {}", e),
    }

    // Try to get temperature
    match driver.get_temperature() {
        Ok(temp) => println!("Temperature: {:.1}°C", temp),
        Err(e) => println!("Warning: Could not get temperature: {}", e),
    }

    println!("Polling for point cloud data... (Press Ctrl+C to stop)");
    println!("View the data in Rerun Viewer");

    let mut frame_count = 0u64;

    loop {
        // Poll for point cloud with 1 second timeout
        match driver.poll_point_cloud(Duration::from_secs(1)) {
            Ok(Some(cloud)) => {
                frame_count += 1;

                println!(
                    "Frame {}: {} points, timestamp: {:.3}s, seq: {}",
                    frame_count,
                    cloud.points.len(),
                    cloud.timestamp,
                    cloud.sequence
                );

                // Log the point cloud to Rerun
                if !cloud.points.is_empty() {
                    // Extract positions, colors (based on intensity), and ring IDs
                    let positions: Vec<glam::Vec3> = cloud
                        .points
                        .iter()
                        .map(|p| glam::Vec3::new(p.x, p.y, p.z))
                        .collect();

                    // Color based on intensity (grayscale)
                    let colors: Vec<rerun::Color> = cloud
                        .points
                        .iter()
                        .map(|p| {
                            let intensity = p.intensity;
                            rerun::Color::from_rgb(intensity, intensity, intensity)
                        })
                        .collect();

                    // Log the point cloud
                    rec.log(
                        "lidar/points",
                        &rerun::Points3D::new(positions)
                            .with_colors(colors)
                            .with_radii([0.02]), // Point size in meters
                    )?;

                    // Optionally log ring information as separate entities
                    // Group points by ring
                    let mut rings: std::collections::HashMap<u16, Vec<glam::Vec3>> =
                        std::collections::HashMap::new();

                    for point in &cloud.points {
                        rings
                            .entry(point.ring)
                            .or_insert_with(Vec::new)
                            .push(glam::Vec3::new(point.x, point.y, point.z));
                    }

                    // Log each ring separately (optional, commented out for performance)
                    // for (ring_id, ring_points) in rings {
                    //     rec.log(
                    //         format!("lidar/rings/{}", ring_id),
                    //         &rerun::Points3D::new(ring_points).with_radii([0.01]),
                    //     )?;
                    // }
                }
            }
            Ok(None) => {
                println!("No point cloud received (timeout)");
            }
            Err(e) => {
                eprintln!("Error polling point cloud: {}", e);
                break;
            }
        }
    }

    println!("Stopping driver...");
    driver.stop();

    // Keep server running a bit longer to ensure data is sent
    println!("Waiting for data to be sent to web viewer...");
    std::thread::sleep(Duration::from_secs(2));

    Ok(())
}
