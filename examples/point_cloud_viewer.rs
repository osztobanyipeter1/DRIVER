use ar_drivers::{any_glasses, DisplayMode, GlassesEvent};
use nalgebra::{Point3, Rotation3, UnitQuaternion, Vector3};
use std::f32::consts::PI;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Connect to glasses
    let mut glasses = any_glasses()?;
    println!("Connected to glasses: {}", glasses.name());

    // Set to 3D mode
    glasses.set_display_mode(DisplayMode::Stereo)?;

    // Generate a spherical point cloud
    let point_cloud = generate_sphere_point_cloud(1000, 5.0);
    let mut orientation = UnitQuaternion::identity();

    // Main rendering loop
    let mut last_update = Instant::now();
    let alpha = 0.98; // Complementary filter coefficient

    loop {
        // Process sensor events
        if let Ok(event) = glasses.read_event() {
            match event {
                GlassesEvent::AccGyro { accelerometer, gyroscope, .. } => {
                    // Update orientation based on gyroscope data
                    let delta_time = last_update.elapsed().as_secs_f32();
                    last_update = Instant::now();

                    // Simple integration of gyro data to get orientation change
                    let delta_rotation = Rotation3::new(gyroscope * delta_time);
                    orientation = orientation * UnitQuaternion::from_rotation_matrix(&delta_rotation);

                    // Complementary filter with accelerometer
                    if accelerometer.norm() > 0.1 { // Avoid during freefall or strong acceleration
                        let accel_dir = accelerometer.normalize();
                        let pitch = accel_dir.y.atan2(accel_dir.z);
                        let roll = (-accel_dir.x).atan2(accel_dir.z);
                        
                        let accel_quat = UnitQuaternion::from_euler_angles(roll, pitch, 0.0);
                        let lerped = orientation.lerp(&accel_quat, 1.0 - alpha);
                        orientation = UnitQuaternion::new_normalize(lerped);
                    }

                    println!("Orientation: {:?}", orientation.euler_angles());
                }
                _ => {}
            }
        }
        std::thread::sleep(Duration::from_millis(16));
    }
}

fn generate_sphere_point_cloud(num_points: usize, radius: f32) -> Vec<Point3<f32>> {
    let mut points = Vec::with_capacity(num_points);
    
    // Use Fibonacci sphere algorithm for even distribution
    let golden_angle = PI * (3.0 - 5.0f32.sqrt());
    
    for i in 0..num_points {
        let y = 1.0 - (i as f32 / (num_points as f32 - 1.0)) * 2.0;  // y goes from 1 to -1
        let radius_at_y = (1.0 - y * y).sqrt();  // radius at y
        
        let theta = golden_angle * i as f32;
        
        let x = theta.cos() * radius_at_y;
        let z = theta.sin() * radius_at_y;
        
        points.push(Point3::new(x * radius, y * radius, z * radius));
    }
    
    points
}