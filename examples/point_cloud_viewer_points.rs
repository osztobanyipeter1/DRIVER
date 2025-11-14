use ar_drivers::{any_glasses, DisplayMode, GlassesEvent};
use nalgebra::{Point3, Unit, UnitQuaternion, Vector3};
use std::f32::consts::PI;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Connect to glasses
    let mut glasses = any_glasses()?;
    println!("Connected to glasses: {}", glasses.name());

    // Set to 3D mode
    if let Err(e) = glasses.set_display_mode(DisplayMode::Stereo) {
        println!("Failed to set Stereo mode: {}", e);
        glasses.set_display_mode(DisplayMode::SameOnBoth)?;
    }

    // Generate a spherical point cloud
    let point_cloud = generate_sphere_point_cloud(1000, 5.0);
    let mut orientation = UnitQuaternion::identity();
    let mut calibrated = false;
    let mut calibration_samples = Vec::new();

    // IMU calibration
    let mut gyro_bias = Vector3::zeros();
    let mut accel_bias = Vector3::zeros();
    const CALIBRATION_SAMPLES: usize = 1000;

    // Complementary filter parameters
    let alpha = 0.98; // Gyro dominance
    let beta = 1.0 - alpha; // Accel dominance

    // Main rendering loop
    let mut last_update = Instant::now();

    loop {
        // Process sensor events
        if let Ok(event) = glasses.read_event() {
            match event {
                GlassesEvent::AccGyro { accelerometer, gyroscope, timestamp: _ } => {
                    let delta_time = last_update.elapsed().as_secs_f32();
                    last_update = Instant::now();

                    // Calibration phase
                    if !calibrated {
                        calibration_samples.push((accelerometer, gyroscope));
                        
                        if calibration_samples.len() >= CALIBRATION_SAMPLES {
                            // Calculate mean values for bias
                            let (acc_sum, gyro_sum) = calibration_samples.iter().fold(
                                (Vector3::zeros(), Vector3::zeros()),
                                |(acc, gyro), (a, g)| (acc + a, gyro + g)
                            );
                            
                            accel_bias = acc_sum / CALIBRATION_SAMPLES as f32;
                            gyro_bias = gyro_sum / CALIBRATION_SAMPLES as f32;
                            
                            // Gravity should point downward (adjust Z component)
                            accel_bias.z -= 9.81; // Subtract gravity
                            
                            calibrated = true;
                            println!("Calibration complete. Biases - Accel: {:?}, Gyro: {:?}", accel_bias, gyro_bias);
                        }
                        continue;
                    }

                    // Apply calibration
                    let corrected_acc = accelerometer - accel_bias;
                    let corrected_gyro = gyroscope - gyro_bias;

                    // Gyroscope integration (more accurate quaternion integration)
                    let gyro_vec = corrected_gyro * delta_time;
                    let gyro_angle = gyro_vec.norm();
                    
                    if gyro_angle > 0.0 {
                        let gyro_axis = Unit::new_normalize(gyro_vec);
                        let delta_q = UnitQuaternion::from_axis_angle(&gyro_axis, gyro_angle);
                        orientation = orientation * delta_q;
                    }

                    // Accelerometer correction only if not accelerating too much
                    if corrected_acc.norm() > 8.0 && corrected_acc.norm() < 11.0 { // Rough gravity range
                        let normalized_acc = corrected_acc.normalize();
                        
                        // Current up vector according to orientation
                        let current_up = orientation * Vector3::z();
                        
                        // Desired up vector from accelerometer
                        let desired_up = normalized_acc;
                        
                        // Correction quaternion between current and desired up
                        let correction = UnitQuaternion::rotation_between(&current_up, &desired_up).unwrap_or_else(|| {
                            UnitQuaternion::identity()
                        });
                        
                        // Apply complementary filter
                        orientation = orientation.slerp(&(correction * orientation), beta);
                    }

                    // Check orientation validity
                    let quat = orientation.quaternion();
                    if !quat.coords.iter().all(|&x| x.is_finite()) {
                        println!("Invalid orientation detected! Resetting...");
                        orientation = UnitQuaternion::identity();
                    }

                    // Print Euler angles for debugging (roll, pitch, yaw)
                    let (roll, pitch, yaw) = orientation.euler_angles();
                    println!("Orientation - Roll: {:.2}°, Pitch: {:.2}°, Yaw: {:.2}°", 
                        roll.to_degrees(), pitch.to_degrees(), yaw.to_degrees());
                }
                _ => {}
            }
        }
        
        std::thread::sleep(Duration::from_millis(16));
    }
}

fn generate_sphere_point_cloud(num_points: usize, radius: f32) -> Vec<Point3<f32>> {
    let mut points = Vec::with_capacity(num_points);
    let golden_angle = PI * (3.0 - 5.0f32.sqrt());
    
    for i in 0..num_points {
        let y = 1.0 - (i as f32 / (num_points as f32 - 1.0)) * 2.0;
        let radius_at_y = (1.0 - y * y).sqrt();
        let theta = golden_angle * i as f32;
        let x = theta.cos() * radius_at_y;
        let z = theta.sin() * radius_at_y;
        points.push(Point3::new(x * radius, y * radius, z * radius));
    }
    
    points
}