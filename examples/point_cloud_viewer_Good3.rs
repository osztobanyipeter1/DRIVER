use ar_drivers::{any_glasses, GlassesEvent};
use nalgebra::{Vector3};
use std::f32::consts::PI;
use std::time::{Duration, Instant};
use std::net::{TcpListener, TcpStream};
use std::io::{Write, BufWriter};
use std::sync::{Arc, Mutex};
use std::thread;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut glasses = any_glasses()?; //ezzel csatlakozik a szemüveghez
    println!("Connected to glasses: {}", glasses.name());

    let listener = TcpListener::bind("0.0.0.0:12345")?; //indítja a TCP szervert
    println!("TCP server running!");

    // Euler-szögek tárolása (pitch, roll, yaw) radiánban
    let euler_angles = Arc::new(Mutex::new((0.0f32, 0.0f32, 0.0f32))); // (pitch, roll, yaw)
    let mut calibrated = false; //jelzi, hogy megtörtént-e a kalibrálás a futás elején
    let mut calibration_samples = Vec::new(); //kalibrációs minták gyűjtése a kalibrációhoz
    let mut gyro_bias = Vector3::zeros(); //szenzorok bias tárolása
    let mut accel_bias = Vector3::zeros();
    const CALIBRATION_SAMPLES: usize = 2000; //kalibrációhoz gyűjtött minták száma

    let sensor_angles = Arc::clone(&euler_angles); //a megosztott változókat másolja a szálba az euler_angles
    thread::spawn(move || { //új szál indítása, a "move" belemozgatja a szükséges sensor_angles változókat
        let mut last_update = Instant::now();
        let alpha = 0.98; // Gyroscope súlyozása
        let beta = 1.0 - alpha; // Accelerometer súlyozása

        loop {
            if let Ok(event) = glasses.read_event() {
                match event {
                    GlassesEvent::AccGyro { accelerometer, gyroscope, timestamp: _ } => {
                        let delta_time = last_update.elapsed().as_secs_f32();
                        last_update = Instant::now();

                        if !calibrated {
                            calibration_samples.push((accelerometer, gyroscope));
                            
                            if calibration_samples.len() >= CALIBRATION_SAMPLES {

                                let (acc_sum, gyro_sum) = calibration_samples.iter().fold(
                                    (Vector3::zeros(), Vector3::zeros()),
                                    |(acc, gyro), (a, g)| (acc + a, gyro + g)
                                );
                                
                                accel_bias = acc_sum / CALIBRATION_SAMPLES as f32;
                                gyro_bias = gyro_sum / CALIBRATION_SAMPLES as f32;

                                accel_bias.z -= 9.81; // Gravitáció eltávolítása
                                
                                calibrated = true;
                                println!("Calibration complete. Biases - Accel: {:?}, Gyro: {:?}", accel_bias, gyro_bias);
                            }
                            continue;
                        }

                        let corrected_acc = accelerometer - accel_bias;
                        let corrected_gyro = gyroscope - gyro_bias;

                        // Gyroscope alapú szögfrissítés (integrálás)
                        let delta_pitch = corrected_gyro.x * delta_time;   // x -> pitch (fel-le)
                        let delta_yaw = corrected_gyro.y * delta_time;    // y -> roll (oldalra dőlés)
                        let delta_roll = corrected_gyro.z * delta_time;     // z -> yaw (forgás balra-jobbra)
                        //let delta_pitch = corrected_gyro.x * delta_time; //fel le lenne x
                        //let delta_yaw = corrected_gyro.y * delta_time; //jobbra balra lenne y
                        //let delta_roll = corrected_gyro.z * delta_time; //billentés lenne z

                        let mut angles = sensor_angles.lock().unwrap();
                        angles.0 += delta_pitch; // Pitch (fel-le)
                        angles.1 += delta_roll;  // Roll (oldalra dőlés)
                        angles.2 += delta_yaw;   // Yaw (forgás balra-jobbra)

                        if corrected_acc.norm() > 9.5 && corrected_acc.norm() < 10.5 {
                            // Javított pitch és roll számítás
                            let acc_pitch = (-corrected_acc.x).atan2(corrected_acc.z); // x -> pitch
                            let acc_roll = corrected_acc.y.atan2(corrected_acc.z);     // y -> roll
                            
                            // Lágy korrekció
                            angles.0 = angles.0 * 0.95 + acc_pitch * 0.05;
                            angles.1 = angles.1 * 0.95 + acc_roll * 0.05;
                            
                        }

                        println!(
                            "Orientation - Pitch (fel le): {:.2}°, Roll (oldalra dőlés):{:.2}°, Yaw (Jobbra_Balra): {:.2}°",
                            angles.0.to_degrees(),
                            angles.1.to_degrees(),
                            angles.2.to_degrees()
                        );
                    }
                    _ => {}
                }
            }
            thread::sleep(Duration::from_millis(16));
        }
    });

    for stream in listener.incoming() {
        match stream {
            Ok(stream) => {
                println!("New client connected");
                let client_angles = Arc::clone(&euler_angles);
                thread::spawn(move || handle_client(stream, client_angles));
            }
            Err(e) => eprintln!("Connection failed: {}", e),
        }
    }

    Ok(())
}

fn handle_client(mut stream: TcpStream, angles: Arc<Mutex<(f32, f32, f32)>>) {
    let mut writer = BufWriter::new(&mut stream);
    
    loop {
        let angles = angles.lock().unwrap();
        let data = format!(
            r#"{{"pitch": {}, "roll": {}, "yaw": {}}}"#,
            angles.0.to_degrees(),
            angles.1.to_degrees(),
            angles.2.to_degrees()
        );
        
        if let Err(e) = writeln!(writer, "{}", data) {
            eprintln!("Error writing to socket: {}", e);
            break;
        }
        
        if let Err(e) = writer.flush() {
            eprintln!("Error flushing socket: {}", e);
            break;
        }
        
        thread::sleep(Duration::from_millis(16));
    }
}