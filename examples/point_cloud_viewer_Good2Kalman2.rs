use ar_drivers::{any_glasses, GlassesEvent};
use nalgebra::{Unit, UnitQuaternion, Vector3, Matrix6, Vector6, Matrix3x6, Matrix6x3, Matrix3};
use std::io::{BufWriter, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

struct KalmanFilter {
    x: Vector6<f32>, 
    P: Matrix6<f32>, 
    F: Matrix6<f32>, 
    B: Matrix6x3<f32>, 
    Q: Matrix6<f32>, 
    H: Matrix3x6<f32>, 
    R: Matrix3<f32>,
}

impl KalmanFilter {
    pub fn new(dt: f32) -> Self {
        let i3 = Matrix3::<f32>::identity();
        let mut F = Matrix6::<f32>::zeros();
        F.fixed_view_mut::<3,3>(0,0).copy_from(&i3);
        F.fixed_view_mut::<3,3>(0,3).copy_from(&(i3 * dt));
        F.fixed_view_mut::<3,3>(3,3).copy_from(&i3);

        let mut B = Matrix6x3::<f32>::zeros();
        B.fixed_view_mut::<3,3>(0,0).copy_from(&(i3 * 0.5 * dt * dt));
        B.fixed_view_mut::<3,3>(3,0).copy_from(&(i3 * dt));

        let mut H = Matrix3x6::<f32>::zeros();
        H.fixed_view_mut::<3,3>(0,3).copy_from(&i3);

        KalmanFilter {
            x: Vector6::<f32>::zeros(),
            P: Matrix6::<f32>::identity() * 0.04,
            F,
            B,
            Q: Matrix6::<f32>::identity() * 0.01,
            H,
            R: Matrix3::<f32>::identity() * 0.1,
        }
    }

    pub fn predict(&mut self, accel: &Vector3<f32>) {
        self.x = self.F * self.x + self.B * accel;
        self.P = self.F * self.P * self.F.transpose() + self.Q;
    }
    pub fn update_zupt(&mut self) {
        let z = Vector3::<f32>::zeros();
        let y = z - self.H * self.x;
        let S = self.H * self.P * self.H.transpose() + self.R;
        let k = self.P * self.H.transpose() * S.try_inverse().unwrap();
        self.x += k * y;
        self.P = (Matrix6::<f32>::identity() - k * self.H) * self.P;
    }
    pub fn get_position(&self) -> Vector3<f32> {
        self.x.fixed_rows::<3>(0).into()
    }
    pub fn get_velocity(&self) -> Vector3<f32> {
        self.x.fixed_rows::<3>(3).into()
    }
}

#[derive(Debug, Default, Copy, Clone)]
struct PositionState {
    orientation: UnitQuaternion<f32>,
    velocity: Vector3<f32>,
    position: Vector3<f32>,
    last_time: Option<Instant>,

    raw_position: Vector3<f32>, // Külön állítsuk meg a raw pozíciót is (integrált gyorsulás nélkül)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut glasses = any_glasses()?; 
    println!("Connected to glasses: {}", glasses.name());

    // TCP szerverek: az első (12345) a Kalman-pozíciót, a második (12346) a raw pozíciót küldi.
    let listener_kalman = TcpListener::bind("0.0.0.0:12345")?;
    let listener_raw = TcpListener::bind("0.0.0.0:12346")?;

    println!("TCP servers running on ports 12345 (Kalman) and 12346 (Raw)");

    let pos_state = Arc::new(Mutex::new(PositionState::default()));

    let mut calibrated = false;
    let mut calibration_samples = Vec::new();
    let mut gyro_bias = Vector3::zeros();
    let mut accel_bias = Vector3::zeros();
    const CALIBRATION_SAMPLES: usize = 2000;

    // Külső integráción kívüli raw pozíció becslése (sebesség és pozíció integráció, komplementer nélkül)
    let mut raw_velocity = Vector3::zeros();
    let mut raw_position = Vector3::zeros();

    let pos_state_sensor = Arc::clone(&pos_state);
    thread::spawn(move || {
        let mut last_update = Instant::now();
        let alpha = 0.98;
        let beta = 1.0 - alpha;
        let mut kf = KalmanFilter::new(0.016);

        loop {
            if let Ok(event) = glasses.read_event() {
                if let GlassesEvent::AccGyro { accelerometer, gyroscope, .. } = event {
                    let delta_time = last_update.elapsed().as_secs_f32().max(0.001);
                    last_update = Instant::now();

                    if !calibrated {
                        calibration_samples.push((accelerometer, gyroscope));
                        if calibration_samples.len() >= CALIBRATION_SAMPLES {
                            let (acc_sum, gyro_sum) = calibration_samples.iter().fold(
                                (Vector3::zeros(), Vector3::zeros()),
                                |(acc, gyro), (a, g)| (acc + a, gyro + g),
                            );
                            accel_bias = acc_sum / CALIBRATION_SAMPLES as f32;
                            gyro_bias = gyro_sum / CALIBRATION_SAMPLES as f32;
                            accel_bias.z -= 9.81;
                            calibrated = true;
                            println!("Calibration complete. Biases - Accel: {:?}, Gyro: {:?}", accel_bias, gyro_bias);
                        }
                        continue;
                    }

                    // Deadzone szűrés
                    let mut corrected_acc = accelerometer - accel_bias;
                    let mut corrected_gyro = gyroscope - gyro_bias;
                    for i in 0..3 {
                        if corrected_acc[i].abs() < 0.03 { corrected_acc[i] = 0.0; }
                        if corrected_gyro[i].abs() < 0.003 { corrected_gyro[i] = 0.0; }
                    }

                    // Integrált raw pozíció (sebesség és pozíció egyszerű numerikus integrálása, komplementer nélkül)
                    // world_acc becsléshez - itt nem levonjuk a gravitációt, emlékezz rá, a raw még nem korrigált (kompenzált!)
                    let mut state = pos_state_sensor.lock().unwrap();

                    // Update raw velocity and position without Kalman filtering
                    let linear_acc = corrected_acc; // most raw quick integráció
                    raw_velocity += linear_acc * delta_time;
                    raw_position += raw_velocity * delta_time + 0.5 * linear_acc * delta_time * delta_time;

                    state.raw_position = raw_position;

                    // A többi változtatás változatlanul az orientációhoz, komplementer és Kalman
                    let world_acc = state.orientation * corrected_acc;
                    let gravity = Vector3::new(0.0, 9.81, 0.0);
                    let mut linear_acc_filtered = world_acc - gravity;
                    for i in 0..3 {
                        if linear_acc_filtered[i].abs() < 0.03 { linear_acc_filtered[i] = 0.0; }
                    }

                    // Orientáció frissítés - komplementer szűrő
                    let gyro_vec = corrected_gyro * delta_time;
                    let gyro_angle = gyro_vec.norm();

                    if gyro_angle > 0.001 {
                        let gyro_axis = Unit::new_normalize(gyro_vec);
                        let delta_q = UnitQuaternion::from_axis_angle(&gyro_axis, gyro_angle);
                        state.orientation = state.orientation * delta_q;
                    }

                    if corrected_acc.norm() > 9.5 && corrected_acc.norm() < 10.5 {
                        let normalized_acc = corrected_acc.normalize();
                        let quat = state.orientation;
                        let current_up = quat * Vector3::y();
                        let desired_up = -normalized_acc;
                        if let Some(correction) = UnitQuaternion::rotation_between(&current_up, &desired_up) {
                            state.orientation = state.orientation.slerp(&(correction * state.orientation), beta);
                        }
                    }

                    // Kalman-predikció a szűrt gyorsulással!
                    kf.F.fixed_view_mut::<3, 3>(0, 3).copy_from(&(&Matrix3::<f32>::identity() * delta_time));
                    kf.B.fixed_view_mut::<3, 3>(0, 0).copy_from(&(&Matrix3::<f32>::identity() * 0.5 * delta_time * delta_time));
                    kf.B.fixed_view_mut::<3, 3>(3, 0).copy_from(&(&Matrix3::<f32>::identity() * delta_time));
                    kf.predict(&linear_acc_filtered);

                    // ZUPT frissítés
                    if (corrected_acc.norm() - 9.81).abs() < 0.2 && corrected_gyro.norm() < 0.05 {
                        kf.update_zupt();
                    }
                    state.position = kf.get_position();
                    state.velocity = kf.get_velocity();
                }
            }
            thread::sleep(Duration::from_millis(5));
        }
    });

    // Kalman pozíció TCP server
    let pos_state_kalman = Arc::clone(&pos_state);
    thread::spawn(move || {
        for stream in listener_kalman.incoming() {
            if let Ok(stream) = stream {
                println!("Kalman client connected");
                let client_pos = Arc::clone(&pos_state_kalman);
                thread::spawn(move || {
                    handle_client_kalman(stream, client_pos);
                });
            }
        }
    });

    // Raw pozíció TCP server
    let pos_state_raw = Arc::clone(&pos_state);
    thread::spawn(move || {
        for stream in listener_raw.incoming() {
            if let Ok(stream) = stream {
                println!("Raw client connected");
                let client_pos = Arc::clone(&pos_state_raw);
                thread::spawn(move || {
                    handle_client_raw(stream, client_pos);
                });
            }
        }
    });

    // Végtelen ciklus, hogy a fő szál ne lépjen ki
    loop {
        thread::sleep(Duration::from_secs(60));
    }
}

fn handle_client_kalman(mut stream: TcpStream, pos_state: Arc<Mutex<PositionState>>) {
    let mut writer = BufWriter::new(&mut stream);
    loop {
        let state = pos_state.lock().unwrap();
        let data = format!(
            r#"{{"x": {:.6}, "y": {:.6}, "z": {:.6}}}"#,
            state.position.x, state.position.y, state.position.z
        );
        if writeln!(writer, "{}", data).is_err() {
            break;
        }
        if writer.flush().is_err() {
            break;
        }
        drop(state);
        thread::sleep(Duration::from_millis(16)); // ~60Hz
    }
}

fn handle_client_raw(mut stream: TcpStream, pos_state: Arc<Mutex<PositionState>>) {
    let mut writer = BufWriter::new(&mut stream);
    loop {
        let state = pos_state.lock().unwrap();
        let data = format!(
            r#"{{"x": {:.6}, "y": {:.6}, "z": {:.6}}}"#,
            state.raw_position.x, state.raw_position.y, state.raw_position.z
        );
        if writeln!(writer, "{}", data).is_err() {
            break;
        }
        if writer.flush().is_err() {
            break;
        }
        drop(state);
        thread::sleep(Duration::from_millis(16));
    }
}
