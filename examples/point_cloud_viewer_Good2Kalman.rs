use ar_drivers::{any_glasses, GlassesEvent};
use nalgebra::{Unit, UnitQuaternion, Vector3, Matrix6, Vector6, Matrix3x6, Matrix6x3, Matrix3};
use std::io::{BufWriter, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

struct KalmanFilter {
    x: Vector6<f32>, //állapotvektor (pozíció, sebesség)
    P: Matrix6<f32>, //állapotbecslés hibamátrixa
    F: Matrix6<f32>, //állapotmeneti mátrix
    B: Matrix6x3<f32>, //vezérlési mátrix
    Q: Matrix6<f32>, //folyamati zaj kovariancia mátrix
    H: Matrix3x6<f32>, // megfigyelési mátrix (a szenzor milyen állapotrészt lát)
    R: Matrix3<f32>, //mérési zaj kovariancia mátrix
}

impl KalmanFilter {
    pub fn new(dt: f32) -> Self {
        let i3 = nalgebra::Matrix3::<f32>::identity();
        let mut F = Matrix6::<f32>::zeros();
        F.fixed_slice_mut::<3,3>(0,0).copy_from(&i3);
        F.fixed_slice_mut::<3,3>(0,3).copy_from(&(i3 * dt));
        F.fixed_slice_mut::<3,3>(3,3).copy_from(&i3);

        let mut B = Matrix6x3::<f32>::zeros();
        B.fixed_slice_mut::<3,3>(0,0).copy_from(&(i3 * 0.5 * dt * dt));
        B.fixed_slice_mut::<3,3>(3,0).copy_from(&(i3 * dt));

        let mut H = Matrix3x6::<f32>::zeros();
        H.fixed_slice_mut::<3,3>(0,3).copy_from(&i3);

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
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut glasses = any_glasses()?; //ezzel csatlakozik a szemüveghez
    println!("Connected to glasses: {}", glasses.name());

    //glasses.set_display_mode(DisplayMode::Stereo)?; //Ez nem kell, ez csak ráküld semmit a szemüvegre! De majd a sztereo módot itt lehet beállítani.

    let listener = TcpListener::bind("0.0.0.0:12345")?; //itt indítja a TCP szervert
    println!("TCP server running!");

    let pos_state = Arc::new(Mutex::new(PositionState::default()));

    let mut calibrated = false; //jelzi, hogy megtörtént-e a kalibrálás a futás elején
    let mut calibration_samples = Vec::new(); //kalibrációs minták gyűjtése a kalibrációhoz
    let mut gyro_bias = Vector3::zeros(); //szenzorok bias tárolása
    let mut accel_bias = Vector3::zeros();
    const CALIBRATION_SAMPLES: usize = 2000; //1000 darab mintát gyűjt a kalibrációhoz

    //THRESHOLDOK
    const ACC_DEADZONE: f32 = 0.03;     // m/s2 alatt nullának tekintett gyorsulás
    const GYRO_DEADZONE: f32 = 0.003;   // rad/s alatt nullának tekintett szögsebesség
    const ORI_ANGLE_DEADZONE: f32 = 0.001; // rad (≈0.6°) alatt figyelmen kívül hagyott elfordulás

    let pos_state_sensor = Arc::clone(&pos_state); //a megosztott változókat másolja a szálba a pos_state
    thread::spawn(move || { //új szál indítása, a "move" belemozgatja a szükséges sensor_orientation változókat
        let mut last_update = Instant::now(); //tárolja az utolsó frissítés idejét
        let alpha = 0.98; //a komplementer szűrés súlyozási tényezője (98% giroszkóp)
        let beta = 1.0 - alpha;  //a gyorsulásmérő súlyozása (jelenleg 2%)
        let mut kf = KalmanFilter::new(0.016);

        loop {//végtelen ciklusban történik
            if let Ok(event) = glasses.read_event() {
                match event {
                    GlassesEvent::AccGyro { accelerometer, gyroscope, .. } => { //a gyorsulásmérő és a giroszkóp adatokat dolgozza fel
                        let delta_time = last_update.elapsed().as_secs_f32().max(0.001); //számolja az előző frissítés óta eltelt időt
                        last_update = Instant::now(); //frissíti az utolsó frissítés idejét

                        if !calibrated {
                            calibration_samples.push((accelerometer, gyroscope));

                            if calibration_samples.len() >= CALIBRATION_SAMPLES {//az első 2000 példát gyűjti a kalibrációhoz

                                let (acc_sum, gyro_sum) = calibration_samples.iter().fold(
                                    (Vector3::zeros(), Vector3::zeros()),
                                    |(acc, gyro), (a, g)| (acc + a, gyro + g)
                                );
                                accel_bias = acc_sum / CALIBRATION_SAMPLES as f32; //átlagos gyorsulásmérőt és giroszkóp értékeket számol
                                gyro_bias = gyro_sum / CALIBRATION_SAMPLES as f32;

                                accel_bias.z -= 9.81; //kivonja a gravitációs gyorsulást

                                calibrated = true;
                                println!(
                                    "Calibration complete. Biases - Accel: {:?}, Gyro: {:?}", accel_bias, gyro_bias);//kalibrációs eredményeket kiírja
                            }
                            continue; //a kalibráció után kihagyja a ciklus többi részét
                        }

                        // Deadzone szűrés (gyorsulás + gyro)
                        let mut corrected_acc = accelerometer - accel_bias; //kivonja kalibrációs (bias) értékeket a szenzor értékekből
                        let mut corrected_gyro = gyroscope - gyro_bias;

                        for i in 0..3 {
                            if corrected_acc[i].abs() < ACC_DEADZONE { corrected_acc[i] = 0.0; }
                            if corrected_gyro[i].abs() < GYRO_DEADZONE { corrected_gyro[i] = 0.0; }
                        }

                        // Deadzone: linear_acc is, ha a gyorsulás szinte nulla, ne legyen véletlen vándorlás!
                        let mut state = pos_state_sensor.lock().unwrap();
                        let world_acc = state.orientation * corrected_acc;
                        let gravity = Vector3::new(0.0, 9.81, 0.0);
                        let mut linear_acc = world_acc - gravity;
                        for i in 0..3 {
                            if linear_acc[i].abs() < ACC_DEADZONE { linear_acc[i] = 0.0; }
                        }

                        // Orientáció frissítés – csak ha a delta szög nagyobb deadzone-nál
                        let gyro_vec = corrected_gyro * delta_time; //szögfordulási vektor radiánban
                        let gyro_angle = gyro_vec.norm(); //forgás nagysága radiánban

                        if gyro_angle > ORI_ANGLE_DEADZONE {
                            let gyro_axis = Unit::new_normalize(gyro_vec);
                            let delta_q = UnitQuaternion::from_axis_angle(&gyro_axis, gyro_angle);
                            state.orientation = state.orientation * delta_q;
                        }

                        // Komplementer korrekció (változatlan)
                        if corrected_acc.norm() > 9.5 && corrected_acc.norm() < 10.5 {
                            let normalized_acc = corrected_acc.normalize();
                            let quat = state.orientation;
                            let current_up = quat * Vector3::y();
                            let desired_up = -normalized_acc;
                            if let Some(correction) = UnitQuaternion::rotation_between(&current_up, &desired_up) {
                                state.orientation = state.orientation.slerp(
                                    &(correction * state.orientation),
                                    beta,
                                );
                            }
                        }

                        // Kalman-predikció a szűrt gyorsulással!
                        kf.F.fixed_view_mut::<3,3>(0,3).copy_from(&(&nalgebra::Matrix3::<f32>::identity() * delta_time));
                        kf.B.fixed_view_mut::<3,3>(0,0).copy_from(&(&nalgebra::Matrix3::<f32>::identity() * 0.5 * delta_time * delta_time));
                        kf.B.fixed_view_mut::<3,3>(3,0).copy_from(&(&nalgebra::Matrix3::<f32>::identity() * delta_time));
                        kf.predict(&linear_acc);

                        // Ha (abs(acc - 9.81) < 0.2) és giroszkóp norm() < 0.05, akkor frissít (ZUPT)
                        if (corrected_acc.norm() - 9.81).abs() < 0.2 && corrected_gyro.norm() < 0.05 {
                            kf.update_zupt();
                        }
                        state.position = kf.get_position();
                        state.velocity = kf.get_velocity();

                        // Debug print (opcionális)
                        let quat = state.orientation;
                        let (pitch, roll, yaw) = quat.euler_angles();
                        
                        println!(
                            "Kálmán pozíció: x (előre(+)-hátra(-))={:.3}, y (felfele(+)-lefele(-)) ={:.3}, z(jobbra(+)-balra(-))={:.3} m | sebesség x={:.2}, y={:.2}, z={:.2} m/s || Orientáció: roll={:.2}°, pitch={:.2}°, yaw={:.2}°",
                            state.position.x/2.0,
                            state.position.y/2.0,
                            state.position.z/2.0,
                            state.velocity.x,
                            state.velocity.y,
                            state.velocity.z,
                            roll.to_degrees(),
                            pitch.to_degrees(),
                            yaw.to_degrees(),
                        );
                    }
                    _ => {}
                }
            }
            thread::sleep(Duration::from_millis(5)); //5 ms várakozás után olvassa újra az adatokat
        }
    });

    for stream in listener.incoming() { //végtelen iterátor, ami új TCP kapcsolatokat vár
        match stream { //feldolgozza a kapcsolódási kísérletet
            Ok(stream) => {
                println!("New client connected");
                let client_pos = Arc::clone(&pos_state); //ha sikeres a kapcsolat, akkor létrehoz egy másolatot az post_state megosztott adatról
                thread::spawn(move || handle_client(stream, client_pos)); //új szálat készít az ügyfél kezelésre, a move segít, hogy a szálra kerüljenek a megosztott változók
            }
            Err(e) => eprintln!("Connection failed: {}", e),
        }
    }
    Ok(())
}

fn handle_client(mut stream: TcpStream, pos_state: Arc<Mutex<PositionState>>) {
    let mut writer = BufWriter::new(&mut stream); //írást biztosít a hálózati kommunikációhoz (BufWriter)

    loop {
        let state = pos_state.lock().unwrap();
        let data = format!(
            r#"{{"x": {:.3}, "y": {:.3}, "z": {:.3}}}"#,
            state.position.x/2.0, state.position.y/2.0, state.position.z/2.0
        );

        if writeln!(writer, "{}", data).is_err() {
            break;
        }

        if writer.flush().is_err() {
            break;
        }

        drop(state);
        std::thread::sleep(std::time::Duration::from_millis(16));
    }
}
