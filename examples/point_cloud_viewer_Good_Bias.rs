use ar_drivers::{any_glasses, DisplayMode, GlassesEvent};
use nalgebra::{Point3, Unit, UnitQuaternion, Vector3};
use std::f32::consts::PI;
use std::time::{Duration, Instant};
use std::net::{TcpListener, TcpStream};
use std::io::{Write, BufWriter};
use std::sync::{Arc, Mutex};
use std::thread;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    
    let mut glasses = any_glasses()?; //ezzel csatlakozik a szemüveghez
    println!("Connected to glasses: {}", glasses.name());

    //glasses.set_display_mode(DisplayMode::Stereo)?; //Ez nem kell, ez csak ráküld semmit a szemüvegre! De majd a sztereo módot itt lehet beállítani.

    let listener = TcpListener::bind("0.0.0.0:12345")?; //itt indítja a TCP szervert
    println!("TCP server running!");

    let orientation = Arc::new(Mutex::new(UnitQuaternion::identity()));

    let gyro_bias = Vector3::new(-0.0031045338, 0.0015237421, -0.004987);
    let accel_bias = Vector3::new(0.32512817, 9.777826, -8.99522);
    let calibrated = true; // Azonnal beállítjuk kalibráltnak

    let sensor_orientation = Arc::clone(&orientation); //a megosztott változókat másolja a szálba az orientation
    thread::spawn(move || { //új szál indítása, a "move" belemozgatja a szükséges sensor_orientation változókat
        let mut last_update = Instant::now(); //tárolja az utolsó frissítés idejét
        let alpha = 0.98; //a komplementer szűrés súlyozási tényezője (98% giroszkóp)
        let beta = 1.0 - alpha; //a gyorsulásmérő súlyozása (jelenleg 2%)

        loop { //végtelen ciklusban történik
            if let Ok(event) = glasses.read_event() { 
                match event {
                    GlassesEvent::AccGyro { accelerometer, gyroscope, timestamp: _ } => { //a gyorsulásmérő és a giroszkóp adatokat dolgozza fel
                        let delta_time = last_update.elapsed().as_secs_f32(); //számolja az előző frissítés óta eltelt időt
                        last_update = Instant::now(); //frissíti az utolsó frissítés idejét                        

                        let corrected_acc = accelerometer - accel_bias; //kivonja kalibrációs (bias) értékeket a szenzor értékekből
                        let corrected_gyro = gyroscope - gyro_bias;

                        let mut ori = sensor_orientation.lock().unwrap(); //szinkronizálja a megosztott tájolási adatokat

                        let gyro_vec = corrected_gyro * delta_time; //szögfordulási vektor radiánban
                        let gyro_angle = gyro_vec.norm(); //forgás nagysága radiánban
                        
                        if gyro_angle > 0.0 {
                            let gyro_axis = Unit::new_normalize(gyro_vec);
                            let delta_q = UnitQuaternion::from_axis_angle(&gyro_axis, gyro_angle); //forgást reprezentáló kvaternió
                            *ori = *ori * delta_q; //új tájolás számolása
                        }

                        if corrected_acc.norm() > 8.0 && corrected_acc.norm() < 11.0 { //csak akkor fut le, ha a gyorsulásmérő értéke az adott számok között van
                            let normalized_acc = corrected_acc.normalize(); //normalizált gyorsulási vektor
                            let quat = *ori;
                            let current_up = quat * Vector3::z(); //jelenlegi fel irány a quaternionból
                            let desired_up = normalized_acc; //gyorsulásmérőből mért fel irány
                            
                            if let Some(correction) = UnitQuaternion::rotation_between(&current_up, &desired_up) { //számolja a fel irányok közötti forgást
                                *ori = ori.slerp(&(correction * *ori), beta); //kismértékben korrigál a gyorsulásmérő alapján
                            }
                        }

                        let quat = *ori;
                        let (roll, pitch, yaw) = quat.euler_angles(); //euler szögek kinyerése a kvaternióból
                        println!("Orientation - Oldalra_Billentés: {:.2}°, Fel_Le: {:.2}°, Balra_Jobbra: {:.2}°", roll.to_degrees(), pitch.to_degrees(), yaw.to_degrees());
                    }
                    _ => {}
                }
            }
            thread::sleep(Duration::from_millis(16)); //16 ms várakozás után olvassa újra az adatokat (ez 60fps)
        }
    });

    for stream in listener.incoming() { //végtelen iterátor, ami új TCP kapcsolatokat vár
        match stream { //feldolgozza a kapcsolódási kísérletet
            Ok(stream) => {
                println!("New client connected");
                let client_orientation = Arc::clone(&orientation); //ha sikeres a kapcsolat, akkor létrehoz egy másolatot az orientation megosztott adatról
                thread::spawn(move || handle_client(stream, client_orientation)); //új szálat készít az ügyfél kezelésre, a move segít, hogy a szálra kerüljenek a megosztott változók
            }
            Err(e) => eprintln!("Connection failed: {}", e),
        }
    }

    Ok(())
}

fn handle_client(mut stream: TcpStream, orientation: Arc<Mutex<UnitQuaternion<f32>>>) { //orientation van megosztva
    let mut writer = BufWriter::new(&mut stream); //írást biztosít a hálózati kommunikációhoz (BufWriter)
    
    loop {
        let ori = orientation.lock().unwrap(); //szinkronizált hozzáférés a megosztott adathoz
        let quat = *ori;
        let (roll, pitch, yaw) = quat.euler_angles(); //Kvaternióból Euler szögek
        
        let data = format!(
            r#"{{"yaw": {}, "pitch": {}, "roll": {}}}"#,
            yaw.to_degrees(), //radián fokba konvertálása
            pitch.to_degrees(),
            roll.to_degrees()
        );
        
        //hibakezelés
        if let Err(e) = writeln!(writer, "{}", data) {
            eprintln!("Error writing to socket: {}", e);
            break;
        }
        
        if let Err(e) = writer.flush() { //flush biztosítja, hogy az adat biztosan továbbmenjen, automatikusan kiürít a pufferből
            eprintln!("Error flushing socket: {}", e);
            break;
        }
        
        drop(ori); //feloldás
        thread::sleep(Duration::from_millis(16)); //frissítési gyakoriság
    }
}

