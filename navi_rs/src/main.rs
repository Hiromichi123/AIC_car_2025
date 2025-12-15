use rclrs::{log_info, CreateBasicExecutor, SpinOptions};
use std::{thread::sleep, time::Duration};

use crate::navi::{CoordUnit, Pos, ServoState};

pub mod navi;

macro_rules! spin_tick {
    ($exec:expr) => {{
        let mut spin_options = SpinOptions::default();
        spin_options.timeout = Some(Duration::from_millis(100));
        spin_options.only_next_available_work = true;
        spin_options.until_promise_resolved = None;
        $exec.spin(spin_options);
    }};
}

macro_rules! set_and_wait {
    ($node:expr, $wps:expr, $trans_th:expr, $rot_th:expr, $exec:expr, $log:literal) => {{
        $node.set_destinations($wps, $trans_th, $rot_th)?;
        let start_time = std::time::Instant::now();
        let timeout = std::time::Duration::from_secs(5);
        
        loop {
            spin_tick!($exec);

            if $node.is_arrived() {
                log_info!("navi_main", $log);
                break;
            }
            
            if start_time.elapsed() > timeout {
                log_info!("navi_main", "⚠️导航超时 5s，继续动作...");
                break;
            }
        }
    }};
}

fn main() -> anyhow::Result<()> {
    let context = rclrs::Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    log_info!("navi_main", "starting navigator...");
    let navi_node = navi::NaviSubNode::new(&executor, "navigator", "lidar_data")?;

    // 先把舵机复位到中位，避免上电姿态不一致
    if let Err(err) =
        navi_node.call_servo_blocking(&mut executor, ServoState::CENTER, Duration::from_secs(1))
    {
        log_info!("navi_main", "SERVO center command failed: {:?}", err);
    }

    let waypoints = vec![Pos {
        translation: CoordUnit(1.1, 0.0, 0.0),
        rotation: CoordUnit(0.0, 0.0, 0.0),
    }];

    set_and_wait!(
        navi_node,
        waypoints,
        0.10,
        0.3,
        executor,
        "First Camera point reached! Stopping navigation..."
    );

    // 循环检测交通灯，直到检测到绿灯
    log_info!("navi_main", "Waiting for green light...");
    match navi_node.call_yolo_blocking(
        &mut executor,
        Some("traffic_light"),
        Some("camera1"),
        Duration::from_secs_f32(2.0),
    ) {
        Ok(yolo_response) => {
            log_info!(
                "navi_main",
                "Traffic light detection: {:?}",
                yolo_response.message
            );
            // 检查响应中是否包含"绿灯"
            if yolo_response.message.contains("绿灯") {
                log_info!("navi_main", "Green light detected! Proceeding...");
            } else {
                log_info!("navi_main", "No green light yet, retrying...");
                sleep(Duration::from_millis(500));
            }
        }
        Err(err) => {
            log_info!("navi_main", "YOLO service call failed, retrying: {:?}", err);
            sleep(Duration::from_millis(500));
        }
    }

    let waypoints = vec![Pos {
        translation: CoordUnit(2.6, 0.0, 0.0),
        rotation: CoordUnit(0.0, 0.0, 0.0),
    }];

    set_and_wait!(
        navi_node,
        waypoints,
        0.10,
        0.30,
        executor,
        "First person reached! Stopping navigation..."
    );

    // 在完成第一段行程后将舵机拨到左侧尝试观察
    if let Err(err) =
        navi_node.call_servo_blocking(&mut executor, ServoState::LEFT, Duration::from_secs(1))
    {
        log_info!("navi_main", "SERVO left command failed: {:?}", err);
    }

    match navi_node.call_yolo_blocking(
        &mut executor,
        Some("people"),
        Some("camera1"),
        Duration::from_secs_f32(2.0),
    ) {
        Ok(yolo_response) => {
            println!("yolo: {:?}", yolo_response.message);
        }
        Err(err) => {
            log_info!(
                "navi_main",
                "YOLO service unavailable or timed out, continuing without detection: {:?}",
                err
            );
        }
    }

    let waypoints = vec![
        Pos {
            translation: CoordUnit(3.35, 0.0, 0.0),
            rotation: CoordUnit(0.0, 0.0, 0.0),
        },
        Pos {
            translation: CoordUnit(3.35, 0.6, 0.0),
            rotation: CoordUnit(0.0, 0.0, 0.0),
        },
    ];

    set_and_wait!(
        navi_node,
        waypoints,
        0.10,
        0.30,
        executor,
        "Second person reached! Stopping navigation..."
    );

    match navi_node.call_yolo_blocking(
        &mut executor,
        Some("people"),
        Some("camera2"),
        Duration::from_secs_f32(2.0),
    ) {
        Ok(yolo_response) => {
            println!("yolo: {:?}", yolo_response.message);
        }
        Err(err) => {
            log_info!(
                "navi_main",
                "YOLO service unavailable or timed out, continuing without detection: {:?}",
                err
            );
        }
    }

    let waypoints = vec![
        Pos {
            translation: CoordUnit(3.35, 1.2, 0.0),
            rotation: CoordUnit(0.0, 0.0, 0.0),
        },
        Pos {
            translation: CoordUnit(2.5, 1.2, 0.0),
            rotation: CoordUnit(0.0, 0.0, 0.0),
        },
    ];

    set_and_wait!(
        navi_node,
        waypoints,
        0.10,
        0.30,
        executor,
        "Third person reached! Stopping navigation..."
    );

    match navi_node.call_yolo_blocking(
        &mut executor,
        Some("people"),
        Some("camera1"),
        Duration::from_secs_f32(2.0),
    ) {
        Ok(yolo_response) => {
            println!("yolo: {:?}", yolo_response.message);
        }
        Err(err) => {
            log_info!(
                "navi_main",
                "YOLO service unavailable or timed out, continuing without detection: {:?}",
                err
            );
        }
    }

    if let Err(err) =
        navi_node.call_servo_blocking(&mut executor, ServoState::RIGHT, Duration::from_secs(1))
    {
        log_info!("navi_main", "SERVO left command failed: {:?}", err);
    }

    match navi_node.call_yolo_blocking(
        &mut executor,
        Some("people"),
        Some("camera1"),
        Duration::from_secs_f32(2.0),
    ) {
        Ok(yolo_response) => {
            println!("yolo: {:?}", yolo_response.message);
        }
        Err(err) => {
            log_info!(
                "navi_main",
                "YOLO service unavailable or timed out, continuing without detection: {:?}",
                err
            );
        }
    }

    match navi_node.call_ocr_blocking(&mut executor, Duration::from_secs_f32(2.0)) {
        Ok(ocr_response) => {
            println!("ocr: {:?}", ocr_response.message);
        }
        Err(err) => {
            log_info!(
                "navi_main",
                "OCR service unavailable or timed out, continuing without detection: {:?}",
                err
            );
        }
    }

    let waypoints = vec![
        Pos {
            translation: CoordUnit(1.8, 1.2, 0.0),
            rotation: CoordUnit(0.0, 0.0, 0.0),
        },
        Pos {
            translation: CoordUnit(1.8, 1.9, 0.0),
            rotation: CoordUnit(0.0, 0.0, 0.0),
        },
    ];

    set_and_wait!(
        navi_node,
        waypoints,
        0.10,
        0.30,
        executor,
        "All waypoints reached! Stopping navigation..."
    );

    if let Err(err) =
        navi_node.call_servo_blocking(&mut executor, ServoState::CENTER, Duration::from_secs(1))
    {
        log_info!("navi_main", "SERVO center command failed: {:?}", err);
    }

    match navi_node.call_yolo_blocking(
        &mut executor,
        Some("people"),
        Some("camera1"),
        Duration::from_secs_f32(2.0),
    ) {
        Ok(yolo_response) => {
            println!("yolo: {:?}", yolo_response.message);
        }
        Err(err) => {
            log_info!(
                "navi_main",
                "YOLO service unavailable or timed out, continuing without detection: {:?}",
                err
            );
        }
    }

    if let Err(err) =
        navi_node.call_servo_blocking(&mut executor, ServoState::LEFT, Duration::from_secs(1))
    {
        log_info!("navi_main", "SERVO left command failed: {:?}", err);
    }

    log_info!("navi_main", "Waiting for green light...");
    match navi_node.call_yolo_blocking(
        &mut executor,
        Some("traffic_light"),
        Some("camera1"),
        Duration::from_secs_f32(2.0),
    ) {
        Ok(yolo_response) => {
            log_info!(
                "navi_main",
                "Traffic light detection: {:?}",
                yolo_response.message
            );
            // 检查响应中是否包含"绿灯"
            if yolo_response.message.contains("绿灯") {
                log_info!("navi_main", "Green light detected! Proceeding...");
            } else {
                log_info!("navi_main", "No green light yet, retrying...");
            }
        }
        Err(err) => {
            log_info!("navi_main", "YOLO service call failed, retrying: {:?}", err);
        }
    }

    let waypoints = vec![Pos {
        translation: CoordUnit(1.8, 3.4, 0.0),
        rotation: CoordUnit(0.0, 0.0, 0.0),
    }];

    set_and_wait!(
        navi_node,
        waypoints,
        0.2,
        0.3,
        executor,
        "All waypoints reached! Stopping navigation..."
    );

    if let Err(err) =
        navi_node.call_servo_blocking(&mut executor, ServoState::CENTER, Duration::from_secs(1))
    {
        log_info!("navi_main", "SERVO left command failed: {:?}", err);
    }

    match navi_node.call_ocr_blocking(&mut executor, Duration::from_secs_f32(2.0)) {
        Ok(ocr_response) => {
            println!("ocr: {:?}", ocr_response.message);
        }
        Err(err) => {
            log_info!(
                "navi_main",
                "OCR service unavailable or timed out, continuing without detection: {:?}",
                err
            );
        }
    }

    // 任务完成后回正舵机
    if let Err(err) =
        navi_node.call_servo_blocking(&mut executor, ServoState::CENTER, Duration::from_secs(1))
    {
        log_info!("navi_main", "SERVO center command failed: {:?}", err);
    }

    let waypoints = vec![
        Pos {
            translation: CoordUnit(0.6, 3.4, 0.0),
            rotation: CoordUnit(0.0, 0.0, 0.0),
        },
        Pos {
            translation: CoordUnit(0.6, 2.5, 0.0),
            rotation: CoordUnit(0.0, 0.0, 0.0),
        },
    ];

    set_and_wait!(
        navi_node,
        waypoints,
        0.10,
        0.30,
        executor,
        "First car reached! Stopping navigation..."
    );

    log_info!("navi_main", "Sleeping for 1 secs.");
    sleep(std::time::Duration::from_secs_f32(1.0));

    match navi_node.call_yolo_blocking(
        &mut executor,
        Some("rubbish_bin"),
        Some("camera1"),
        Duration::from_secs_f32(2.0),
    ) {
        Ok(yolo_response) => {
            println!("yolo: {:?}", yolo_response.message);
        }
        Err(err) => {
            log_info!(
                "navi_main",
                "YOLO service unavailable or timed out, continuing without detection: {:?}",
                err
            );
        }
    }

    let waypoints = vec![Pos {
        translation: CoordUnit(0.6, 1.5, 0.0),
        rotation: CoordUnit(0.0, 0.0, 0.0),
    }];

    set_and_wait!(
        navi_node,
        waypoints,
        0.10,
        0.30,
        executor,
        "Forth building reached! Stopping navigation..."
    );

    match navi_node.call_yolo_blocking(
        &mut executor,
        Some("e_bike"),
        Some("camera2"),
        Duration::from_secs_f32(2.0),
    ) {
        Ok(yolo_response) => {
            println!("yolo: {:?}", yolo_response.message);
        }
        Err(err) => {
            log_info!(
                "navi_main",
                "YOLO service unavailable or timed out, continuing without detection: {:?}",
                err
            );
        }
    }

    let waypoints = vec![Pos {
        translation: CoordUnit(0.6, 0.7, 0.0),
        rotation: CoordUnit(0.0, 0.0, 0.0),
    }];

    set_and_wait!(
        navi_node,
        waypoints,
        0.10,
        0.30,
        executor,
        "Fifth automobile reached! Stopping navigation..."
    );

    match navi_node.call_yolo_blocking(
        &mut executor,
        Some("e_bike"),
        Some("camera2"),
        Duration::from_secs_f32(2.0),
    ) {
        Ok(yolo_response) => {
            println!("yolo: {:?}", yolo_response.message);
        }
        Err(err) => {
            log_info!(
                "navi_main",
                "YOLO service unavailable or timed out, continuing without detection: {:?}",
                err
            );
        }
    }

    let waypoints = vec![
        Pos {
            translation: CoordUnit(0.6, 0.0, 0.0),
            rotation: CoordUnit(0.0, 0.0, 6.28),
        },
        Pos {
            translation: CoordUnit(0.0, 0.0, 0.0),
            rotation: CoordUnit(0.0, 0.0, 6.28),
        },
    ];

    set_and_wait!(
        navi_node,
        waypoints,
        0.10,
        0.30,
        executor,
        "All waypoints reached! Stopping navigation..."
    );

    log_info!("navi_main", "Navigation completed, exiting cleanly");
    Ok(())
}
