use rclrs::{log_info, CreateBasicExecutor, SpinOptions};
use std::{thread::sleep, time::Duration};

mod navi;
mod vision;

use crate::navi::{CoordUnit, Pos, ServoState};
use crate::vision::{VisionCategory, VisionRequest, VisionResult};

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
    let mut vision_result = VisionResult::new();

    navi_node.call_tts_blocking(
        &mut executor,
        "语音合成网络初始化成功",
        Duration::from_secs(10),
    )?;

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
    vision::wait_for_green_light(
        &navi_node,
        &mut executor,
        Duration::from_secs_f32(2.0),
        20,
        Some("camera1"),
    );

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

    vision::run_yolo_detection(
        &navi_node,
        &mut executor,
        VisionRequest {
            label: "camera1 people scan (waypoint 1)",
            model: Some("people"),
            camera: Some("camera1"),
            timeout: Duration::from_secs_f32(2.0),
            category: Some(VisionCategory::GoodPeople),
        },
        &mut vision_result,
    );

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

    vision::run_yolo_detection(
        &navi_node,
        &mut executor,
        VisionRequest {
            label: "camera2 people scan (waypoint 2)",
            model: Some("people"),
            camera: Some("camera2"),
            timeout: Duration::from_secs_f32(2.0),
            category: Some(VisionCategory::BadPeople),
        },
        &mut vision_result,
    );

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

    vision::run_yolo_detection(
        &navi_node,
        &mut executor,
        VisionRequest {
            label: "camera1 people scan (waypoint 3)",
            model: Some("people"),
            camera: Some("camera1"),
            timeout: Duration::from_secs_f32(2.0),
            category: Some(VisionCategory::GoodPeople),
        },
        &mut vision_result,
    );

    if let Err(err) =
        navi_node.call_servo_blocking(&mut executor, ServoState::RIGHT, Duration::from_secs(1))
    {
        log_info!("navi_main", "SERVO left command failed: {:?}", err);
    }

    vision::run_yolo_detection(
        &navi_node,
        &mut executor,
        VisionRequest {
            label: "camera1 people scan (servo right sweep)",
            model: Some("people"),
            camera: Some("camera1"),
            timeout: Duration::from_secs_f32(2.0),
            category: Some(VisionCategory::GoodPeople),
        },
        &mut vision_result,
    );

    vision::run_ocr_detection(
        &navi_node,
        &mut executor,
        Duration::from_secs_f32(2.0),
        "OCR checkpoint 1",
        &mut vision_result,
    );

    vision::run_yolo_detection(
        &navi_node,
        &mut executor,
        VisionRequest {
            label: "fire scan (camera2)",
            model: Some("fire"),
            camera: Some("camera2"),
            timeout: Duration::from_secs_f32(2.0),
            category: Some(VisionCategory::Fire),
        },
        &mut vision_result,
    );

    navi_node.call_tts_blocking(&mut executor, "晨光大厦", Duration::from_secs(10))?;

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

    vision::run_yolo_detection(
        &navi_node,
        &mut executor,
        VisionRequest {
            label: "camera1 people scan (mid route)",
            model: Some("people"),
            camera: Some("camera1"),
            timeout: Duration::from_secs_f32(2.0),
            category: Some(VisionCategory::GoodPeople),
        },
        &mut vision_result,
    );

    if let Err(err) =
        navi_node.call_servo_blocking(&mut executor, ServoState::LEFT, Duration::from_secs(1))
    {
        log_info!("navi_main", "SERVO left command failed: {:?}", err);
    }

    vision::wait_for_green_light(
        &navi_node,
        &mut executor,
        Duration::from_secs_f32(2.0),
        20,
        Some("camera1"),
    );

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

    vision::run_ocr_detection(
        &navi_node,
        &mut executor,
        Duration::from_secs_f32(2.0),
        "OCR checkpoint 2",
        &mut vision_result,
    );

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

    vision::run_yolo_detection(
        &navi_node,
        &mut executor,
        VisionRequest {
            label: "rubbish bin scan",
            model: Some("rubbish_bin"),
            camera: Some("camera1"),
            timeout: Duration::from_secs_f32(2.0),
            category: None,
        },
        &mut vision_result,
    );

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

    vision::run_yolo_detection(
        &navi_node,
        &mut executor,
        VisionRequest {
            label: "e-bike scan (segment 1)",
            model: Some("e_bike"),
            camera: Some("camera2"),
            timeout: Duration::from_secs_f32(2.0),
            category: Some(VisionCategory::Automobile),
        },
        &mut vision_result,
    );

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

    vision::run_yolo_detection(
        &navi_node,
        &mut executor,
        VisionRequest {
            label: "e-bike scan (segment 2)",
            model: Some("e_bike"),
            camera: Some("camera2"),
            timeout: Duration::from_secs_f32(2.0),
            category: Some(VisionCategory::Automobile),
        },
        &mut vision_result,
    );

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

    log_info!("navi_main", "Vision summary => {}", vision_result.summary());
    log_info!("navi_main", "Navigation completed, exiting cleanly");
    Ok(())
}
