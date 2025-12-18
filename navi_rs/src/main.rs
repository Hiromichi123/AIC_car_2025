use rclrs::{log_info, CreateBasicExecutor, SpinOptions};
use std::thread::sleep;
use std::time::Duration;

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

fn format_plate_for_tts(raw: &str) -> String {
    let trimmed = raw.trim();
    if trimmed.is_empty() {
        return String::new();
    }

    // For license plates, we want character-by-character pronunciation.
    // - ASCII letters -> Chinese letter names (more reliable than raw 'ABC')
    // - ASCII digits -> Chinese digits (avoid being read as a whole number)
    // - Keep Chinese characters (province, etc.) as-is
    // - Drop separators like spaces/dots/middle dots/hyphens
    let mut tokens: Vec<String> = Vec::new();
    for ch in trimmed.chars() {
        if ch.is_whitespace() {
            continue;
        }
        if matches!(
            ch,
            '-' | '_' | '.' | '·' | '•' | '，' | ',' | '。' | ':' | '：'
        ) {
            continue;
        }

        let token = if ch.is_ascii_alphabetic() {
            match ch.to_ascii_uppercase() {
                'A' => "诶",
                'B' => "比",
                'C' => "西",
                'D' => "迪",
                'E' => "伊",
                'F' => "艾弗",
                'G' => "吉",
                'H' => "艾尺",
                'I' => "艾",
                'J' => "杰",
                'K' => "开",
                'L' => "艾勒",
                'M' => "艾姆",
                'N' => "恩",
                'O' => "欧",
                'P' => "屁",
                'Q' => "丘",
                'R' => "阿尔",
                'S' => "艾丝",
                'T' => "提",
                'U' => "优",
                'V' => "维",
                'W' => "达不溜",
                'X' => "艾克斯",
                'Y' => "歪",
                'Z' => "贼德",
                _ => {
                    // Should be unreachable for ASCII alphabetic.
                    ""
                }
            }
            .to_string()
        } else if ch.is_ascii_digit() {
            match ch {
                '0' => "零",
                '1' => "一",
                '2' => "二",
                '3' => "三",
                '4' => "四",
                '5' => "五",
                '6' => "六",
                '7' => "七",
                '8' => "八",
                '9' => "九",
                _ => "",
            }
            .to_string()
        } else {
            ch.to_string()
        };

        if !token.is_empty() {
            tokens.push(token);
        }
    }

    if tokens.is_empty() {
        trimmed.to_string()
    } else {
        tokens.join("，")
    }
}

fn main() -> anyhow::Result<()> {
    let context = rclrs::Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    log_info!("navi_main", "starting navigator...");
    let navi_node = navi::NaviSubNode::new(&executor, "navigator", "lidar_data")?;
    let mut vision_result_blocka = VisionResult::new();

    let _ = navi_node.call_tts_blocking(
        &mut executor,
        "语音合成神经网络初始化成功",
        Duration::from_secs(20),
    );

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
        Duration::from_secs_f32(10.0),
        2,
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
        &mut vision_result_blocka,
    );

    let waypoints = vec![
        Pos {
            translation: CoordUnit(3.35, 0.0, 0.0),
            rotation: CoordUnit(0.0, 0.0, 0.0),
        },
        Pos {
            translation: CoordUnit(3.35, 0.65, 0.0),
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
        &mut vision_result_blocka,
    );

    let _ = navi_node.call_tts_blocking(
        &mut executor,
        "前方是禁止通行指示牌",
        Duration::from_secs(20),
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

    if let Err(err) =
        navi_node.call_servo_blocking(&mut executor, ServoState::RIGHT, Duration::from_secs(1))
    {
        log_info!("navi_main", "SERVO left command failed: {:?}", err);
    }

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
        &mut vision_result_blocka,
    );

    vision::run_yolo_detection(
        &navi_node,
        &mut executor,
        VisionRequest {
            label: "e-bike scan (block a)",
            model: Some("e_bike"),
            camera: Some("camera1"),
            timeout: Duration::from_secs_f32(2.0),
            category: Some(VisionCategory::Automobile),
        },
        &mut vision_result_blocka,
    );

    // Block A Summary
    log_info!(
        "navi_main",
        "\\033[31m Vision summary => {} \\033[0m",
        vision_result_blocka.summary()
    );

    let good_people = vision_result_blocka.good_people.unwrap_or_default();
    let bad_people = vision_result_blocka.bad_people.unwrap_or_default();
    let ebike = vision_result_blocka.automobile.unwrap_or_default();

    let tts_text = format!(
        "诶区域发现社区人员{}人,发现非社区人员{}人,违停车辆{}辆,已拍照",
        good_people, bad_people, ebike
    );
    navi_node.call_tts_blocking(&mut executor, &tts_text, Duration::from_secs(10))?;

    let mut vision_result_blockb = VisionResult::new();

    if let Err(err) =
        navi_node.call_servo_blocking(&mut executor, ServoState::LEFT, Duration::from_secs(1))
    {
        log_info!("navi_main", "SERVO left command failed: {:?}", err);
    }

    vision::run_yolo_detection(
        &navi_node,
        &mut executor,
        VisionRequest {
            label: "camera1 people scan (servo left sweep)",
            model: Some("people"),
            camera: Some("camera1"),
            timeout: Duration::from_secs_f32(2.0),
            category: Some(VisionCategory::GoodPeople),
        },
        &mut vision_result_blockb,
    );

    let waypoints = vec![Pos {
        translation: CoordUnit(2.2, 1.2, 0.0),
        rotation: CoordUnit(0.0, 0.0, 0.0),
    }];

    set_and_wait!(
        navi_node,
        waypoints,
        0.10,
        0.30,
        executor,
        "All waypoints reached! Stopping navigation..."
    );

    vision::run_ocr_detection(
        &navi_node,
        &mut executor,
        Some("camera2"),
        Duration::from_secs_f32(8.0),
        "OCR checkpoint 1",
        &mut vision_result_blockb,
    );

    let mut fire_building = vision_result_blockb.fire.unwrap_or_default();

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
        &mut vision_result_blockb,
    );

    let mut tts_building = String::from("未知大厦");
    while let Some(text) = vision_result_blockb.ocr.pop() {
        if text.contains("大厦") {
            tts_building = text;
            break;
        }
    }

    fire_building = vision_result_blockb.fire.unwrap_or_default() - fire_building;

    // 使用 format! 重新赋值
    let tts_text = format!("{}发现火灾{}处", tts_building, fire_building);
    navi_node.call_tts_blocking(&mut executor, &tts_text, Duration::from_secs(10))?;

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
        &mut vision_result_blockb,
    );

    vision::run_ocr_detection(
        &navi_node,
        &mut executor,
        Some("camera1"),
        Duration::from_secs_f32(8.0),
        "OCR checkpoint 1",
        &mut vision_result_blockb,
    );

    let mut fire_building = vision_result_blockb.fire.unwrap_or_default();

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
        &mut vision_result_blockb,
    );

    let mut tts_building = String::from("未知大厦");
    while let Some(text) = vision_result_blockb.ocr.pop() {
        if text.contains("大厦") {
            tts_building = text;
            break;
        }
    }

    fire_building = vision_result_blockb.fire.unwrap_or_default() - fire_building;

    // 使用 format! 重新赋值
    let tts_text = format!("{}发现火灾{}处", tts_building, fire_building);
    navi_node.call_tts_blocking(&mut executor, &tts_text, Duration::from_secs(10))?;

    // Block B Summary
    log_info!(
        "navi_main",
        "\\033[31m Vision summary => {} \\033[0m",
        vision_result_blockb.summary()
    );

    let good_people = vision_result_blockb.good_people.unwrap_or_default();
    let bad_people = vision_result_blockb.bad_people.unwrap_or_default();
    let ebike = vision_result_blocka.automobile.unwrap_or_default();

    let people_sum = vision_result_blocka.good_people.unwrap_or_default()
        + vision_result_blockb.good_people.unwrap_or_default();

    let tts_text = format!(
        "比区域发现社区人员{}人,发现非社区人员{}人,违停车辆{}辆,已拍照",
        good_people, bad_people, ebike
    );
    navi_node.call_tts_blocking(&mut executor, &tts_text, Duration::from_secs(10))?;

    let tts_text2 = format!("共发现社区人员{}人", people_sum);
    navi_node.call_tts_blocking(&mut executor, &tts_text2, Duration::from_secs(10))?;

    let mut vision_result_block_parking = VisionResult::new();

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

    // 任务完成后回正舵机
    if let Err(err) =
        navi_node.call_servo_blocking(&mut executor, ServoState::CENTER, Duration::from_secs(1))
    {
        log_info!("navi_main", "SERVO center command failed: {:?}", err);
    }

    let waypoints = vec![Pos {
        translation: CoordUnit(0.6, 3.4, 0.0),
        rotation: CoordUnit(0.0, 0.0, 0.0),
    }];

    set_and_wait!(
        navi_node,
        waypoints,
        0.10,
        0.30,
        executor,
        "First car reached! Stopping navigation..."
    );

    vision::run_ocr_detection(
        &navi_node,
        &mut executor,
        Some("camera2"),
        Duration::from_secs_f32(8.0),
        "OCR checkpoint 3 car1",
        &mut vision_result_block_parking,
    );

    let car_number = vision_result_block_parking.ocr.pop().unwrap_or_else(|| {
        log_info!("vision", "Car number parse error.");
        String::from("未知车牌")
    });

    let plate_tts = format_plate_for_tts(&car_number);
    let tts_text = format!("1号停车场车牌号为{}", plate_tts);
    navi_node.call_tts_blocking(&mut executor, &tts_text, Duration::from_secs(10))?;

    let waypoints = vec![Pos {
        translation: CoordUnit(0.6, 2.8, 0.0),
        rotation: CoordUnit(0.0, 0.0, 0.0),
    }];

    set_and_wait!(
        navi_node,
        waypoints,
        0.10,
        0.30,
        executor,
        "Second car reached! Stopping navigation..."
    );

    vision::run_ocr_detection(
        &navi_node,
        &mut executor,
        Some("camera2"),
        Duration::from_secs_f32(8.0),
        "OCR checkpoint 4 car2",
        &mut vision_result_block_parking,
    );

    let car_number = vision_result_block_parking.ocr.pop().unwrap_or_else(|| {
        log_info!("vision", "Car number parse error.");
        String::from("未知车牌")
    });

    let plate_tts = format_plate_for_tts(&car_number);
    let tts_text = format!("2号停车场车牌号为{}", plate_tts);
    navi_node.call_tts_blocking(&mut executor, &tts_text, Duration::from_secs(10))?;

    sleep(std::time::Duration::from_secs_f32(1.0));

    vision::run_yolo_detection(
        &navi_node,
        &mut executor,
        VisionRequest {
            label: "rubbish bin scan",
            model: Some("rubbish_bin"),
            camera: Some("camera1"),
            timeout: Duration::from_secs_f32(20.0),
            category: None,
        },
        &mut vision_result_block_parking,
    );

    if let Some(rubbish_tts) = vision_result_block_parking.rubbish.as_deref() {
        let _ = navi_node.call_tts_blocking(&mut executor, rubbish_tts, Duration::from_secs(10));
    } else {
        let _ =
            navi_node.call_tts_blocking(&mut executor, "垃圾分类识别失败", Duration::from_secs(10));
    }

    let waypoints = vec![Pos {
        translation: CoordUnit(0.6, 2.1, 0.0),
        rotation: CoordUnit(0.0, 0.0, 0.0),
    }];

    set_and_wait!(
        navi_node,
        waypoints,
        0.10,
        0.20,
        executor,
        "Third car reached! Stopping navigation..."
    );

    vision::run_ocr_detection(
        &navi_node,
        &mut executor,
        Some("camera2"),
        Duration::from_secs_f32(8.0),
        "OCR checkpoint 4 car3",
        &mut vision_result_block_parking,
    );

    let car_number = vision_result_block_parking.ocr.pop().unwrap_or_else(|| {
        log_info!("vision", "Car number parse error.");
        String::from("未知车牌")
    });

    let plate_tts = format_plate_for_tts(&car_number);
    let tts_text = format!("3号停车场车牌号为{}", plate_tts);
    navi_node.call_tts_blocking(&mut executor, &tts_text, Duration::from_secs(10))?;

    let waypoints = vec![Pos {
        translation: CoordUnit(0.6, 1.5, 0.0),
        rotation: CoordUnit(0.0, 0.0, 0.0),
    }];

    set_and_wait!(
        navi_node,
        waypoints,
        0.10,
        0.20,
        executor,
        "Forth building reached! Stopping navigation..."
    );

    vision::run_ocr_detection(
        &navi_node,
        &mut executor,
        Some("camera1"),
        Duration::from_secs_f32(8.0),
        "OCR checkpoint 4",
        &mut vision_result_block_parking,
    );

    vision::run_yolo_detection(
        &navi_node,
        &mut executor,
        VisionRequest {
            label: "camera1 fire scan (third building)",
            model: Some("fire"),
            camera: Some("camera1"),
            timeout: Duration::from_secs_f32(10.0),
            category: Some(VisionCategory::Fire),
        },
        &mut vision_result_block_parking,
    );

    let mut tts_building = String::from("未知大厦");
    while let Some(text) = vision_result_block_parking.ocr.pop() {
        if text.contains("大厦") {
            tts_building = text;
            break;
        }
    }

    let fire_building = vision_result_block_parking.fire.unwrap_or_default();

    // 使用 format! 重新赋值
    let tts_text = format!("{}发现火灾{}处", tts_building, fire_building);
    navi_node.call_tts_blocking(&mut executor, &tts_text, Duration::from_secs(10))?;

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
        &mut vision_result_block_parking,
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
        &mut vision_result_block_parking,
    );

    let ebike_upright = vision_result_block_parking
        .ebike_upright
        .unwrap_or_default();
    let ebike_fallen = vision_result_block_parking.ebike_fallen.unwrap_or_default();
    let tts_text = format!(
        "停车区域电瓶车停放正确{}辆,倒伏{}辆",
        ebike_upright, ebike_fallen
    );
    let _ = navi_node.call_tts_blocking(&mut executor, &tts_text, Duration::from_secs(10));

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

    log_info!(
        "navi_main",
        "\\033[31m Vision summary => {} \\033[0m",
        vision_result_block_parking.summary()
    );
    log_info!("navi_main", "Navigation completed, exiting cleanly");
    Ok(())
}
