use rclrs::{log_info, CreateBasicExecutor, SpinOptions};
use std::time::Duration;

use crate::navi::{CoordUnit, Pos};

pub mod navi;

fn normalize_angle(mut angle: f64) -> f64 {
    while angle > std::f64::consts::PI {
        angle -= 2.0 * std::f64::consts::PI;
    }
    while angle < -std::f64::consts::PI {
        angle += 2.0 * std::f64::consts::PI;
    }
    angle
}

fn almost_same_xy(a: &Pos, b: &Pos) -> bool {
    (a.translation.0 - b.translation.0).abs() < 1e-6
        && (a.translation.1 - b.translation.1).abs() < 1e-6
}

fn build_rounded_turn(
    pivot: Pos,
    yaw_in: f64,
    yaw_out: f64,
    radius: f64,
    samples: usize,
) -> Vec<Pos> {
    let delta = normalize_angle(yaw_out - yaw_in);
    if delta.abs() < 0.05 || radius <= 0.0 {
        return vec![Pos {
            translation: pivot.translation,
            rotation: CoordUnit(0.0, 0.0, yaw_out),
        }];
    }

    let v_in = (yaw_in.cos(), yaw_in.sin());
    let v_out = (yaw_out.cos(), yaw_out.sin());
    let sign = if delta >= 0.0 { 1.0 } else { -1.0 };

    let trim = (radius * (delta.abs() / 2.0).tan()).min(0.35);
    let entry = (
        pivot.translation.0 - v_in.0 * trim,
        pivot.translation.1 - v_in.1 * trim,
    );
    let exit = (
        pivot.translation.0 + v_out.0 * trim,
        pivot.translation.1 + v_out.1 * trim,
    );

    let left_normal = (-v_in.1, v_in.0);
    let center = (
        entry.0 + left_normal.0 * sign * radius,
        entry.1 + left_normal.1 * sign * radius,
    );

    let start_angle = (entry.1 - center.1).atan2(entry.0 - center.0);
    let mut end_angle = (exit.1 - center.1).atan2(exit.0 - center.0);
    if sign > 0.0 && end_angle < start_angle {
        end_angle += 2.0 * std::f64::consts::PI;
    }
    if sign < 0.0 && end_angle > start_angle {
        end_angle -= 2.0 * std::f64::consts::PI;
    }
    let sweep = end_angle - start_angle;

    let mut points = Vec::new();
    let n = samples.max(3);
    for i in 0..=n {
        let t = i as f64 / n as f64;
        let theta = start_angle + sweep * t;
        let x = center.0 + radius * theta.cos();
        let y = center.1 + radius * theta.sin();
        let yaw = yaw_in + delta * t;

        points.push(Pos {
            translation: CoordUnit(x, y, pivot.translation.2),
            rotation: CoordUnit(0.0, 0.0, yaw),
        });
    }

    points
}

fn add_rounded_turns(raw: Vec<Pos>, radius: f64, samples: usize) -> Vec<Pos> {
    let mut result = Vec::new();
    let mut i = 0;

    while i < raw.len() {
        if i + 1 < raw.len() && almost_same_xy(&raw[i], &raw[i + 1]) {
            let yaw_in = raw[i].rotation.2;
            let yaw_out = raw[i + 1].rotation.2;
            let arc = build_rounded_turn(raw[i], yaw_in, yaw_out, radius, samples);
            result.extend(arc);
            i += 2;
            continue;
        }

        result.push(raw[i]);
        i += 1;
    }

    result
}

fn main() -> anyhow::Result<()> {
    let context = rclrs::Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    log_info!("navi_main", "starting navigator...");
    let navi_node = navi::NaviSubNode::new(&executor, "navigator", "lidar_data")?;
    // Add a small deterministic jitter to mimic real controller/estimation oscillation.
    navi_node.configure_jitter(true, 0.04, 0.12)?;

    let waypoints = add_rounded_turns(
        vec![
            Pos {
                translation: CoordUnit(3.3, 0.0, 0.0),
                rotation: CoordUnit(0.0, 0.0, 0.0),
            },
            Pos {
                translation: CoordUnit(3.3, 0.0, 0.0),
                rotation: CoordUnit(0.0, 0.0, 1.6),
            },
            Pos {
                translation: CoordUnit(3.3, 1.3, 0.0),
                rotation: CoordUnit(0.0, 0.0, 1.6),
            },
            Pos {
                translation: CoordUnit(3.3, 1.3, 0.0),
                rotation: CoordUnit(0.0, 0.0, 3.14),
            },
            Pos {
                translation: CoordUnit(2.4, 1.3, 0.0),
                rotation: CoordUnit(0.0, 0.0, 3.14),
            },
        ],
        0.22,
        5,
    );

    navi_node.set_destinations(waypoints, 0.25)?;
    loop {
        let mut spin_options = SpinOptions::default();
        spin_options.timeout = Some(Duration::from_millis(100));
        spin_options.only_next_available_work = true;
        spin_options.until_promise_resolved = None;
        executor.spin(spin_options);

        if navi_node.is_arrived() {
            log_info!(
                "navi_main",
                "First Camera point reached! Stopping navigation..."
            );
            break;
        }
    }

    let yolo_response =
        navi_node.call_yolo_blocking(&mut executor, Duration::from_secs_f32(2.0))?;
    println!("yolo: {:?}", yolo_response.message);
    std::thread::sleep(Duration::from_secs(2));

    let waypoints = add_rounded_turns(
        vec![
            Pos {
                translation: CoordUnit(1.7, 1.3, 0.0),
                rotation: CoordUnit(0.0, 0.0, 3.14),
            },
            Pos {
                translation: CoordUnit(1.7, 1.3, 0.0),
                rotation: CoordUnit(0.0, 0.0, 1.6),
            },
            Pos {
                translation: CoordUnit(1.7, 3.4, 0.0),
                rotation: CoordUnit(0.0, 0.0, 1.6),
            },
            Pos {
                translation: CoordUnit(1.7, 3.4, 0.0),
                rotation: CoordUnit(0.0, 0.0, 3.14),
            },
            Pos {
                translation: CoordUnit(0.5, 3.3, 0.0),
                rotation: CoordUnit(0.0, 0.0, 3.14),
            },
            Pos {
                translation: CoordUnit(0.5, 3.3, 0.0),
                rotation: CoordUnit(0.0, 0.0, 1.6),
            },
        ],
        0.2,
        5,
    );

    navi_node.set_destinations(waypoints, 0.25)?;
    loop {
        let mut spin_options = SpinOptions::default();
        spin_options.timeout = Some(Duration::from_millis(100));
        spin_options.only_next_available_work = true;
        spin_options.until_promise_resolved = None;
        executor.spin(spin_options);

        if navi_node.is_arrived() {
            log_info!("navi_main", "All waypoints reached! Stopping navigation...");
            break;
        }
    }

    // OCR can take longer on CPU; allow a more generous timeout
    let ocr_response = navi_node.call_ocr_blocking(&mut executor, Duration::from_secs_f32(5.0))?;
    println!("ocr: {:?}", ocr_response.message);
    std::thread::sleep(Duration::from_secs(2));

    let waypoints = vec![Pos {
        translation: CoordUnit(0.6, 2.5, 0.0),
        rotation: CoordUnit(0.0, 0.0, 1.6),
    }];

    navi_node.set_destinations(waypoints, 0.25)?;
    loop {
        let mut spin_options = SpinOptions::default();
        spin_options.timeout = Some(Duration::from_millis(100));
        spin_options.only_next_available_work = true;
        spin_options.until_promise_resolved = None;
        executor.spin(spin_options);

        if navi_node.is_arrived() {
            log_info!("navi_main", "All waypoints reached! Stopping navigation...");
            break;
        }
    }

    // log_info!("navi_main", "Sleeping for 4 secs.");
    // sleep(std::time::Duration::from_secs_f32(4.0));
    // OCR can take longer on CPU; allow a more generous timeout
    let ocr_response = navi_node.call_ocr_blocking(&mut executor, Duration::from_secs_f32(5.0))?;
    println!("ocr: {:?}", ocr_response.message);
    std::thread::sleep(Duration::from_secs(2));

    let waypoints = vec![
        Pos {
            translation: CoordUnit(0.6, 2.5, 0.0),
            rotation: CoordUnit(0.0, 0.0, 1.6),
        },
        Pos {
            translation: CoordUnit(0.6, 0.0, 0.0),
            rotation: CoordUnit(0.0, 0.0, 1.6),
        },
        Pos {
            translation: CoordUnit(0.0, 0.0, 0.0),
            rotation: CoordUnit(0.0, 0.0, 6.28),
        },
    ];

    navi_node.set_destinations(waypoints, 0.25)?;
    loop {
        let mut spin_options = SpinOptions::default();
        spin_options.timeout = Some(Duration::from_millis(100));
        spin_options.only_next_available_work = true;
        spin_options.until_promise_resolved = None;
        executor.spin(spin_options);

        if navi_node.is_arrived() {
            log_info!("navi_main", "All waypoints reached! Stopping navigation...");
            break;
        }
    }

    log_info!("navi_main", "Navigation completed, exiting cleanly");
    Ok(())
}
