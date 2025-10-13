use rclrs::{log_info, CreateBasicExecutor, SpinOptions};
use std::{thread::sleep, time::Duration};

use crate::navi::{CoordUnit, Pos};

pub mod navi;

fn main() -> anyhow::Result<()> {
    let context = rclrs::Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    log_info!("navi_main", "starting navigator...");
    let navi_node = navi::NaviSubNode::new(&executor, "navi1", "lidar_data")?;

    let waypoints = vec![
        Pos {
            translation: CoordUnit(3.3, 0.0, 0.0),
            rotation: CoordUnit(0.0, 0.0, 0.0),
        },
        Pos {
            translation: CoordUnit(3.3, 1.3, 0.0),
            rotation: CoordUnit(0.0, 0.0, 0.0),
        },
        Pos {
            translation: CoordUnit(2.5, 1.3, 0.0),
            rotation: CoordUnit(0.0, 0.0, 0.0),
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
            log_info!("navi_main", "First Camera reached! Stopping navigation...");
            break;
        }
    }

    log_info!("navi_main", "Sleeping for 5 secs.");
    sleep(std::time::Duration::from_secs_f32(5.0));

    let waypoints = vec![
        Pos {
            translation: CoordUnit(1.7, 1.3, 0.0),
            rotation: CoordUnit(0.0, 0.0, 0.0),
        },
        Pos {
            translation: CoordUnit(1.7, 3.3, 0.0),
            rotation: CoordUnit(0.0, 0.0, 0.0),
        },
        Pos {
            translation: CoordUnit(0.6, 3.1, 0.0),
            rotation: CoordUnit(0.0, 0.0, 1.6),
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

    log_info!("navi_main", "Sleeping for 4 secs.");
    sleep(std::time::Duration::from_secs_f32(4.0));

    let waypoints = vec![Pos {
        translation: CoordUnit(0.6, 2.8, 0.0),
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

    log_info!("navi_main", "Sleeping for 4 secs.");
    sleep(std::time::Duration::from_secs_f32(4.0));

    let waypoints = vec![
        Pos {
            translation: CoordUnit(0.6, 2.8, 0.0),
            rotation: CoordUnit(0.0, 0.0, 6.28),
        },
        Pos {
            translation: CoordUnit(0.6, 0.0, 0.0),
            rotation: CoordUnit(0.0, 0.0, 6.28),
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
