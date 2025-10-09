use rclrs::{log_info, CreateBasicExecutor, SpinOptions};

use crate::navi::{CoordUnit, Pos};

pub mod navi;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let context = rclrs::Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    log_info!("navi_main", "starting navigator...");
    let navi_node = navi::NaviSubNode::new(&executor, "navi1", "lidar_data")?;

    let waypoints = vec![
        Pos {
            translation: CoordUnit(2.8, 0.0, 0.0),
            rotation: CoordUnit(0.0, 0.0, 0.0),
        },
        Pos {
            translation: CoordUnit(3.2, 0.5, 0.0),
            rotation: CoordUnit(0.0, 0.0, 1.57),
        },
        Pos {
            translation: CoordUnit(3.2, 3.2, 0.0),
            rotation: CoordUnit(0.0, 0.0, 1.57),
        },
    ];

    navi_node.set_destinations(waypoints, 0.2)?;

    executor.spin(SpinOptions::default());

    Ok(())
}
