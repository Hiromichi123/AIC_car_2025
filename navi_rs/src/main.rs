use rclrs::{log_info, CreateBasicExecutor};

use crate::navi::{CoordUnit, Pos};

pub mod navi;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let context = rclrs::Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    log_info!("navi_main", "starting navigator...");
    let navi_node = navi::NaviSubNode::new(&executor, "navi1", "lidar_data")?;

    let destination = Pos {
        translation: CoordUnit(3.5, 0.0, 0.0),
        rotation: CoordUnit(0.0, 0.0, 0.0),
    };
    navi_node.set_destination(destination, 0.5)?;
    while !navi_node.is_arrived() {
        rclrs::RclrsErrorFilter::first_error(executor.spin(rclrs::SpinOptions::default()))?;
    }

    let destination = Pos {
        translation: CoordUnit(3.5, 0.0, 0.0),
        rotation: CoordUnit(0.0, 1.7, 0.0),
    };
    navi_node.set_destination(destination, 0.5)?;
    while !navi_node.is_arrived() {
        rclrs::RclrsErrorFilter::first_error(executor.spin(rclrs::SpinOptions::default()))?;
    }

    Ok(())
}
