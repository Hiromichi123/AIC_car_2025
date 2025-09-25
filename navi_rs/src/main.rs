use rclrs::{log_info, CreateBasicExecutor};

pub mod navi;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let context = rclrs::Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    log_info!("navi_main", "starting navigator...");
    let _subscription = navi::NaviSubNode::new(&executor, "navi1", "navi_coords")?;
    rclrs::RclrsErrorFilter::first_error(executor.spin(rclrs::SpinOptions::default()))?;

    Ok(())
}
