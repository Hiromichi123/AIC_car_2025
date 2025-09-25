use anyhow::Result;
use rclrs::{log_error, log_info, Context, CreateBasicExecutor, Executor};
use ros2_tools::msg::LidarPose;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

// 引入我们要测试的模块
use navi_rs::navi::{NaviSubNode, Pos};

/// 测试用的LidarPose消息发布器
#[allow(unused)]
struct TestPublisher {
    publisher: rclrs::Publisher<LidarPose>,
    node: rclrs::Node,
}

impl TestPublisher {
    fn new(executor: &Executor, topic: &str) -> Result<Self> {
        let node = executor.create_node("test_publisher")?;
        let publisher = node.create_publisher::<LidarPose>(topic)?;

        Ok(TestPublisher { publisher, node })
    }

    fn publish_position(
        &self,
        x: f64,
        y: f64,
        z: f64,
        roll: f64,
        pitch: f64,
        yaw: f64,
    ) -> Result<()> {
        let mut msg = LidarPose::default();
        msg.x = x;
        msg.y = y;
        msg.z = z;
        msg.roll = roll;
        msg.pitch = pitch;
        msg.yaw = yaw;

        self.publisher.publish(msg)?;
        log_info!(
            "test_publisher",
            "Published position: ({:.2}, {:.2}, {:.2})",
            x,
            y,
            z
        );
        Ok(())
    }
}

/// 测试结果收集器
#[derive(Debug, Default, Clone)]
struct TestResults {
    received_positions: Vec<(f64, f64, f64)>,
    arrival_detected: bool,
    _test_completed: bool,
}

/// 扩展NaviSubNode以支持测试 (通过本地trait扩展避免外部类型固有实现限制)
trait NaviSubNodeTestExt {
    fn set_destination(&self, dest: Pos, threshold: f64) -> Result<()>;
    fn is_arrived(&self) -> bool;
    fn get_current_position(&self) -> Option<Pos>;
}

impl NaviSubNodeTestExt for NaviSubNode {
    fn set_destination(&self, dest: Pos, threshold: f64) -> Result<()> {
        if let Ok(mut navi) = self.navi_instance.lock() {
            navi.set_dest(dest, threshold);
            Ok(())
        } else {
            Err(anyhow::anyhow!("Failed to acquire navigation lock"))
        }
    }

    fn is_arrived(&self) -> bool {
        self.navi_instance
            .lock()
            .map(|navi| navi.is_arrived)
            .unwrap_or(false)
    }

    fn get_current_position(&self) -> Option<Pos> {
        self.navi_instance.lock().ok().map(|navi| navi.current_pos)
    }
}

#[tokio::test]
async fn test_navigation_basic_functionality() -> Result<()> {
    // 初始化ROS2上下文
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    log_info!("test", "Starting basic navigation functionality test");

    // 创建测试发布器和导航订阅器
    let test_topic = "test_lidar_data";
    let publisher = TestPublisher::new(&executor, test_topic)?;
    let navi_node = NaviSubNode::new(&executor, "test_navigation_node", test_topic)?;

    // 设置目标位置 (5.0, 5.0, 0.0)，阈值为 1.0 米
    let destination = Pos {
        translation: navi_rs::navi::CoordUnit(5.0, 5.0, 0.0),
        rotation: navi_rs::navi::CoordUnit(0.0, 0.0, 0.0),
    };
    navi_node.set_destination(destination, 1.0)?;

    // 创建测试结果收集器
    let test_results = Arc::new(Mutex::new(TestResults::default()));
    let results_clone = Arc::clone(&test_results);

    // 启动执行器在后台线程
    let _executor_handle = thread::spawn(move || {
        let errors = executor.spin(rclrs::SpinOptions::default());
        for e in errors {
            log_error!("executor", "Executor error: {}", e);
        }
    });

    // 等待一小段时间让节点初始化
    thread::sleep(Duration::from_millis(500));

    // 测试序列：模拟机器人从(0,0)移动到目标(5,5)
    let test_positions = vec![
        (0.0, 0.0, 0.0), // 起始位置
        (1.0, 1.0, 0.0), // 中间位置1
        (2.0, 2.0, 0.0), // 中间位置2
        (3.0, 3.0, 0.0), // 中间位置3
        (4.0, 4.0, 0.0), // 接近目标
        (5.0, 5.0, 0.0), // 到达目标
    ];

    for (i, (x, y, z)) in test_positions.iter().enumerate() {
        // 发布位置
        publisher.publish_position(*x, *y, *z, 0.0, 0.0, 0.0)?;

        // 等待消息处理
        thread::sleep(Duration::from_millis(200));

        // 记录测试结果
        if let Ok(mut results) = results_clone.lock() {
            results.received_positions.push((*x, *y, *z));

            // 检查当前位置
            if let Some(current_pos) = navi_node.get_current_position() {
                log_info!(
                    "test",
                    "Step {}: Current position ({:.2}, {:.2}, {:.2})",
                    i,
                    current_pos.translation.0,
                    current_pos.translation.1,
                    current_pos.translation.2
                );
            }

            // 检查是否到达目标
            if navi_node.is_arrived() {
                results.arrival_detected = true;
                log_info!("test", "✅ Arrival detected at step {}", i);
                break;
            }
        }
    }

    // 最终验证
    thread::sleep(Duration::from_millis(300));

    let final_results = test_results.lock().unwrap();

    // 验证测试结果
    assert!(
        !final_results.received_positions.is_empty(),
        "No positions were received"
    );
    assert!(final_results.arrival_detected, "Arrival was not detected");
    assert!(
        navi_node.is_arrived(),
        "Navigation node should report arrival"
    );

    log_info!("test", "✅ Basic navigation functionality test passed!");
    log_info!(
        "test",
        "Received {} positions, arrival detected: {}",
        final_results.received_positions.len(),
        final_results.arrival_detected
    );

    Ok(())
}

#[tokio::test]
async fn test_navigation_precision() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    log_info!("test", "Starting navigation precision test");

    let test_topic = "precision_test_lidar_data";
    let publisher = TestPublisher::new(&executor, test_topic)?;
    let navi_node = NaviSubNode::new(&executor, "precision_test_node", test_topic)?;

    // 设置精确的目标和小阈值
    let destination = Pos {
        translation: navi_rs::navi::CoordUnit(10.0, 10.0, 0.0),
        rotation: navi_rs::navi::CoordUnit(0.0, 0.0, 0.0),
    };
    navi_node.set_destination(destination, 0.1)?; // 0.1米的精确阈值

    let _executor_handle = thread::spawn(move || {
        executor.spin(rclrs::SpinOptions::default());
    });

    thread::sleep(Duration::from_millis(300));

    // 测试不同精度的接近
    let precision_tests = vec![
        (9.0, 9.0, 0.0, false),    // 距离 > 0.1m，不应该到达
        (9.95, 9.95, 0.0, true),   // 距离 ≈ 0.07m，应该到达
        (10.05, 10.05, 0.0, true), // 距离 ≈ 0.07m，应该到达
    ];

    for (i, (x, y, z, should_arrive)) in precision_tests.iter().enumerate() {
        // 重置到达状态（如果需要）
        if i > 0 {
            // 先移动到远处重置状态
            publisher.publish_position(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)?;
            thread::sleep(Duration::from_millis(100));

            // 重新设置目标
            navi_node.set_destination(destination, 0.1)?;
            thread::sleep(Duration::from_millis(100));
        }

        publisher.publish_position(*x, *y, *z, 0.0, 0.0, 0.0)?;
        thread::sleep(Duration::from_millis(300));

        let arrived = navi_node.is_arrived();
        let distance = ((x - 10.0).powi(2) + (y - 10.0).powi(2)).sqrt();

        log_info!(
            "test",
            "Precision test {}: pos=({:.2}, {:.2}), distance={:.3}, arrived={}, expected={}",
            i,
            x,
            y,
            distance,
            arrived,
            should_arrive
        );

        if *should_arrive {
            assert!(
                arrived,
                "Should have arrived at position ({}, {}) with distance {:.3}",
                x, y, distance
            );
        } else {
            assert!(
                !arrived,
                "Should NOT have arrived at position ({}, {}) with distance {:.3}",
                x, y, distance
            );
        }
    }

    log_info!("test", "✅ Navigation precision test passed!");
    Ok(())
}

#[tokio::test]
async fn test_navigation_no_destination() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    log_info!("test", "Starting no destination test");

    let test_topic = "no_dest_test_lidar_data";
    let publisher = TestPublisher::new(&executor, test_topic)?;
    let navi_node = NaviSubNode::new(&executor, "no_dest_test_node", test_topic)?;

    // 不设置目标，直接发布位置
    let _executor_handle = thread::spawn(move || {
        executor.spin(rclrs::SpinOptions::default());
    });

    thread::sleep(Duration::from_millis(300));

    // 发布一些位置数据
    for i in 0..5 {
        publisher.publish_position(i as f64, i as f64, 0.0, 0.0, 0.0, 0.0)?;
        thread::sleep(Duration::from_millis(100));

        // 应该始终没有到达
        assert!(
            !navi_node.is_arrived(),
            "Should not arrive without destination set"
        );
    }

    log_info!("test", "✅ No destination test passed!");
    Ok(())
}

#[tokio::test]
async fn test_navigation_multiple_destinations() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    log_info!("test", "Starting multiple destinations test");

    let test_topic = "multi_dest_test_lidar_data";
    let publisher = TestPublisher::new(&executor, test_topic)?;
    let navi_node = NaviSubNode::new(&executor, "multi_dest_test_node", test_topic)?;

    let _executor_handle = thread::spawn(move || {
        executor.spin(rclrs::SpinOptions::default());
    });

    thread::sleep(Duration::from_millis(300));

    // 第一个目标
    let dest1 = Pos {
        translation: navi_rs::navi::CoordUnit(3.0, 3.0, 0.0),
        rotation: navi_rs::navi::CoordUnit(0.0, 0.0, 0.0),
    };
    navi_node.set_destination(dest1, 0.5)?;

    // 移动到第一个目标
    publisher.publish_position(3.0, 3.0, 0.0, 0.0, 0.0, 0.0)?;
    thread::sleep(Duration::from_millis(300));

    assert!(navi_node.is_arrived(), "Should arrive at first destination");
    log_info!("test", "✅ Reached first destination");

    // 设置第二个目标
    let dest2 = Pos {
        translation: navi_rs::navi::CoordUnit(6.0, 6.0, 0.0),
        rotation: navi_rs::navi::CoordUnit(0.0, 0.0, 0.0),
    };
    navi_node.set_destination(dest2, 0.5)?;

    // 移动到第二个目标
    publisher.publish_position(6.0, 6.0, 0.0, 0.0, 0.0, 0.0)?;
    thread::sleep(Duration::from_millis(300));

    assert!(
        navi_node.is_arrived(),
        "Should arrive at second destination"
    );
    log_info!("test", "✅ Reached second destination");

    log_info!("test", "✅ Multiple destinations test passed!");
    Ok(())
}
