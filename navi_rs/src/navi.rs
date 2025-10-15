use geometry_msgs::msg::PoseStamped;
use rclrs::{log_info, Client, Publisher};
use rclrs::{SubscriptionState, WorkerState};
use ros2_tools::msg::LidarPose;
use ros2_tools::srv::{OCR_Request, OCR_Response, YOLO_Request, YOLO_Response, OCR, YOLO};
use std::sync::Arc;
use std::sync::Mutex;
use std::time::{Duration, Instant};

#[derive(Debug, Default, Clone, Copy)]
pub struct CoordUnit(pub f64, pub f64, pub f64);

#[derive(Debug, Default, Clone, Copy)]
pub struct Pos {
    pub translation: CoordUnit, // (x, y, z)
    pub rotation: CoordUnit,    // (roll, pitch, yaw)
}

#[derive(Debug, Default)]
pub struct Navi {
    pub dest_pos: Vec<Pos>,
    pub current_pos: Pos,
    pub velocity: Pos,
    pub is_arrived: bool,
    arrive_threshold: f64,
    current_waypoint_index: usize, // 当前目标点索引
}

pub struct NaviSubNode {
    #[allow(unused)]
    pub navi_instance: Arc<Mutex<Navi>>,
    goal_publisher: Publisher<PoseStamped>,
    yolo_client: Arc<Client<YOLO>>,
    ocr_client: Arc<Client<OCR>>,
    _subscription: Arc<SubscriptionState<LidarPose, Arc<WorkerState<LidarPose>>>>,
}

impl From<LidarPose> for Pos {
    fn from(lidar_pose: LidarPose) -> Self {
        Pos {
            translation: CoordUnit(lidar_pose.x, lidar_pose.y, lidar_pose.z),
            rotation: CoordUnit(lidar_pose.roll, lidar_pose.pitch, lidar_pose.yaw),
        }
    }
}

impl CoordUnit {
    fn cal_distance(&self, other: &CoordUnit) -> f64 {
        let dx = self.0 - other.0;
        let dy = self.1 - other.1;
        let dz = self.2 - other.2;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// 计算角度距离，考虑2π周期性
    fn cal_angular_distance(&self, other: &CoordUnit) -> f64 {
        let normalize_angle = |angle: f64| {
            let mut a = angle % (2.0 * std::f64::consts::PI);
            if a > std::f64::consts::PI {
                a -= 2.0 * std::f64::consts::PI;
            } else if a < -std::f64::consts::PI {
                a += 2.0 * std::f64::consts::PI;
            }
            a
        };

        let d_roll = normalize_angle(self.0 - other.0).abs();
        let d_pitch = normalize_angle(self.1 - other.1).abs();
        let d_yaw = normalize_angle(self.2 - other.2).abs();

        (d_roll * d_roll + d_pitch * d_pitch + d_yaw * d_yaw).sqrt()
    }
}

impl Navi {
    fn new() -> Self {
        Navi::default()
    }

    /// 设置一系列目标点
    fn set_destinations(&mut self, destinations: Vec<Pos>, threshold: f64) {
        if !destinations.is_empty() {
            self.dest_pos = destinations;
            self.arrive_threshold = threshold;
            self.is_arrived = false;
            self.current_waypoint_index = 0;
            log_info!(
                "set destinations",
                "set {} waypoints, threshold: {:.2}",
                self.dest_pos.len(),
                threshold
            );
        } else {
            log_info!("set destinations", "empty destination list provided");
        }
    }

    /// 获取当前目标点
    fn get_current_destination(&self) -> Option<&Pos> {
        self.dest_pos.get(self.current_waypoint_index)
    }

    /// 更新位置并检查是否到达当前目标点
    fn update(&mut self, current_pos: Pos) -> Option<bool> {
        self.current_pos = current_pos;

        // 检查是否还有目标点
        let dest_pos = match self.get_current_destination() {
            Some(pos) => pos,
            None => {
                if self.dest_pos.is_empty() {
                    log_info!("navi update", "no destinations set");
                } else {
                    log_info!("navi update", "all waypoints reached");
                    self.is_arrived = true;
                }
                return None;
            }
        };

        let translation_distance = dest_pos
            .translation
            .cal_distance(&self.current_pos.translation);
        let rotation_distance = dest_pos
            .rotation
            .cal_angular_distance(&self.current_pos.rotation);
        let distance = translation_distance + rotation_distance;

        if distance <= self.arrive_threshold {
            self.current_waypoint_index += 1;

            if self.current_waypoint_index >= self.dest_pos.len() {
                // 所有点都到达了
                self.is_arrived = true;
                log_info!(
                    "pos update",
                    "reached final waypoint! Total: {}",
                    self.dest_pos.len()
                );
                return Some(true);
            } else {
                // 到达当前点，继续下一个
                log_info!(
                    "pos update",
                    "reached waypoint {}/{}, moving to next",
                    self.current_waypoint_index,
                    self.dest_pos.len()
                );
                return Some(false);
            }
        } else {
            log_info!(
                "pos update",
                "waypoint {}/{}, distance: {:.2}m (trans: {:.2}, rot: {:.2})",
                self.current_waypoint_index + 1,
                self.dest_pos.len(),
                distance,
                translation_distance,
                rotation_distance
            );
            return Some(false);
        }
    }

    /// 清除所有目标点
    pub fn clear_destinations(&mut self) {
        self.dest_pos.clear();
        self.current_waypoint_index = 0;
        self.is_arrived = false;
        log_info!("clear destinations", "all destinations cleared");
    }

    /// 获取剩余路径点数量
    pub fn remaining_waypoints(&self) -> usize {
        if self.dest_pos.is_empty() {
            0
        } else {
            self.dest_pos
                .len()
                .saturating_sub(self.current_waypoint_index)
        }
    }
}

impl NaviSubNode {
    pub fn new(executor: &rclrs::Executor, name: &str, topic: &str) -> anyhow::Result<Self> {
        let node = executor.create_node(name)?;
        let worker = node.create_worker(LidarPose::default());

        let goal_publisher = node.create_publisher::<PoseStamped>("/goal")?;
        let goal_pub_clone = goal_publisher.clone();

        let yolo_client = node.create_client::<YOLO>("yolo_trigger")?;
        let ocr_client = node.create_client::<OCR>("ocr_trigger")?;

        let navi_instance = Arc::new(Mutex::new(Navi::new()));
        let navi_instance_clone = Arc::clone(&navi_instance);

        let _subscription =
            worker.create_subscription::<LidarPose, _>(topic, move |msg: LidarPose| {
                log_info!("navi worker", "received lidar pos: {:?}", msg);
                if let Ok(mut navi) = navi_instance_clone.lock() {
                    match navi.update(msg.into()) {
                        Some(true) => {
                            log_info!("navi worker", "navigation completed!");
                        }
                        Some(false) => {
                            // 继续导航，发布当前目标点
                            let _ = NaviSubNode::publish_goal_with(&goal_pub_clone, &*navi);
                        }
                        None => {
                            // 没有目标点
                        }
                    }
                };
            })?;

        Ok(NaviSubNode {
            navi_instance,
            goal_publisher,
            yolo_client: Arc::new(yolo_client),
            ocr_client: Arc::new(ocr_client),
            _subscription,
        })
    }

    /// 阻塞调用 YOLO 服务
    pub fn call_yolo_blocking(&self, timeout: Duration) -> anyhow::Result<YOLO_Response> {
        log_info!("navi", "Calling YOLO service (blocking)...");

        let request = YOLO_Request::default();

        // 发送一次请求，然后阻塞等待响应
        let mut promise = self
            .yolo_client
            .call(&request)
            .map_err(|e| anyhow::anyhow!("YOLO service call failed: {:?}", e))?;

        let time_start = Instant::now();
        let response: YOLO_Response = loop {
            if time_start.elapsed() > timeout {
                return Err(anyhow::anyhow!(
                    "YOLO service call timeout after {:?}",
                    timeout
                ));
            }

            let maybe = promise
                .try_recv()
                .map_err(|e| anyhow::anyhow!("Receiving YOLO service response error: {:?}", e))?;
            match maybe {
                Some(r) => break r,
                None => {
                    std::thread::sleep(Duration::from_millis(10));
                    continue;
                }
            }
        };

        log_info!(
            "navi",
            "YOLO response - success: {}, message: {}",
            response.success,
            response.message
        );
        return Ok(response);
    }

    /// 阻塞调用 OCR 服务
    pub fn call_ocr_blocking(&self, timeout: Duration) -> anyhow::Result<OCR_Response> {
        log_info!("navi", "Calling OCR service (blocking)...");

        let request = OCR_Request::default();

        // 使用阻塞调用：等待 Promise 完成并获取响应
        let mut promise = self
            .ocr_client
            .call(&request)
            .map_err(|e| anyhow::anyhow!("OCR service call failed: {:?}", e))?;

        let time_start = Instant::now();
        let response: OCR_Response = loop {
            if time_start.elapsed() > timeout {
                return Err(anyhow::anyhow!(
                    "OCR service call timeout after {:?}",
                    timeout
                ));
            }

            let maybe = promise
                .try_recv()
                .map_err(|e| anyhow::anyhow!("Receiving OCR service response error: {:?}", e))?;
            match maybe {
                Some(r) => break r,
                None => {
                    std::thread::sleep(Duration::from_millis(10));
                    continue;
                }
            }
        };

        log_info!(
            "navi",
            "OCR response - success: {}, message: {}",
            response.success,
            response.message
        );
        Ok(response)
    }

    fn publish_goal_with(publisher: &Publisher<PoseStamped>, navi: &Navi) -> anyhow::Result<()> {
        let dest = match navi.get_current_destination() {
            Some(dest) => dest,
            None => {
                log_info!("navi", "No destination set; skipping publish");
                return Ok(());
            }
        };

        let x = dest.translation.0;
        let y = dest.translation.1;
        let z = dest.translation.2;
        let yaw = dest.rotation.2;

        let mut goal_msg = PoseStamped::default();
        goal_msg.header.frame_id = "odom".to_string();
        goal_msg.pose.position.x = x;
        goal_msg.pose.position.y = y;
        goal_msg.pose.position.z = z;

        // 将yaw转换为四元数
        let qw = (yaw / 2.0).cos();
        let qz = (yaw / 2.0).sin();
        goal_msg.pose.orientation.w = qw;
        goal_msg.pose.orientation.z = qz;

        publisher.publish(goal_msg)?;
        log_info!(
            "navi",
            "Published goal {}/{}: ({:.2}, {:.2}, yaw: {:.2})",
            navi.current_waypoint_index + 1,
            navi.dest_pos.len(),
            x,
            y,
            yaw
        );

        Ok(())
    }

    pub fn publish_goal(&self) -> anyhow::Result<()> {
        if let Ok(navi) = self.navi_instance.lock() {
            Self::publish_goal_with(&self.goal_publisher, &*navi)
        } else {
            Err(anyhow::anyhow!("Failed to acquire navigation lock"))
        }
    }

    /// 设置导航目标点列表
    pub fn set_destinations(&self, destinations: Vec<Pos>, threshold: f64) -> anyhow::Result<()> {
        if let Ok(mut navi) = self.navi_instance.lock() {
            navi.set_destinations(destinations, threshold);
            // 立即发布第一个目标点
            Self::publish_goal_with(&self.goal_publisher, &*navi)?;
            Ok(())
        } else {
            Err(anyhow::anyhow!("Failed to acquire navigation lock"))
        }
    }

    /// 是否完成所有导航
    pub fn is_arrived(&self) -> bool {
        self.navi_instance
            .lock()
            .map(|navi| navi.is_arrived)
            .unwrap_or(false)
    }

    /// 获取当前位置
    pub fn get_current_position(&self) -> Option<Pos> {
        self.navi_instance.lock().ok().map(|navi| navi.current_pos)
    }

    /// 获取剩余路径点数量
    pub fn remaining_waypoints(&self) -> usize {
        self.navi_instance
            .lock()
            .map(|navi| navi.remaining_waypoints())
            .unwrap_or(0)
    }

    /// 清除所有目标点
    pub fn clear_destinations(&self) -> anyhow::Result<()> {
        if let Ok(mut navi) = self.navi_instance.lock() {
            navi.clear_destinations();
            Ok(())
        } else {
            Err(anyhow::anyhow!("Failed to acquire navigation lock"))
        }
    }
}
