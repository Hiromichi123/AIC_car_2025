use geometry_msgs::msg::PoseStamped;
use rclrs::{log_info, Client, Publisher};
use rclrs::{SubscriptionState, WorkerState};
use ros2_tools::msg::LidarPose;
use ros2_tools::srv::{
    OCR_Request, OCR_Response, SERVO_Request, SERVO_Response, TTS_Request, TTS_Response,
    YOLO_Request, YOLO_Response, OCR, SERVO, TTS, YOLO,
};
use std::sync::Arc;
use std::sync::Mutex;
use std::thread::sleep;
use std::time::{Duration, Instant};

macro_rules! spin_until_response {
    ($executor:expr, $promise:expr, $timeout:expr, $svc:literal) => {{
        let time_start = Instant::now();
        loop {
            if time_start.elapsed() > $timeout {
                break Err(anyhow::anyhow!(
                    concat!($svc, " service call timeout after {:?}"),
                    $timeout
                ));
            }

            let mut spin_options = rclrs::SpinOptions::default();
            spin_options.timeout = Some(Duration::from_millis(10));
            spin_options.only_next_available_work = true;
            $executor.spin(spin_options);

            match $promise.try_recv() {
                Ok(Some(r)) => break Ok(r),
                Ok(None) => continue,
                Err(e) => {
                    break Err(anyhow::anyhow!(
                        concat!($svc, " service call failed: {:?}"),
                        e
                    ));
                }
            }
        }
    }};
}

#[derive(Debug, Default, Clone, Copy)]
pub struct CoordUnit(pub f64, pub f64, pub f64);

#[derive(Debug, Default, Clone, Copy)]
pub struct Pos {
    pub translation: CoordUnit, // (x, y, z)
    pub rotation: CoordUnit,    // (roll, pitch, yaw)
}

pub enum ServoState {
    RIGHT,
    CENTER,
    LEFT,
}

#[derive(Debug, Default)]
pub struct Navi {
    pub dest_pos: Vec<Pos>,
    pub current_pos: Pos,
    pub is_arrived: bool,
    translation_threshold: f64,
    rotation_threshold: f64,
    current_waypoint_index: usize, // 当前目标点索引
}

pub struct NaviSubNode {
    #[allow(unused)]
    pub navi_instance: Arc<Mutex<Navi>>,
    goal_publisher: Publisher<PoseStamped>,
    tts_client: Arc<Client<TTS>>,
    yolo_client: Arc<Client<YOLO>>,
    ocr_client: Arc<Client<OCR>>,
    servo_client: Arc<Client<SERVO>>,
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
    fn set_destinations(
        &mut self,
        destinations: Vec<Pos>,
        translation_threshold: f64,
        rotation_threshold: f64,
    ) {
        if !destinations.is_empty() {
            self.dest_pos = destinations;
            self.translation_threshold = translation_threshold;
            self.rotation_threshold = rotation_threshold;
            self.is_arrived = false;
            self.current_waypoint_index = 0;
            log_info!(
                "set destinations",
                "set {} waypoints, translation_threshold: {:.2}, rotation_threshold: {:.2}",
                self.dest_pos.len(),
                translation_threshold,
                rotation_threshold
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
                    // log_info!("navi update", "no destinations set");
                } else {
                    // log_info!("navi update", "all waypoints reached");
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

        if translation_distance <= self.translation_threshold
            && rotation_distance <= self.rotation_threshold
        {
            self.current_waypoint_index += 1;

            if self.current_waypoint_index >= self.dest_pos.len() {
                // 所有点都到达了
                self.is_arrived = true;
                // log_info!(
                //     "pos update",
                //     "reached final waypoint! Total: {}",
                //     self.dest_pos.len()
                // );
                return Some(true);
            } else {
                // 到达当前点，继续下一个
                return Some(false);
            }
        } else {
            // log_info!(
            //     "pos update",
            //     "waypoint {}/{}, trans: {:.2}m (threshold: {:.2}), rot: {:.2}rad (threshold: {:.2})",
            //     self.current_waypoint_index + 1,
            //     self.dest_pos.len(),
            //     translation_distance,
            //     self.translation_threshold,
            //     rotation_distance,
            //     self.rotation_threshold
            // );
            return Some(false);
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
        let servo_client = node.create_client::<SERVO>("servo_control")?;
        let tts_client = node.create_client::<TTS>("tts_play")?;

        let navi_instance = Arc::new(Mutex::new(Navi::new()));
        let navi_instance_clone = Arc::clone(&navi_instance);

        let _subscription =
            worker.create_subscription::<LidarPose, _>(topic, move |msg: LidarPose| {
                // log_info!("navi worker", "received lidar pos: {:?}", msg);
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
            tts_client: Arc::new(tts_client),
            yolo_client: Arc::new(yolo_client),
            ocr_client: Arc::new(ocr_client),
            servo_client: Arc::new(servo_client),
            _subscription,
        })
    }

    /// 阻塞调用 YOLO 服务
    pub fn call_yolo_blocking(
        &self,
        executor: &mut rclrs::Executor,
        model: Option<&str>,
        camera: Option<&str>,
        timeout: Duration,
    ) -> anyhow::Result<YOLO_Response> {
        log_info!("navi", "Calling YOLO service (blocking)...");

        let mut request = YOLO_Request::default();
        request.model = model.unwrap_or("").to_string();
        request.camera = camera.unwrap_or("both").to_string();
        sleep(std::time::Duration::from_secs_f32(0.5));

        // 发送一次请求，然后阻塞等待响应
        log_info!("navi", "calling yolo service");
        let mut promise = self
            .yolo_client
            .call(&request)
            .map_err(|e| anyhow::anyhow!("YOLO service call failed: {:?}", e))?;
        let response: YOLO_Response = spin_until_response!(executor, promise, timeout, "YOLO")?;

        log_info!(
            "navi",
            "YOLO response - success: {}, message: {}",
            response.success,
            response.message
        );
        return Ok(response);
    }

    /// 阻塞调用 OCR 服务
    pub fn call_ocr_blocking(
        &self,
        executor: &mut rclrs::Executor,
        timeout: Duration,
    ) -> anyhow::Result<OCR_Response> {
        log_info!("navi", "Calling OCR service (blocking)...");

        let request = OCR_Request::default();
        sleep(std::time::Duration::from_secs_f32(0.5));

        // 使用阻塞调用：等待 Promise 完成并获取响应
        log_info!("navi", "calling ocr service");
        let mut promise = self
            .ocr_client
            .call(&request)
            .map_err(|e| anyhow::anyhow!("OCR service call failed: {:?}", e))?;
        let response: OCR_Response = spin_until_response!(executor, promise, timeout, "OCR")?;

        log_info!(
            "navi",
            "OCR response - success: {}, message: {}",
            response.success,
            response.message
        );
        Ok(response)
    }

    /// 阻塞调用 SERVO 服务，命令取值 {-1,0,1}
    pub fn call_servo_blocking(
        &self,
        executor: &mut rclrs::Executor,
        command: ServoState,
        timeout: Duration,
    ) -> anyhow::Result<SERVO_Response> {
        log_info!("navi", "Calling SERVO service (blocking)...");

        let mut request = SERVO_Request::default();

        request.command = match command {
            ServoState::LEFT => -1,
            ServoState::CENTER => 0,
            ServoState::RIGHT => 1,
        };

        let mut promise = self
            .servo_client
            .call(&request)
            .map_err(|e| anyhow::anyhow!("SERVO service call failed: {:?}", e))?;
        let response: SERVO_Response = spin_until_response!(executor, promise, timeout, "SERVO")?;

        log_info!(
            "navi",
            "SERVO response - success: {}, message: {}",
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
        // log_info!(
        //     "navi",
        //     "Published goal {}/{}: ({:.2}, {:.2}, yaw: {:.2})",
        //     navi.current_waypoint_index + 1,
        //     navi.dest_pos.len(),
        //     x,
        //     y,
        //     yaw
        // );

        Ok(())
    }

    /// 设置导航目标点列表
    pub fn set_destinations(
        &self,
        destinations: Vec<Pos>,
        translation_threshold: f64,
        rotation_threshold: f64,
    ) -> anyhow::Result<()> {
        if let Ok(mut navi) = self.navi_instance.lock() {
            navi.set_destinations(destinations, translation_threshold, rotation_threshold);
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

    /// 阻塞调用 TTS 服务
    pub fn call_tts_blocking(
        &self,
        executor: &mut rclrs::Executor,
        text: &str,
        timeout: Duration,
    ) -> anyhow::Result<TTS_Response> {
        log_info!("navi", "Calling TTS service (blocking)...");

        let mut request = TTS_Request::default();
        request.text = text.to_string();

        let mut promise = self
            .tts_client
            .call(&request)
            .map_err(|e| anyhow::anyhow!("TTS service call failed: {:?}", e))?;
        let response: TTS_Response = spin_until_response!(executor, promise, timeout, "TTS")?;

        log_info!(
            "navi",
            "TTS response - success: {}, message: {}",
            response.success,
            response.message
        );
        Ok(response)
    }
}
