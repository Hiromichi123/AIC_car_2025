use geometry_msgs::msg::PoseStamped;
use rclrs::{log_info, Publisher};
use rclrs::{SubscriptionState, WorkerState};
use ros2_tools::msg::LidarPose;
use std::sync::Arc;
use std::sync::Mutex;

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
}

pub struct NaviSubNode {
    #[allow(unused)]
    pub navi_instance: Arc<Mutex<Navi>>,
    goal_publisher: Publisher<PoseStamped>,
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
}

impl Navi {
    fn new() -> Self {
        Navi::default()
    }

    pub fn set_dest(&mut self, dest: &Pos, threshold: f64) {
        match self.dest_pos {
            None => {
                self.dest_pos.push(*dest);
                self.arrive_threshold = threshold;
                self.is_arrived = false;
                log_info!("set dest", "destination set to {:?}", dest);
            }
            Some(_) => {
                log_info!("set dest", "car not arrived!");
            }
        }
    }

    fn update(&mut self, current_pos: Pos) -> Option<bool> {
        self.current_pos = current_pos;

        let dest_pos = match &self.dest_pos {
            Some(pos) => pos,
            None => {
                log_info!("navi update", "dest not set");
                return None;
            }
        };

        let distance = dest_pos
            .translation
            .cal_distance(&self.current_pos.translation);
        if !self.is_arrived && distance <= self.arrive_threshold {
            self.is_arrived = true;
            self.dest_pos = None; // for moving interval check
            log_info!("pos update", "car arrived!");
            return Some(true);
        } else {
            log_info!("pos update", "dest distance: {distance}");
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

        let navi_instance = Arc::new(Mutex::new(Navi::new()));
        let navi_instance_clone = Arc::clone(&navi_instance);

        let _subscription =
            worker.create_subscription::<LidarPose, _>(topic, move |msg: LidarPose| {
                log_info!("navi worker", "received lidar pos: {:?}", msg);
                if let Ok(mut navi) = navi_instance_clone.lock() {
                    if navi.update(msg.into()) == Some(false) {
                        NaviSubNode::publish_goal_with(&goal_pub_clone, &*navi);
                    };
                };
            })?;

        Ok(NaviSubNode {
            navi_instance,
            goal_publisher,
            _subscription,
        })
    }

    fn publish_goal_with(publisher: &Publisher<PoseStamped>, navi: &Navi) -> anyhow::Result<()> {
        let dest = match navi.dest_pos {
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
            "Published goal: ({:.2}, {:.2}, yaw: {:.2})",
            x,
            y,
            yaw
        );

        Ok(())
    }

    pub fn publish_goal(&self, navi: &Navi) -> anyhow::Result<()> {
        let dest = match navi.dest_pos {
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

        self.goal_publisher.publish(goal_msg)?;
        log_info!(
            "navi",
            "Published goal: ({:.2}, {:.2}, yaw: {:.2})",
            x,
            y,
            yaw
        );

        Ok(())
    }

    pub fn set_destination(&self, dest: Vec<Pos>, threshold: f64) -> anyhow::Result<()> {
        if let Ok(mut navi) = self.navi_instance.lock() {
            let mut dest_iter = dest.iter();
            while let Some(dest_point) = dest_iter.next() {
                navi.set_dest(dest_point, threshold);
            }
            Ok(())
        } else {
            Err(anyhow::anyhow!("Failed to acquire navigation lock"))
        }
    }

    pub fn is_arrived(&self) -> bool {
        self.navi_instance
            .lock()
            .map(|navi| navi.is_arrived)
            .unwrap_or(false)
    }

    pub fn get_current_position(&self) -> Option<Pos> {
        self.navi_instance.lock().ok().map(|navi| navi.current_pos)
    }
}
