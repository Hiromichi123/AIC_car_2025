use rclrs::log_info;
use rclrs::{SubscriptionState, WorkerState};
use ros2_tools::msg::LidarPose;
use std::sync::Arc;
use std::sync::Mutex;

#[derive(Debug, Default, Clone, Copy)]
struct CoordUnit(f64, f64, f64);

#[derive(Debug, Default, Clone, Copy)]
pub struct Pos {
    translation: CoordUnit, // (x, y, z)
    rotation: CoordUnit,    // (roll, pitch, yaw)
}

#[derive(Debug, Default)]
pub struct Navi {
    dest_pos: Option<Pos>,
    current_pos: Pos,
    velocity: Pos,
    is_arrived: bool,
    arrive_threshold: f64,
}

pub struct NaviSubNode {
    #[allow(unused)]
    navi_instance: Arc<Mutex<Navi>>,
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

    fn set_dest(&mut self, dest: Pos, threshold: f64) {
        self.dest_pos = Some(dest);
        self.arrive_threshold = threshold;
        log_info!("set dest", "destination set to {:?}", dest);
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

        let navi_instance = Arc::new(Mutex::new(Navi::new()));

        let navi_instance_clone = Arc::clone(&navi_instance);
        let _subscription =
            worker.create_subscription::<LidarPose, _>(topic, move |msg: LidarPose| {
                log_info!("navi worker", "received lidar pos: {:?}", msg);
                let mut navi = navi_instance_clone.lock().unwrap();
                navi.update(msg.into());
            })?;

        Ok(NaviSubNode {
            navi_instance,
            _subscription,
        })
    }
}
