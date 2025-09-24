use rclrs::{log_debug, log_error, log_info};
use std::sync::Arc;
use std_msgs::msg::Float64;
use std_msgs::msg::String as StringMsg;

#[derive(Debug, Default)]
struct CoordUnit(f64, f64, f64);

#[derive(Debug, Default)]
struct Pos {
    translation: CoordUnit, // (x, y, z)
    rotation: CoordUnit,    // (roll, pitch, yaw)
}

struct Navi {
    dest_pos: Pos,
    current_pos: Pos,
    velocity: Pos,
    is_arrived: bool,
    arrive_threshold: f64,
}

struct NaviSubData {
    node: rclrs::Node,
    current_pos: Pos,
}

pub struct NaviSubNode {
    subscription: rclrs::WorkerSubscription<StringMsg, NaviSubData>,
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
    fn new(dest_pos: Pos, threshold: f64) -> Self {
        Navi {
            dest_pos,
            current_pos: Pos::default(),
            velocity: Pos::default(),
            is_arrived: false,
            arrive_threshold: threshold,
        }
    }

    fn update(&mut self, current_pos: Pos) -> bool {
        self.current_pos = current_pos;
        if !self.is_arrived
            && self
                .dest_pos
                .translation
                .cal_distance(&self.current_pos.translation)
                <= self.arrive_threshold
        {
            self.is_arrived = true;
            return true;
        }

        false
    }
}

impl NaviSubNode {
    pub fn new(
        executor: &rclrs::Executor,
        name: &str,
        topic: &str,
    ) -> Result<Self, rclrs::RclrsError> {
        let node = executor.create_node(name)?;
        let worker = node.create_worker::<NaviSubData>(NaviSubData {
            node: Arc::clone(&node),
            current_pos: Pos::default(),
        });

        let subscription =
            worker.create_subscription(topic, |data: &mut NaviSubData, msg: StringMsg| todo!())?;

        Ok(NaviSubNode { subscription })
    }
}
