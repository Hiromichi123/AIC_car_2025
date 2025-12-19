use std::thread:: sleep;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use rclrs::log_info;

use crate::navi::NaviSubNode;

#[derive(Debug, Clone)]
pub struct VisionResult {
    pub timestamp: SystemTime,
    pub good_people: Option<u32>,
    pub bad_people: Option<u32>,
    pub fire:  Option<u32>,
    pub ocr: Vec<String>,
    pub automobile: Option<u32>,
    pub ebike_upright: Option<u32>,
    pub ebike_fallen: Option<u32>,
    pub rubbish: Option<String>,
}

#[derive(Debug, Clone, Copy)]
pub enum VisionCategory {
    GoodPeople,
    BadPeople,
    Fire,
    Automobile,
    EbikeUpright,
    EbikeFallen,
}

#[derive(Debug, Clone)]
pub enum VisionUpdate {
    Count {
        category: VisionCategory,
        count: Option<u32>,
    },
    Ocr(Vec<String>),
    Rubbish(String),  // 简化：直接存储完整消息
}

#[derive(Debug, Clone, Copy)]
pub struct VisionRequest<'a> {
    pub label: &'a str,
    pub model: Option<&'a str>,
    pub camera: Option<&'a str>,
    pub timeout: Duration,
    pub category: Option<VisionCategory>,
}

impl Default for VisionResult {
    fn default() -> Self {
        Self {
            timestamp: SystemTime:: now(),
            good_people:  None,
            bad_people:  None,
            fire: None,
            ocr: Vec::new(),
            automobile: None,
            ebike_upright: None,
            ebike_fallen:  None,
            rubbish:  None,
        }
    }
}

impl VisionResult {
    pub fn new() -> Self {
        Self:: default()
    }

    pub fn push(&mut self, update: VisionUpdate) {
        match update {
            VisionUpdate::Count { category, count } => match category {
                VisionCategory:: GoodPeople => merge_count(&mut self.good_people, count),
                VisionCategory:: BadPeople => merge_count(&mut self.bad_people, count),
                VisionCategory::Fire => merge_count(&mut self. fire, count),
                VisionCategory::Automobile => merge_count(&mut self.automobile, count),
                VisionCategory::EbikeUpright => merge_count(&mut self.ebike_upright, count),
                VisionCategory::EbikeFallen => merge_count(&mut self.ebike_fallen, count),
            },
            VisionUpdate:: Ocr(entries) => {
                for entry in entries {
                    if ! entry.is_empty() {
                        self.ocr.push(entry);
                    }
                }
            }
            VisionUpdate::Rubbish(message) => {
                // 直接存储原始消息，不做任何处理
                self.rubbish = Some(message);
            }
        }
    }

    pub fn summary(&self) -> String {
        let ts = self
            .timestamp
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or_default();

        format!(
            "ts={} good_people={} bad_people={} fire={} automobile={} ebike_upright={} ebike_fallen={} rubbish={} ocr={:?}",
            ts,
            fmt_count(self.good_people),
            fmt_count(self.bad_people),
            fmt_count(self.fire),
            fmt_count(self.automobile),
            fmt_count(self.ebike_upright),
            fmt_count(self.ebike_fallen),
            self.rubbish.as_deref().unwrap_or("N/A"),
            self.ocr
        )
    }
}

fn fmt_count(count: Option<u32>) -> String {
    count
        .map(|value| value.to_string())
        .unwrap_or_else(|| "N/A".to_string())
}

fn merge_count(target: &mut Option<u32>, delta: Option<u32>) {
    if let Some(value) = delta {
        let current = target.unwrap_or(0);
        *target = Some(current + value);
    }
}

pub fn wait_for_green_light(
    node: &NaviSubNode,
    executor: &mut rclrs::Executor,
    timeout: Duration,
    max_attempts: u32,
    camera:  Option<&str>,
) -> bool {
    log_info!("vision", "Waiting for green light.. .");

    for attempt in 1..=max_attempts {
        match node.call_yolo_blocking(executor, Some("traffic_light"), camera, timeout) {
            Ok(response) => {
                log_info!(
                    "vision",
                    "Traffic light attempt {}: {}",
                    attempt,
                    response.message
                );

                // 检查 success 标志或 status 为 green_detected
                if response.success || response.message.contains("green_detected") {
                    log_info! ("vision", "Green light detected!  Proceeding.. .");
                    return true;
                }
            }
            Err(err) => {
                log_info!(
                    "vision",
                    "YOLO traffic_light attempt {} failed: {:?}",
                    attempt,
                    err
                );
            }
        }

        sleep(Duration::from_millis(500));
    }

    log_info!(
        "vision",
        "Green light not detected after {} attempts.",
        max_attempts
    );
    false
}

pub fn run_yolo_detection(
    node: &NaviSubNode,
    executor: &mut rclrs::Executor,
    request: VisionRequest<'_>,
    result: &mut VisionResult,
) {
    log_info!("vision", "{}", request.label);

    match node.call_yolo_blocking(executor, request.model, request.camera, request.timeout) {
        Ok(response) => {
            log_info!("vision", "{} -> {}", request.label, response. message);

            // 简化：如果是垃圾分类，直接返回完整消息
            if request.model == Some("rubbish_bin") {
                result.push(VisionUpdate::Rubbish(response.message. clone()));
                return;
            }

            // YOLO nodes in this repo tend to return comma-separated labels, e.g. 
            // "非社区人员, 社区人员" or "火灾, 火灾". 
            let labels = parse_yolo_labels(&response. message);

            // e-bike model in this repo may return:  "停放正确" and/or "倒伏". 
            if request.model == Some("e_bike")
                || labels
                    .iter()
                    .any(|l| l == "倒伏" || l == "停放正确" || l == "非倒伏")
            {
                let upright = count_any(&labels, &["停放正确", "非倒伏"]);
                let fallen = count_any(&labels, &["倒伏"]);
                let total = upright + fallen;

                if upright > 0 {
                    result.push(VisionUpdate::Count {
                        category:  VisionCategory::EbikeUpright,
                        count: Some(upright),
                    });
                }
                if fallen > 0 {
                    result.push(VisionUpdate::Count {
                        category: VisionCategory::EbikeFallen,
                        count: Some(fallen),
                    });
                }
                if total > 0 {
                    result.push(VisionUpdate:: Count {
                        category: VisionCategory:: Automobile,
                        count: Some(total),
                    });
                }
                return;
            }

            // People model:  count both good/bad from the same response. 
            if request.model == Some("people")
                || labels. iter().any(|l| l == "社区人员" || l == "非社区人员")
            {
                let good = count_any(&labels, &["社区人员"]);
                let bad = count_any(&labels, &["非社区人员"]);
                if good > 0 {
                    result.push(VisionUpdate::Count {
                        category:  VisionCategory::GoodPeople,
                        count: Some(good),
                    });
                }
                if bad > 0 {
                    result.push(VisionUpdate::Count {
                        category: VisionCategory:: BadPeople,
                        count: Some(bad),
                    });
                }
                return;
            }

            if let Some(category) = request.category {
                let count = match category {
                    VisionCategory::GoodPeople => count_any(&labels, &["社区人员"]),
                    VisionCategory::BadPeople => count_any(&labels, &["非社区人员"]),
                    VisionCategory::Fire => count_any(&labels, &["火灾"]),
                    VisionCategory::Automobile => {
                        count_any(&labels, &["电动车", "电瓶车", "e-bike", "e_bike", "ebike"])
                    }
                    VisionCategory::EbikeUpright => count_any(&labels, &["停放正确", "非倒伏"]),
                    VisionCategory::EbikeFallen => count_any(&labels, &["倒伏"]),
                };

                if count > 0 {
                    result.push(VisionUpdate::Count {
                        category,
                        count: Some(count),
                    });
                }
            }
        }
        Err(err) => {
            log_info!("vision", "{} failed: {:?}", request.label, err);
        }
    }
}

pub fn run_ocr_detection(
    node: &NaviSubNode,
    executor:  &mut rclrs::Executor,
    camera: Option<&str>,
    timeout: Duration,
    description: &str,
    result: &mut VisionResult,
) {
    log_info!("vision", "{}", description);

    match node.call_ocr_blocking(executor, camera, timeout) {
        Ok(response) => {
            log_info!("vision", "{} -> {}", description, response.message);
            let tokens = parse_ocr_tokens(&response.message);
            if !tokens.is_empty() {
                result.push(VisionUpdate::Ocr(tokens));
            }
        }
        Err(err) => {
            log_info!("vision", "{} failed: {:?}", description, err);
        }
    }
}

fn parse_yolo_labels(message: &str) -> Vec<String> {
    // Support both ',' and '，' plus newlines. 
    message
        .split(|c| c == ',' || c == '，' || c == '\n')
        .map(|token| token.trim().to_string())
        .filter(|token| !token.is_empty())
        .collect()
}

fn count_any(labels: &[String], targets: &[&str]) -> u32 {
    labels
        .iter()
        .filter(|label| targets.iter().any(|t| label. as_str() == *t))
        .count() as u32
}

fn parse_ocr_tokens(raw: &str) -> Vec<String> {
    raw.split(|c| c == ',' || c == '\n')
        .map(|token| token.trim().to_string())
        .filter(|token| !token.is_empty())
        .collect()
}