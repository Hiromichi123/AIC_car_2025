use std::thread::sleep;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use rclrs::log_info;

use crate::navi::NaviSubNode;

#[derive(Debug, Clone)]
pub struct VisionResult {
    pub timestamp: SystemTime,
    pub good_people: Option<u32>,
    pub bad_people: Option<u32>,
    pub fire: Option<u32>,
    pub ocr: Vec<String>,
    pub automobile: Option<u32>,
}

#[derive(Debug, Clone, Copy)]
pub enum VisionCategory {
    GoodPeople,
    BadPeople,
    Fire,
    Automobile,
}

#[derive(Debug, Clone)]
pub enum VisionUpdate {
    Count {
        category: VisionCategory,
        count: Option<u32>,
    },
    Ocr(Vec<String>),
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
            timestamp: SystemTime::now(),
            good_people: None,
            bad_people: None,
            fire: None,
            ocr: Vec::new(),
            automobile: None,
        }
    }
}

impl VisionResult {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(&mut self, update: VisionUpdate) {
        match update {
            VisionUpdate::Count { category, count } => match category {
                VisionCategory::GoodPeople => merge_count(&mut self.good_people, count),
                VisionCategory::BadPeople => merge_count(&mut self.bad_people, count),
                VisionCategory::Fire => merge_count(&mut self.fire, count),
                VisionCategory::Automobile => merge_count(&mut self.automobile, count),
            },
            VisionUpdate::Ocr(entries) => {
                for entry in entries {
                    if !entry.is_empty() {
                        self.ocr.push(entry);
                    }
                }
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
            "ts={} good_people={} bad_people={} fire={} automobile={} ocr={:?}",
            ts,
            fmt_count(self.good_people),
            fmt_count(self.bad_people),
            fmt_count(self.fire),
            fmt_count(self.automobile),
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
    camera: Option<&str>,
) -> bool {
    log_info!("vision", "Waiting for green light...");

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
                    log_info!("vision", "Green light detected! Proceeding...");
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
            log_info!("vision", "{} -> {}", request.label, response.message);

            if let Some(category) = request.category {
                let count = extract_first_number(&response.message);
                result.push(VisionUpdate::Count { category, count });
            }
        }
        Err(err) => {
            log_info!("vision", "{} failed: {:?}", request.label, err);
        }
    }
}

pub fn run_ocr_detection(
    node: &NaviSubNode,
    executor: &mut rclrs::Executor,
    timeout: Duration,
    description: &str,
    result: &mut VisionResult,
) {
    log_info!("vision", "{}", description);

    match node.call_ocr_blocking(executor, timeout) {
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

fn extract_first_number(message: &str) -> Option<u32> {
    let mut digits = String::new();
    for ch in message.chars() {
        if ch.is_ascii_digit() {
            digits.push(ch);
        } else if !digits.is_empty() {
            break;
        }
    }

    if digits.is_empty() {
        None
    } else {
        digits.parse().ok()
    }
}

fn parse_ocr_tokens(raw: &str) -> Vec<String> {
    raw.split(|c| c == ',' || c == '\n')
        .map(|token| token.trim().to_string())
        .filter(|token| !token.is_empty())
        .collect()
}
