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
    Rubbish {
        category: String,
        item: String,
        confidence: Option<f32>,
        bin_state: Option<String>,
        placement: Option<String>,
    },
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
            ebike_upright: None,
            ebike_fallen: None,
            rubbish: None,
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
                VisionCategory::EbikeUpright => merge_count(&mut self.ebike_upright, count),
                VisionCategory::EbikeFallen => merge_count(&mut self.ebike_fallen, count),
            },
            VisionUpdate::Ocr(entries) => {
                for entry in entries {
                    if !entry.is_empty() {
                        self.ocr.push(entry);
                    }
                }
            }
            VisionUpdate::Rubbish {
                category,
                item,
                confidence,
                bin_state,
                placement,
            } => {
                let bin_state = bin_state.unwrap_or_else(|| "关闭".to_string());
                let placement = placement.unwrap_or_else(|| "错误".to_string());

                // Keep confidence in logs only; TTS uses a fixed format.
                let _ = confidence;

                let text = format!(
                    "{}垃圾桶状态为{}，垃圾桶里的垃圾为{}，投放{}",
                    category, bin_state, item, placement
                );
                self.rubbish = Some(text);
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

            if let Some(parsed) = parse_rubbish_result(&response.message) {
                result.push(VisionUpdate::Rubbish {
                    category: parsed.category,
                    item: parsed.item,
                    confidence: parsed.confidence,
                    bin_state: parsed.bin_state,
                    placement: parsed.placement,
                });
                return;
            }

            // YOLO nodes in this repo tend to return comma-separated labels, e.g.
            // "非社区人员, 社区人员" or "火灾, 火灾".
            let labels = parse_yolo_labels(&response.message);

            // e-bike model in this repo may return: "停放正确" and/or "倒伏".
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
                        category: VisionCategory::EbikeUpright,
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
                    result.push(VisionUpdate::Count {
                        category: VisionCategory::Automobile,
                        count: Some(total),
                    });
                }
                return;
            }

            // People model: count both good/bad from the same response.
            if request.model == Some("people")
                || labels.iter().any(|l| l == "社区人员" || l == "非社区人员")
            {
                let good = count_any(&labels, &["社区人员"]);
                let bad = count_any(&labels, &["非社区人员"]);
                if good > 0 {
                    result.push(VisionUpdate::Count {
                        category: VisionCategory::GoodPeople,
                        count: Some(good),
                    });
                }
                if bad > 0 {
                    result.push(VisionUpdate::Count {
                        category: VisionCategory::BadPeople,
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
    executor: &mut rclrs::Executor,
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
        .filter(|label| targets.iter().any(|t| label.as_str() == *t))
        .count() as u32
}

fn parse_ocr_tokens(raw: &str) -> Vec<String> {
    raw.split(|c| c == ',' || c == '\n')
        .map(|token| token.trim().to_string())
        .filter(|token| !token.is_empty())
        .collect()
}

struct RubbishParsed {
    category: String,
    item: String,
    confidence: Option<f32>,
    bin_state: Option<String>,
    placement: Option<String>,
}

fn parse_rubbish_result(message: &str) -> Option<RubbishParsed> {
    // Example:
    // "垃圾分类完成: [其它垃圾] 扫把 (置信度: 0.99). 垃圾桶顺序: 有害闭, 可回收闭, 其他闭, 厨余闭"
    if !message.contains("垃圾分类完成") {
        return None;
    }

    let left = message.find('[')?;
    let right = message[left + 1..].find(']')? + left + 1;
    let raw_category = message[left + 1..right].trim();
    let category = normalize_rubbish_category(raw_category)?;

    let after = message[right + 1..].trim_start();
    let item_end = after
        .find('(')
        .or_else(|| after.find('.'))
        .unwrap_or(after.len());
    let item = after[..item_end].trim().to_string();

    let confidence = if let Some(idx) = message.find("置信度") {
        let tail = &message[idx..];
        let colon = tail.find(':').or_else(|| tail.find('：'))?;
        let num_tail = tail[colon + 1..].trim();
        let end = num_tail
            .find(|c: char| !(c.is_ascii_digit() || c == '.'))
            .unwrap_or(num_tail.len());
        num_tail[..end].trim().parse::<f32>().ok()
    } else {
        None
    };

    if category.is_empty() || item.is_empty() {
        return None;
    }

    let bin_state = parse_bin_state_for_category(message, &category);

    let placement = if message.contains("投放正确") {
        Some("正确".to_string())
    } else if message.contains("投放错误") {
        Some("错误".to_string())
    } else {
        // Best-effort inference: if the target bin is open, consider placement correct.
        match bin_state.as_deref() {
            Some("打开") => Some("正确".to_string()),
            Some("关闭") => Some("错误".to_string()),
            _ => None,
        }
    };

    Some(RubbishParsed {
        category,
        item,
        confidence,
        bin_state,
        placement,
    })
}

fn normalize_rubbish_category(raw: &str) -> Option<String> {
    // Normalize to: 可回收 / 其他 / 有害 / 厨余
    let s = raw.trim();
    if s.contains("可回收") {
        return Some("可回收".to_string());
    }
    if s.contains("有害") {
        return Some("有害".to_string());
    }
    if s.contains("厨余") {
        return Some("厨余".to_string());
    }
    if s.contains("其它") || s.contains("其他") {
        return Some("其他".to_string());
    }
    None
}

fn parse_bin_state_for_category(message: &str, category: &str) -> Option<String> {
    // Parse tail like: "垃圾桶顺序: 有害闭, 可回收开, 其他闭, 厨余闭"
    let idx = message.find("垃圾桶顺序")?;
    let tail = &message[idx..];
    let colon = tail.find(':').or_else(|| tail.find('：'))?;
    let list = tail[colon + 1..].trim();

    for token in list.split(|c| c == ',' || c == '，' || c == '\n') {
        let t = token.trim();
        if t.is_empty() {
            continue;
        }

        // Expected: "有害闭" / "可回收开" / "其他闭" / "厨余开"
        let (name, state_char) = match t.chars().last() {
            Some(last) if last == '开' || last == '闭' => {
                (&t[..t.len().saturating_sub(last.len_utf8())], last)
            }
            _ => continue,
        };

        let name = name.trim();
        let name_norm = if name.contains("可回收") {
            "可回收"
        } else if name.contains("有害") {
            "有害"
        } else if name.contains("厨余") {
            "厨余"
        } else if name.contains("其它") || name.contains("其他") {
            "其他"
        } else {
            continue;
        };

        if name_norm == category {
            return Some(if state_char == '开' {
                "打开".to_string()
            } else {
                "关闭".to_string()
            });
        }
    }

    None
}
