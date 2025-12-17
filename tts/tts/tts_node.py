#!/usr/bin/env python3
import os
os.environ["HF_HUB_OFFLINE"] = "1" # 强制禁止联网

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import threading
import queue
import sounddevice as sd
import scipy.signal 

# 引入 kokoro (index-tts)
# 如果你无法直接 import kokoro，请确保该库在你的 PYTHONPATH 中
try:
    from kokoro import KPipeline
except ImportError:
    print("Error: 'kokoro' library not found. Please install it via pip or check PYTHONPATH.")
    exit(1)

class IndexTTSNode(Node):
    def __init__(self):
        super().__init__('index_tts_node')

        # 1. 声明参数 (可选配置)
        self.declare_parameter('lang_code', 'z') # 'a' for American English, 'z' for Chinese, etc.
        self.declare_parameter('voice', 'zm_yunjian') # Voice ID
        self.declare_parameter('speed', 0.9)
        self.declare_parameter('sample_rate', 24000)

        # --- 新增：音频设备参数 ---
        # 默认留空，使用系统默认设备；如果指定字符串，则尝试匹配
        self.declare_parameter('device', 'hw:2,0') 
        
        self.output_device = self.get_parameter('device').value
        
        # 如果参数不为空，尝试将其转换为整数（如果是数字索引）
        if self.output_device:
            try:
                self.output_device = int(self.output_device)
            except ValueError:
                # 如果不是数字，保留为字符串，sounddevice 会自动模糊匹配名称
                pass
            self.get_logger().info(f"Audio output device set to: {self.output_device}")
        else:
            self.output_device = None # None 表示使用系统默认

        # 获取参数
        self.lang_code = self.get_parameter('lang_code').value
        self.voice = self.get_parameter('voice').value
        self.speed = self.get_parameter('speed').value
        self.sample_rate = self.get_parameter('sample_rate').value

        self.get_logger().info(f"Initializing Index-TTS (Kokoro) with lang='{self.lang_code}'...")

        # 2. 初始化模型
        # 注意：KPipeline 初始化可能需要下载模型，建议首次运行保证网络通畅
        try:
            self.pipeline = KPipeline(lang_code=self.lang_code)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize KPipeline: {e}")
            raise e
        
        self.get_logger().info("Model loaded successfully.")

        # 3. 创建队列和工作线程
        # 使用队列是为了避免 TTS 生成阻塞 ROS 的回调循环
        self.text_queue = queue.Queue()
        self.worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker_thread.start()

        # 4. 创建订阅者
        self.subscription = self.create_subscription(
            String,
            'tts_input',  # 订阅的话题名称
            self.listener_callback,
            10
        )
        self.get_logger().info(f"Subscribed to topic: 'tts_input'")

    def listener_callback(self, msg):
        """ROS 回调：只负责接收文本并推入队列"""
        text = msg.data.strip()
        if text:
            self.get_logger().info(f"Received text: '{text}'")
            self.text_queue.put(text)

    def _worker_loop(self):
        """后台线程：负责从队列取数据、推理、播放"""
        while rclpy.ok():
            try:
                # 阻塞等待新文本，超时是为了定期检查线程是否需要退出
                text = self.text_queue.get(timeout=1.0) 
            except queue.Empty:
                continue

            try:
                self.get_logger().info(f"Synthesizing: {text}")
                
                # --- Index-TTS (Kokoro) 推理核心 ---
                # pipeline 返回一个生成器，包含 (graphemes, phonemes, audio)
                generator = self.pipeline(
                    text, 
                    voice=self.voice, 
                    speed=self.speed, 
                    split_pattern=r'\n+'
                )

                # 遍历生成器播放每一段音频
                for i, (gs, ps, audio) in enumerate(generator):
                    if audio is None or len(audio) == 0:
                        continue
                    
                    # --- 新增：重采样逻辑 ---
                    target_rate = 48000 # 也就是声卡通常支持的 48kHz
                    if self.sample_rate != target_rate:
                        # 计算新的采样点数
                        num_samples = int(len(audio) * target_rate / self.sample_rate)
                        # 执行重采样
                        audio = scipy.signal.resample(audio, num_samples)
                    
                    # --- 播放 ---
                    # 注意：这里传入 target_rate (48000) 而不是 self.sample_rate
                    sd.play(audio, target_rate, device=self.output_device, blocking=True)
                
                self.get_logger().info("Playback finished.")

            except Exception as e:
                self.get_logger().error(f"Error during synthesis/playback: {e}")
            finally:
                self.text_queue.task_done()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = IndexTTSNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Runtime error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()