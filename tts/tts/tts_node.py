#!/usr/bin/env python3
import os
os.environ["HF_HUB_OFFLINE"] = "1" # 强制禁止联网

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from threading import Lock
import sounddevice as sd
import scipy.signal 

from ros2_tools.srv import TTS

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
        self.declare_parameter('device', 'USB Audio') 
        
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

        # 3. 创建服务锁与服务端，确保同一时间只播放一个请求
        self._play_lock = Lock()
        self.cb_group = ReentrantCallbackGroup()
        self.tts_service = self.create_service(TTS, 'tts_play', self.handle_tts_request, callback_group=self.cb_group)
        self.get_logger().info("TTS service 'tts_play' ready (blocking mode)")
    def synthesize_and_play(self, text: str):
        self.get_logger().info(f"Synthesizing: {text}")

        generator = self.pipeline(
            text,
            voice=self.voice,
            speed=self.speed,
            split_pattern=r'\n+',
        )

        for _, _, audio in generator:
            if audio is None or len(audio) == 0:
                continue

            target_rate = 48000
            if self.sample_rate != target_rate:
                num_samples = int(len(audio) * target_rate / self.sample_rate)
                audio = scipy.signal.resample(audio, num_samples)

            sd.play(audio, target_rate, device=self.output_device, blocking=True)

        self.get_logger().info("Playback finished.")

    def handle_tts_request(self, request, response):
        text = request.text.strip()
        if not text:
            response.success = False
            response.message = "Empty text requested"
            return response

        self._play_lock.acquire()
        try:
            self.synthesize_and_play(text)
            response.success = True
            response.message = "Playback finished"
        except Exception as exc:
            self.get_logger().error(f"Error during synthesis/playback: {exc}")
            response.success = False
            response.message = str(exc)
        finally:
            self._play_lock.release()

        return response

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = IndexTTSNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Runtime error: {e}")
    finally:
        if 'node' in locals():
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            # Prevent crashing on shutdown when the context is already shut down.
            pass

if __name__ == '__main__':
    main()