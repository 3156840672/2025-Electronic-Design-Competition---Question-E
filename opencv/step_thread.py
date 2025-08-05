import RPi.GPIO as GPIO
import time
import threading
import logging
import atexit

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class StepperMotor:
    _gpio_initialized = False
    
    def __init__(self, dir_pin, step_pin, steps_per_rev=200):
        """初始化步进电机"""
        if not self._gpio_initialized:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            StepperMotor._gpio_initialized = True
            atexit.register(self._global_cleanup)
            
        self.dir_pin = dir_pin
        self.step_pin = step_pin
        self.steps_per_rev = steps_per_rev
        
        GPIO.setup(self.dir_pin, GPIO.OUT)
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.output(self.dir_pin, GPIO.LOW)
        GPIO.output(self.step_pin, GPIO.LOW)
        
        self._stop_event = threading.Event()
        self._current_thread = None
        self._lock = threading.Lock()
        
        logging.info(f"步进电机初始化完成 DIR:BCM{dir_pin}, STEP:BCM{step_pin}")

    def _global_cleanup(self):
        """程序退出时的全局清理"""
        if self._gpio_initialized:
            GPIO.cleanup()
            logging.info("全局GPIO资源已清理")

    def step(self, direction, steps, rpm=10):
        """非阻塞方式移动指定步数"""
        self._stop_event.set()
        if self._current_thread and self._current_thread.is_alive():
            self._current_thread.join()
        
        self._stop_event.clear()
        self._current_thread = threading.Thread(
            target=self._move,
            args=(direction, steps, rpm),
            daemon=True
        )
        self._current_thread.start()

    def _move(self, direction, steps, rpm):
        """实际移动函数"""
        delay = 60.0 / (rpm * self.steps_per_rev) if rpm > 0 else 0.01
        
        with self._lock:
            GPIO.output(self.dir_pin, direction)
            time.sleep(0.001)
            
            for _ in range(steps):
                if self._stop_event.is_set():
                    break
                
                GPIO.output(self.step_pin, GPIO.HIGH)
                time.sleep(delay / 2)
                GPIO.output(self.step_pin, GPIO.LOW)
                time.sleep(delay / 2)

    def wait_until_idle(self, timeout=None):
        """等待电机完成运动"""
        if self._current_thread:
            self._current_thread.join(timeout)

    def stop(self, immediate=False):
        """停止电机运动"""
        self._stop_event.set()
        if immediate and self._current_thread:
            self._current_thread.join(0.1)

    def cleanup(self):
        """清理当前电机资源"""
        self.stop(immediate=True)
        # 不清理GPIO状态，由全局清理处理
        logging.info("步进电机资源已清理")

    def __del__(self):
        """避免使用析构函数清理"""
        pass
