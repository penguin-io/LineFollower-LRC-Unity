#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
import time

try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except ImportError:
    HAS_GPIO = False
    print("Warning: RPi.GPIO not available. Running in simulation mode.")

class PumpController(Node):
    """
    Controls peristaltic pump via stepper motor
    Subscribes: /gates/g1_crossed (Bool)
    Publishes: /pump/state (String), /pump/volume_dispensed (Float32)
    """
    
    def __init__(self):
        super().__init__('pump_controller')
        
        # Parameters
        self.declare_parameter('step_pin', 17)
        self.declare_parameter('dir_pin', 27)
        self.declare_parameter('steps_per_ml', 80.0)  # Calibration factor
        self.declare_parameter('target_volume_ml', 125.0)
        
        self.step_pin = self.get_parameter('step_pin').value
        self.dir_pin = self.get_parameter('dir_pin').value
        self.steps_per_ml = self.get_parameter('steps_per_ml').value
        self.target_volume = self.get_parameter('target_volume_ml').value
        
        # GPIO setup
        if HAS_GPIO:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.step_pin, GPIO.OUT)
            GPIO.setup(self.dir_pin, GPIO.OUT)
            GPIO.output(self.dir_pin, GPIO.HIGH)  # Forward direction
            self.get_logger().info('GPIO initialized')
        
        # ROS interfaces
        self.gate1_sub = self.create_subscription(
            Bool, '/gates/g1_crossed', self.gate1_callback, 10)
        
        self.state_pub = self.create_publisher(String, '/pump/state', 10)
        self.volume_pub = self.create_publisher(Float32, '/pump/volume_dispensed', 10)
        
        self.is_dispensing = False
        self.volume_dispensed = 0.0
        
        self.get_logger().info('Pump controller ready')
    
    def gate1_callback(self, msg):
        """Trigger pump when gate 1 is crossed"""
        if msg.data and not self.is_dispensing:
            self.get_logger().info('Gate 1 crossed - starting dispense')
            self.dispense_volume(self.target_volume)
    
    def dispense_volume(self, volume_ml):
        """Dispense specified volume"""
        self.is_dispensing = True
        
        # Publish state
        state_msg = String()
        state_msg.data = 'running'
        self.state_pub.publish(state_msg)
        
        # Calculate steps
        total_steps = int(volume_ml * self.steps_per_ml)
        step_delay = 0.001  # 1ms between steps (1000 steps/sec)
        
        self.get_logger().info(f'Dispensing {volume_ml} ml ({total_steps} steps)')
        
        # Execute steps
        if HAS_GPIO:
            for step in range(total_steps):
                GPIO.output(self.step_pin, GPIO.HIGH)
                time.sleep(step_delay)
                GPIO.output(self.step_pin, GPIO.LOW)
                time.sleep(step_delay)
                
                # Publish progress every 1000 steps
                if step % 1000 == 0:
                    self.volume_dispensed = step / self.steps_per_ml
                    vol_msg = Float32()
                    vol_msg.data = self.volume_dispensed
                    self.volume_pub.publish(vol_msg)
        else:
            # Simulation: just wait equivalent time
            time.sleep(total_steps * step_delay * 2)
            self.get_logger().info(f'Simulated dispense of {volume_ml} ml')
        
        # Complete
        self.volume_dispensed = volume_ml
        vol_msg = Float32()
        vol_msg.data = self.volume_dispensed
        self.volume_pub.publish(vol_msg)
        
        state_msg.data = 'complete'
        self.state_pub.publish(state_msg)
        
        self.is_dispensing = False
        self.get_logger().info(f'Dispense complete: {volume_ml} ml')
    
    def cleanup(self):
        if HAS_GPIO:
            GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    controller = PumpController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.cleanup()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
