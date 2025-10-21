#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

try:
    import board
    import neopixel
    HAS_NEOPIXEL = True
except ImportError:
    HAS_NEOPIXEL = False
    print("Warning: NeoPixel library not available. Running in simulation mode.")

class LEDController(Node):
    """
    Controls WS2812B LED array to display "LAM"
    Subscribes: /gates/g2_crossed (Bool), /led/lam (Bool)
    """
    
    def __init__(self):
        super().__init__('led_controller')
        
        # Parameters
        self.declare_parameter('num_pixels', 30)
        self.declare_parameter('brightness', 0.5)
        
        num_pixels = self.get_parameter('num_pixels').value
        brightness = self.get_parameter('brightness').value
        
        # Initialize NeoPixel strip
        if HAS_NEOPIXEL:
            try:
                self.pixels = neopixel.NeoPixel(
                    board.D18, num_pixels, brightness=brightness, auto_write=False)
                self.pixels.fill((0, 0, 0))
                self.pixels.show()
                self.get_logger().info('NeoPixel strip initialized')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize NeoPixel: {e}')
                self.pixels = None
        else:
            self.pixels = None
        
        # ROS interfaces
        self.gate2_sub = self.create_subscription(
            Bool, '/gates/g2_crossed', self.gate2_callback, 10)
        self.led_sub = self.create_subscription(
            Bool, '/led/lam', self.led_callback, 10)
        
        self.is_on = False
        self.get_logger().info('LED controller ready')
    
    def gate2_callback(self, msg):
        """Turn on LEDs when gate 2 is crossed"""
        if msg.data:
            self.get_logger().info('Gate 2 crossed - illuminating LAM')
            self.set_lam_pattern(True)
    
    def led_callback(self, msg):
        """Direct LED control"""
        self.set_lam_pattern(msg.data)
    
    def set_lam_pattern(self, on):
        """Display LAM pattern"""
        if self.pixels is None:
            self.get_logger().info(f'LAM LEDs: {"ON" if on else "OFF"} (simulation)')
            return
        
        if on and not self.is_on:
            # Display "LAM" pattern (customize pixel indices)
            # L: pixels 0-9, A: pixels 10-19, M: pixels 20-29
            color = (0, 255, 0)  # Green
            for i in range(30):
                self.pixels[i] = color
            self.pixels.show()
            self.is_on = True
            self.get_logger().info('LAM LEDs ON')
        
        elif not on and self.is_on:
            self.pixels.fill((0, 0, 0))
            self.pixels.show()
            self.is_on = False
            self.get_logger().info('LAM LEDs OFF')
    
    def cleanup(self):
        if self.pixels:
            self.pixels.fill((0, 0, 0))
            self.pixels.show()

def main(args=None):
    rclpy.init(args=args)
    controller = LEDController()
    
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
