#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

try:
    from hx711 import HX711
    HAS_HX711 = True
except ImportError:
    HAS_HX711 = False
    print("Warning: HX711 library not available. Running in simulation mode.")

try:
    import RPLCD.i2c as LCD_I2C
    HAS_LCD = True
except ImportError:
    HAS_LCD = False
    print("Warning: RPLCD library not available. Running in simulation mode.")

class LoadCellLCDController(Node):
    """
    Monitors load cell and controls LCD display
    Subscribes: /gates/g3_crossed (Bool)
    Publishes: /load_cell/weight_g (Float32), /lcd/display (String)
    """
    
    def __init__(self):
        super().__init__('loadcell_lcd_controller')
        
        # Parameters
        self.declare_parameter('expected_weight_g', 125.0)
        self.declare_parameter('tolerance_g', 5.0)
        self.declare_parameter('team_name', 'Team LRC')
        self.declare_parameter('hx711_dout', 5)
        self.declare_parameter('hx711_sck', 6)
        
        self.expected_weight = self.get_parameter('expected_weight_g').value
        self.tolerance = self.get_parameter('tolerance_g').value
        self.team_name = self.get_parameter('team_name').value
        dout_pin = self.get_parameter('hx711_dout').value
        sck_pin = self.get_parameter('hx711_sck').value
        
        # Hardware setup
        if HAS_HX711:
            try:
                self.hx = HX711(dout_pin=dout_pin, pd_sck_pin=sck_pin)
                self.hx.reset()
                self.get_logger().info('HX711 load cell initialized')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize HX711: {e}')
                self.hx = None
        else:
            self.hx = None
        
        if HAS_LCD:
            try:
                self.lcd = LCD_I2C.CharLCD('PCF8574', 0x27, cols=16, rows=2)
                self.lcd.clear()
                self.get_logger().info('LCD initialized')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize LCD: {e}')
                self.lcd = None
        else:
            self.lcd = None
        
        # ROS interfaces
        self.gate3_sub = self.create_subscription(
            Bool, '/gates/g3_crossed', self.gate3_callback, 10)
        
        self.weight_pub = self.create_publisher(Float32, '/load_cell/weight_g', 10)
        self.display_pub = self.create_publisher(String, '/lcd/display', 10)
        
        # Timer for periodic weight reading
        self.weight_timer = self.create_timer(0.5, self.read_weight)
        
        self.verification_active = False
        self.get_logger().info('LoadCell + LCD controller ready')
    
    def gate3_callback(self, msg):
        """Start weight verification when gate 3 is crossed"""
        if msg.data:
            self.get_logger().info('Gate 3 crossed - starting weight verification')
            self.verification_active = True
            self.verify_weight()
    
    def read_weight(self):
        """Periodically read and publish weight"""
        if self.hx:
            try:
                weight = self.hx.get_weight(5)  # Average of 5 readings
            except:
                weight = 0.0
        else:
            weight = 125.0  # Simulation value
        
        msg = Float32()
        msg.data = weight
        self.weight_pub.publish(msg)
    
    def verify_weight(self):
        """Check if weight matches expected value and update LCD"""
        if self.hx:
            try:
                weight = self.hx.get_weight(10)  # More samples for final check
            except:
                weight = 0.0
        else:
            weight = 125.0  # Simulation
        
        self.get_logger().info(f'Final weight: {weight:.2f} g (expected {self.expected_weight} g)')
        
        # Check tolerance
        if abs(weight - self.expected_weight) <= self.tolerance:
            # Success!
            display_text = f'{self.team_name}\nSuccess!'
            self.update_lcd(display_text)
            
            display_msg = String()
            display_msg.data = 'SUCCESS'
            self.display_pub.publish(display_msg)
            
            self.get_logger().info('Weight verification PASSED')
        else:
            # Error
            display_text = f'Weight Mismatch\n{weight:.1f}g != {self.expected_weight:.1f}g'
            self.update_lcd(display_text)
            
            display_msg = String()
            display_msg.data = 'ERROR'
            self.display_pub.publish(display_msg)
            
            self.get_logger().error('Weight verification FAILED')
    
    def update_lcd(self, text):
        """Update LCD display"""
        if self.lcd:
            try:
                self.lcd.clear()
                lines = text.split('\n')
                for i, line in enumerate(lines[:2]):  # Max 2 lines
                    self.lcd.cursor_pos = (i, 0)
                    self.lcd.write_string(line[:16])  # Max 16 chars per line
            except Exception as e:
                self.get_logger().error(f'LCD write error: {e}')
        else:
            self.get_logger().info(f'LCD Display: {text}')
    
    def cleanup(self):
        if self.lcd:
            try:
                self.lcd.clear()
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    controller = LoadCellLCDController()
    
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
