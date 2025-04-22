import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int8
import busio
import board
import adafruit_amg88xx
import time
import RPi.GPIO as GPIO
###################################
# SET UP
###################################
GPIO.setmode(GPIO.BCM)
servo_pin = 21
solenoid_pin = 12
GPIO.setup(servo_pin, GPIO.OUT)
GPIO.setup(solenoid_pin,GPIO.OUT)
p = GPIO.PWM(servo_pin, 50)
i2c = busio.I2C(board.SCL, board.SDA)
amg = adafruit_amg88xx.AMG88XX(i2c,addr=0x69)

###################################
# CLASSES
###################################
class HeatSensorLauncher(Node):
    def __init__(self):
        super().__init__("HeatSensorLauncher")
        # Publisher setup: publishes thermal image data every second.
        self.publisher = self.create_publisher(Float64MultiArray, "/thermal_image", 10)
        self.timer = self.create_timer(1.0, self.publish_callback)
        
        # Subscriber setup: listens to the 'launch_topic'
        self.subscriber = self.create_subscription(Int8, "launch_topic", self.launch_callback, 10)
    
    def publish_callback(self):
        msg = Float64MultiArray()
        # Flatten the array of pixel values
        msg.data = [value for row in amg.pixels for value in row]
        self.publisher.publish(msg)
        self.get_logger().info("Published thermal image data")
    
    def launch_callback(self, msg):
        # This callback is triggered when an Int8 message arrives on 'launch_topic'
        self.get_logger().info("Received launch command, activating actuator sequence")
        p.start(8)
        for x in range(3):
            for i in range(8, 12):
                p.ChangeDutyCycle(i)
                time.sleep(0.1)
            for i in range(120, 80, -1):
                p.ChangeDutyCycle(i / 10)
                time.sleep(0.02)
            time.sleep(0.3)
            GPIO.output(solenoid_pin, GPIO.HIGH)
            time.sleep(0.5)
            GPIO.output(solenoid_pin, GPIO.LOW)
            if x == 1:
                time.sleep(2)


def main(args=None):
    rclpy.init(args=args)
    node = HeatSensorLauncher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()
    p.stop() 

if __name__ == "__main__":
	main()
