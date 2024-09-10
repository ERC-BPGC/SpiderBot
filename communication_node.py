import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from serial import Serial, SerialException
from serial.tools import list_ports

ARDUINO_SERIAL_NUMBERS = [
    "557373130313519040B1"
]

class CommunicationNode(Node):
    def __init__(self):
        super().__init__("communication_node")
        self.get_logger().info("Starting communication node")
        
        devices = list_ports.comports()
        ports = []
        for device in devices:
            port, desc, hwid = device
            serial_number = hwid.split()[2].split("=")[1]
            
            if serial_number in ARDUINO_SERIAL_NUMBERS:
                ports.append(port)
        
        for port in ports:
            try:
                arduino = Serial(port, baudrate=9600)
                self.get_logger().info(f"Connection to arduino on port {port}")
            except SerialException as e:
                self.get_logger().info(f"There was a problem connecting to arduino on port {port}.\n{e.with_traceback()}")
def main(args=None):
    rclpy.init(args=args)
    node = CommunicationNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()