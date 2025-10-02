#!/usr/bin/env python3
"""
ROS2 Node for NAU7802 Load Cell - Fixed for higher sample rates
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import WrenchStamped
import os
from ament_index_python.packages import get_package_share_directory

from nau7802_load_cell_ros2.load_cell_driver import NAU7802LoadCell


class LoadCellNode(Node):
    def __init__(self):
        super().__init__('load_cell_node')
        
        # Declare parameters
        self.declare_parameter('publish_rate', 50.0)  # Increased default
        self.declare_parameter('average_samples', 1)  # Reduced default for speed
        self.declare_parameter('calibration_file', 'load_cell_calibration.json')
        self.declare_parameter('i2c_bus', 2)
        self.declare_parameter('i2c_addr', 0x2A)
        self.declare_parameter('adc_sample_rate', 80)  # 10, 20, 40, 80, or 320 SPS
        
        # Get parameters
        publish_rate = self.get_parameter('publish_rate').value
        self.average_samples = self.get_parameter('average_samples').value
        calibration_file = self.get_parameter('calibration_file').value
        adc_rate_param = self.get_parameter('adc_sample_rate').value
        
        # Map sample rate parameter to driver constants
        rate_map = {
            10: NAU7802LoadCell.RATE_10SPS,
            20: NAU7802LoadCell.RATE_20SPS,
            40: NAU7802LoadCell.RATE_40SPS,
            80: NAU7802LoadCell.RATE_80SPS,
            320: NAU7802LoadCell.RATE_320SPS
        }
        adc_rate = rate_map.get(adc_rate_param, NAU7802LoadCell.RATE_80SPS)
        
        # Setup calibration file path
        try:
            pkg_share = get_package_share_directory('nau7802_load_cell_ros2')
            self.calibration_path = os.path.join(pkg_share, 'config', calibration_file)
        except:
            self.calibration_path = calibration_file
        
        # Initialize load cell driver with specified sample rate
        self.load_cell = NAU7802LoadCell(
            calibration_file=self.calibration_path,
            sample_rate=adc_rate
        )
        
        # Publishers
        self.weight_pub = self.create_publisher(Float32, 'load_cell/weight', 10)
        self.wrench_pub = self.create_publisher(WrenchStamped, 'load_cell/wrench', 10)
        self.raw_pub = self.create_publisher(Float32, 'load_cell/raw', 10)
        
        # Services
        self.zero_srv = self.create_service(
            Trigger,
            'load_cell/zero_calibration',
            self.zero_calibration_callback
        )
        self.weight_cal_srv = self.create_service(
            Trigger,
            'load_cell/weight_calibration',
            self.weight_calibration_callback
        )
        self.save_cal_srv = self.create_service(
            Trigger,
            'load_cell/save_calibration',
            self.save_calibration_callback
        )
        self.load_cal_srv = self.create_service(
            Trigger,
            'load_cell/load_calibration',
            self.load_calibration_callback
        )
        
        # Initialize hardware
        self.get_logger().info('Initializing NAU7802 load cell...')
        if not self.load_cell.initialize():
            self.get_logger().error('Failed to initialize NAU7802!')
            return
        
        self.get_logger().info('NAU7802 initialized successfully')
        self.get_logger().info(f'ADC sample rate: {adc_rate_param} SPS')
        
        # Try to load existing calibration
        if self.load_cell.load_calibration():
            self.get_logger().info(f'Loaded calibration from {self.calibration_path}')
            self.get_logger().info(f'Zero offset: {self.load_cell.zero_offset:.1f}')
            self.get_logger().info(f'Scale factor: {self.load_cell.scale_factor:.6f}')
        else:
            self.get_logger().warn('No calibration file found. Please calibrate the load cell.')
        
        # Create timer for publishing
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_callback)
        
        self.get_logger().info(f'Load cell node started, publishing at {publish_rate} Hz')
        if self.average_samples > 1:
            self.get_logger().info(f'Averaging {self.average_samples} samples per reading')
    
    def publish_callback(self):
        """Timer callback to read and publish weight data"""
        try:
            # Read weight (with optional averaging)
            if self.average_samples > 1:
                weight_grams = self.load_cell.read_weight_average(self.average_samples)
            else:
                weight_grams = self.load_cell.read_weight()
            
            if weight_grams is None:
                return
            
            # Get timestamp once
            stamp = self.get_clock().now().to_msg()
            
            # Publish weight in grams
            weight_msg = Float32()
            weight_msg.data = float(weight_grams)
            self.weight_pub.publish(weight_msg)
            
            # Publish as WrenchStamped (force in Newtons)
            wrench_msg = WrenchStamped()
            wrench_msg.header.stamp = stamp
            wrench_msg.header.frame_id = 'load_cell'
            wrench_msg.wrench.force.z = float(weight_grams * 0.00980665)  # grams to Newtons
            self.wrench_pub.publish(wrench_msg)
            
            # Publish raw value (only if data is ready to avoid blocking)
            if self.load_cell.data_ready():
                raw_msg = Float32()
                raw_msg.data = float(self.load_cell.read_raw())
                self.raw_pub.publish(raw_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error reading load cell: {str(e)}')
    
    def zero_calibration_callback(self, request, response):
        """Service callback for zero calibration"""
        self.get_logger().info('Starting zero calibration...')
        self.get_logger().info('Make sure NO WEIGHT is on the load cell!')
        
        try:
            if self.load_cell.zero_calibration():
                response.success = True
                response.message = f'Zero calibration successful. Offset: {self.load_cell.zero_offset:.1f}'
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = 'Zero calibration failed'
                self.get_logger().error(response.message)
        except Exception as e:
            response.success = False
            response.message = f'Error during calibration: {str(e)}'
            self.get_logger().error(response.message)
        
        return response
    
    def weight_calibration_callback(self, request, response):
        """Service callback for weight calibration"""
        # Note: This assumes a 1kg weight. You may want to make this a parameter
        known_weight_kg = 1.0
        
        self.get_logger().info(f'Starting weight calibration with {known_weight_kg}kg weight...')
        self.get_logger().info(f'Place the {known_weight_kg}kg weight on the load cell!')
        
        try:
            if self.load_cell.weight_calibration(known_weight_kg):
                response.success = True
                response.message = f'Weight calibration successful. Scale factor: {self.load_cell.scale_factor:.6f}'
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = 'Weight calibration failed. Check connections and weight placement.'
                self.get_logger().error(response.message)
        except Exception as e:
            response.success = False
            response.message = f'Error during calibration: {str(e)}'
            self.get_logger().error(response.message)
        
        return response
    
    def save_calibration_callback(self, request, response):
        """Service callback to save calibration"""
        self.get_logger().info('Saving calibration...')
        
        try:
            if self.load_cell.save_calibration():
                response.success = True
                response.message = f'Calibration saved to {self.calibration_path}'
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = 'Failed to save calibration'
                self.get_logger().error(response.message)
        except Exception as e:
            response.success = False
            response.message = f'Error saving calibration: {str(e)}'
            self.get_logger().error(response.message)
        
        return response
    
    def load_calibration_callback(self, request, response):
        """Service callback to load calibration"""
        self.get_logger().info('Loading calibration...')
        
        try:
            if self.load_cell.load_calibration():
                response.success = True
                response.message = f'Calibration loaded from {self.calibration_path}'
                self.get_logger().info(response.message)
                self.get_logger().info(f'Zero offset: {self.load_cell.zero_offset:.1f}')
                self.get_logger().info(f'Scale factor: {self.load_cell.scale_factor:.6f}')
            else:
                response.success = False
                response.message = 'Failed to load calibration file'
                self.get_logger().error(response.message)
        except Exception as e:
            response.success = False
            response.message = f'Error loading calibration: {str(e)}'
            self.get_logger().error(response.message)
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LoadCellNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
