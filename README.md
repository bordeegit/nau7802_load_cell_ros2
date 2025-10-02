# NAU7802 Load Cell ROS2 Package

ROS2 driver package for NAU7802 load cell sensor connected via I2C.

## Package Structure

```
nau7802_load_cell_ros2/
├── nau7802_load_cell_ros2/
│   ├── __init__.py
│   ├── load_cell_driver.py       # Hardware driver class
│   └── load_cell_node.py          # ROS2 node
├── launch/
│   └── load_cell.launch.py        # Launch file
├── config/
│   └── load_cell_calibration.json # Calibration data (created after calibration)
├── resource/
│   └── nau7802_load_cell_ros2          # Package marker
├── package.xml
├── setup.py
└── README.md
```

## Installation

### Dependencies

Install Python dependencies:
```bash
pip3 install smbus2
```

### Build the Package

1. Copy this package to your ROS2 workspace:
```bash
cd ~/ros2_ws/src
# Copy the nau7802_load_cell_ros2 folder here
```

2. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select nau7802_load_cell_ros2
source install/setup.bash
```

## Usage

### Launch the Node

```bash
ros2 launch nau7802_load_cell_ros2 load_cell.launch.py
```

With custom parameters:
```bash
ros2 launch nau7802_load_cell_ros2 load_cell.launch.py publish_rate:=20.0 average_samples:=10
```

### Topics

#### Published Topics

- `/load_cell/weight` (`std_msgs/Float32`) - Weight in grams
- `/load_cell/wrench` (`geometry_msgs/WrenchStamped`) - Force in Newtons (z-axis)
- `/load_cell/raw` (`std_msgs/Float32`) - Raw ADC value

#### Example: Subscribe to Weight

```bash
ros2 topic echo /load_cell/weight
```

### Services

#### Calibration Services

1. **Zero Calibration** (Tare)
   ```bash
   # Remove all weight from the load cell first!
   ros2 service call /load_cell/zero_calibration std_srvs/srv/Trigger
   ```

2. **Weight Calibration**
   ```bash
   # Place a 1kg calibration weight on the load cell first!
   ros2 service call /load_cell/weight_calibration std_srvs/srv/Trigger
   ```

3. **Save Calibration**
   ```bash
   ros2 service call /load_cell/save_calibration std_srvs/srv/Trigger
   ```

4. **Load Calibration**
   ```bash
   ros2 service call /load_cell/load_calibration std_srvs/srv/Trigger
   ```

### Calibration Procedure

Follow these steps to calibrate your load cell:

1. **Start the node**:
   ```bash
   ros2 launch nau7802_load_cell_ros2 load_cell.launch.py
   ```

2. **Zero calibration** (remove all weight):
   ```bash
   ros2 service call /load_cell/zero_calibration std_srvs/srv/Trigger
   ```

3. **Weight calibration** (place 1kg weight):
   ```bash
   ros2 service call /load_cell/weight_calibration std_srvs/srv/Trigger
   ```

4. **Save calibration**:
   ```bash
   ros2 service call /load_cell/save_calibration std_srvs/srv/Trigger
   ```

5. **Test** - Monitor the weight readings:
   ```bash
   ros2 topic echo /load_cell/weight
   ```

## Parameters

- `publish_rate` (double, default: 10.0) - Publishing frequency in Hz
- `average_samples` (int, default: 5) - Number of samples to average per reading
- `calibration_file` (string, default: "load_cell_calibration.json") - Calibration file name
- `i2c_bus` (int, default: 2) - I2C bus number
- `i2c_addr` (int, default: 0x2A) - I2C device address

## Hardware Configuration

- **I2C Address**: 0x2A (42)
- **I2C Bus**: 2 (Odroid H4+)
- **Gain**: 128x
- **Sample Rate**: 10 SPS
- **LDO Voltage**: 3.0V

## Troubleshooting

### Permission Issues

If you get I2C permission errors, add your user to the i2c group:
```bash
sudo usermod -a -G i2c $USER
sudo reboot
```

### Node Fails to Start

1. Check I2C connection:
   ```bash
   i2cdetect -y 2
   ```
   You should see device at address 0x2A.

2. Verify permissions:
   ```bash
   ls -l /dev/i2c-2
   ```

3. Check ROS2 logs:
   ```bash
   ros2 run nau7802_load_cell_ros2 load_cell_node --ros-args --log-level debug
   ```

### Unstable Readings

- Ensure proper mechanical mounting of the load cell
- Check for loose connections
- Increase `average_samples` parameter
- Verify power supply stability
- Recalibrate the sensor

## Example Python Client

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class LoadCellListener(Node):
    def __init__(self):
        super().__init__('load_cell_listener')
        self.subscription = self.create_subscription(
            Float32,
            '/load_cell/weight',
            self.weight_callback,
            10)
    
    def weight_callback(self, msg):
        weight_kg = msg.data / 1000.0
        self.get_logger().info(f'Weight: {weight_kg:.3f} kg')

def main():
    rclpy.init()
    node = LoadCellListener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## License

MIT License

## Author

Your Name <your.email@example.com>
