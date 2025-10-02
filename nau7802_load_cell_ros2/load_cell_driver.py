"""
NAU7802 Load Cell Driver for ROS2
"""
import smbus2
import time
import statistics
import json
import os

class NAU7802LoadCell:
    # I2C configuration
    I2C_ADDR = 0x2A
    I2C_BUS = 2
    
    # NAU7802 registers
    REG_PU_CTRL = 0x00
    REG_CTRL1 = 0x01
    REG_CTRL2 = 0x02
    REG_ADCO_B2 = 0x12
    REG_REVISION = 0x1F
    REG_ADC = 0x15
    REG_PGA = 0x1B
    
    def __init__(self, calibration_file='load_cell_calibration.json'):
        self.bus = smbus2.SMBus(self.I2C_BUS)
        self.zero_offset = 0
        self.scale_factor = 1.0
        self.initialized = False
        self.calibration_file = calibration_file
    
    def reset(self):
        """Soft reset sequence"""
        self.bus.write_byte_data(self.I2C_ADDR, self.REG_PU_CTRL, 0x01)
        time.sleep(0.01)
        self.bus.write_byte_data(self.I2C_ADDR, self.REG_PU_CTRL, 0x02)
        time.sleep(0.001)
        val = self.bus.read_byte_data(self.I2C_ADDR, self.REG_PU_CTRL)
        return val & (1 << 3)
    
    def enable(self):
        """Enable analog + digital blocks"""
        val = self.bus.read_byte_data(self.I2C_ADDR, self.REG_PU_CTRL)
        val |= (1 << 1) | (1 << 2)
        self.bus.write_byte_data(self.I2C_ADDR, self.REG_PU_CTRL, val)
        time.sleep(0.6)
        val |= (1 << 4)
        self.bus.write_byte_data(self.I2C_ADDR, self.REG_PU_CTRL, val)
        val = self.bus.read_byte_data(self.I2C_ADDR, self.REG_PU_CTRL)
        return val & (1 << 3)
    
    def configure(self):
        """Configure NAU7802 for load cells"""
        pu_ctrl = self.bus.read_byte_data(self.I2C_ADDR, self.REG_PU_CTRL)
        pu_ctrl |= (1 << 7)
        self.bus.write_byte_data(self.I2C_ADDR, self.REG_PU_CTRL, pu_ctrl)
        
        self.bus.write_byte_data(self.I2C_ADDR, self.REG_CTRL1, (5 << 3) | 7)
        
        ctrl2 = self.bus.read_byte_data(self.I2C_ADDR, self.REG_CTRL2)
        ctrl2 &= ~(0b111 << 4)
        self.bus.write_byte_data(self.I2C_ADDR, self.REG_CTRL2, ctrl2)
        
        adc_reg = self.bus.read_byte_data(self.I2C_ADDR, self.REG_ADC)
        adc_reg = (adc_reg & ~(0b11 << 4)) | (0b11 << 4)
        self.bus.write_byte_data(self.I2C_ADDR, self.REG_ADC, adc_reg)
        
        pga_reg = self.bus.read_byte_data(self.I2C_ADDR, self.REG_PGA)
        pga_reg &= ~(1 << 6)
        self.bus.write_byte_data(self.I2C_ADDR, self.REG_PGA, pga_reg)
    
    def calibrate_internal(self):
        """Internal calibration"""
        ctrl2 = self.bus.read_byte_data(self.I2C_ADDR, self.REG_CTRL2)
        ctrl2 = (ctrl2 & ~0b11) | (1 << 2)
        self.bus.write_byte_data(self.I2C_ADDR, self.REG_CTRL2, ctrl2)
        
        for _ in range(100):
            ctrl2 = self.bus.read_byte_data(self.I2C_ADDR, self.REG_CTRL2)
            if not (ctrl2 & (1 << 2)):
                break
            time.sleep(0.1)
        
        return not (ctrl2 & (1 << 3))
    
    def initialize(self):
        """Initialize the NAU7802"""
        if not self.reset() or not self.enable():
            return False
        
        self.configure()
        
        if not self.calibrate_internal():
            return False
        
        self.initialized = True
        return True
    
    def data_ready(self):
        """Check if data is ready"""
        status = self.bus.read_byte_data(self.I2C_ADDR, self.REG_PU_CTRL)
        return bool(status & (1 << 5))
    
    def read_raw(self):
        """Read raw ADC value"""
        data = self.bus.read_i2c_block_data(self.I2C_ADDR, self.REG_ADCO_B2, 3)
        raw = (data[0] << 16) | (data[1] << 8) | data[2]
        if raw & 0x800000:
            raw |= 0xFF000000
        return raw
    
    def read_average(self, samples=20):
        """Read average of multiple samples"""
        readings = []
        for _ in range(samples):
            while not self.data_ready():
                time.sleep(0.01)
            readings.append(self.read_raw())
            time.sleep(0.05)
        return statistics.mean(readings)
    
    def zero_calibration(self, samples=30):
        """Zero point calibration"""
        time.sleep(3)  # Stabilization
        self.zero_offset = self.read_average(samples)
        return True
    
    def weight_calibration(self, known_weight_kg, samples=30):
        """Calibrate with known weight"""
        time.sleep(3)  # Stabilization
        weight_reading = self.read_average(samples)
        raw_difference = weight_reading - self.zero_offset
        
        if abs(raw_difference) < 1000:
            return False
        
        known_weight_grams = known_weight_kg * 1000
        self.scale_factor = known_weight_grams / raw_difference
        return True
    
    def save_calibration(self):
        """Save calibration to file"""
        calibration_data = {
            'zero_offset': self.zero_offset,
            'scale_factor': self.scale_factor,
            'calibration_date': time.strftime('%Y-%m-%d %H:%M:%S'),
            'units': 'grams'
        }
        
        with open(self.calibration_file, 'w') as f:
            json.dump(calibration_data, f, indent=2)
        
        return True
    
    def load_calibration(self):
        """Load calibration from file"""
        if os.path.exists(self.calibration_file):
            try:
                with open(self.calibration_file, 'r') as f:
                    data = json.load(f)
                
                self.zero_offset = data['zero_offset']
                self.scale_factor = data['scale_factor']
                return True
            except Exception:
                return False
        return False
    
    def read_weight(self):
        """Read weight in grams"""
        if not self.initialized:
            return None
        
        while not self.data_ready():
            time.sleep(0.01)
        
        raw = self.read_raw()
        weight_grams = (raw - self.zero_offset) * self.scale_factor
        return weight_grams
    
    def read_weight_average(self, samples=5):
        """Read average weight"""
        weights = []
        for _ in range(samples):
            weight = self.read_weight()
            if weight is not None:
                weights.append(weight)
        return statistics.mean(weights) if weights else None
