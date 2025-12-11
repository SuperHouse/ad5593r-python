#
#    FILE: AD5593R.py
#  AUTHOR: Converted from Arduino library by Rob Tillaart
# VERSION: 0.1.1
#    DATE: 2024-01-30
# PURPOSE: Python library for AD5593R, I2C, 8 channel ADC / DAC / GPIO device.
#     URL: https://github.com/RobTillaart/AD5593R
#
#  read datasheet for details.


from smbus2 import SMBus
import time


#  LIBRARY VERSION
AD5593R_LIB_VERSION = "0.1.1"


#  ERROR CODES
AD5593R_OK = 0x0000
AD5593R_PIN_ERROR = 0xFF81
AD5593R_I2C_ERROR = 0xFF82
AD5593R_LDAC_ERROR = 0xFF83


#  LDAC MODES
AD5593R_LDAC_DIRECT = 0
AD5593R_LDAC_HOLD = 1
AD5593R_LDAC_RELEASE = 2


#  CONFIG REGISTERS (aka pointer bytes)
AD5593_NOP = 0x00
AD5593_ADC_SEQ = 0x02
AD5593_GEN_CTRL_REG = 0x03
AD5593_ADC_CONFIG = 0x04
AD5593_DAC_CONFIG = 0x05
AD5593_PULLDOWN_CONFIG = 0x06
AD5593_LDAC_MODE = 0x07
AD5593_GPIO_CONFIG = 0x08
AD5593_GPIO_OUTPUT = 0x09
AD5593_GPIO_INPUT = 0x0A
AD5593_POWERDOWN_REF_CTRL = 0x0B
AD5593_GPIO_OPENDRAIN_CONFIG = 0x0C
AD5593_IO_TS_CONFIG = 0x0D
AD5593_SW_RESET = 0x0F


#  IO REGISTERS
def AD5593_DAC_WRITE(x):
    return 0x10 + x


AD5593_ADC_READ = 0x40


def AD5593_DAC_READ(x):
    return 0x50 + x


AD5593_GPIO_READ = 0x60
AD5593_GPIO_READ_CONFIG = 0x70


class AD5593R:
    def __init__(self, device_address, bus_number=1):
        """
        Constructor for AD5593R.
        
        Args:
            device_address: I2C device address (0x10 or 0x11)
            bus_number: I2C bus number (default: 1 for Raspberry Pi)
        """
        self._address = device_address
        self._error = AD5593R_OK
        self._Vref = 2.5
        self._gain = 1
        self._bus = None
        self._bus_number = bus_number

    def begin(self):
        """
        Initialize the device.
        
        Returns:
            bool: True if device is connected, False otherwise
        """
        if not self.is_connected():
            return False
        return True

    def is_connected(self):
        """
        Check if device is connected on I2C bus.
        
        Returns:
            bool: True if device responds, False otherwise
        """
        try:
            if self._bus is None:
                self._bus = SMBus(self._bus_number)
            # Try to write to device to check connection (I2C ping)
            self._bus.write_byte(self._address, 0x00)
            return True
        except (IOError, OSError):
            return False

    def get_address(self):
        """
        Get the I2C address.
        
        Returns:
            int: Device address
        """
        return self._address

    #  MODE CONFIGURATION
    def set_mode(self, config):
        """
        Set mode for all 8 channels using a configuration string.
        A=ADC, D=DAC, I=INPUT, O=OUTPUT, T=THREESTATE e.g. "AADDIIOT"
        
        Args:
            config: String of 8 characters, one per channel
            
        Returns:
            int: 0 on success, -1 on error
        """
        if len(config) != 8:
            return -1
        
        bit_mask_dac = 0x00
        bit_mask_adc = 0x00
        bit_mask_in = 0x00
        bit_mask_out = 0x00
        bit_mask_ts = 0x00
        
        # Parse configuration string
        bm = 0x01
        for i in range(8):
            char = config[i].upper()
            if char == 'A':
                bit_mask_adc |= bm
            elif char == 'D':
                bit_mask_dac |= bm
            elif char == 'I':
                bit_mask_in |= bm
            elif char == 'T':
                bit_mask_ts |= bm
                bit_mask_out |= bm
            elif char == 'O':
                bit_mask_out |= bm
            bm <<= 1
        
        # Set all modes
        self.set_adc_mode(bit_mask_adc)
        self.set_dac_mode(bit_mask_dac)
        self.set_input_mode(bit_mask_in)
        self.set_output_mode(bit_mask_out)
        self.set_threestate_mode(bit_mask_ts)
        return 0

    def set_adc_mode(self, bit_mask):
        """
        Set pins to ADC mode.
        
        Args:
            bit_mask: Bit mask where 1's indicate ADC pins
            
        Returns:
            int: Error code
        """
        return self.write_register(AD5593_ADC_CONFIG, bit_mask)

    def set_dac_mode(self, bit_mask):
        """
        Set pins to DAC mode.
        
        Args:
            bit_mask: Bit mask where 1's indicate DAC pins
            
        Returns:
            int: Error code
        """
        return self.write_register(AD5593_DAC_CONFIG, bit_mask)

    def set_input_mode(self, bit_mask):
        """
        Set pins to INPUT mode.
        
        Args:
            bit_mask: Bit mask where 1's indicate INPUT pins
            
        Returns:
            int: Error code
        """
        return self.write_register(AD5593_GPIO_INPUT, bit_mask)

    def set_output_mode(self, bit_mask):
        """
        Set pins to OUTPUT mode.
        
        Args:
            bit_mask: Bit mask where 1's indicate OUTPUT pins
            
        Returns:
            int: Error code
        """
        return self.write_register(AD5593_GPIO_CONFIG, bit_mask)

    def set_threestate_mode(self, bit_mask):
        """
        Set pins to Three State OUTPUT mode.
        
        Args:
            bit_mask: Bit mask where 1's indicate Three State pins
            
        Returns:
            int: Error code
        """
        return self.write_register(AD5593_IO_TS_CONFIG, bit_mask)

    def set_pulldown_mode(self, bit_mask):
        """
        Set pins to 85 kOhm pull down to GND.
        
        Args:
            bit_mask: Bit mask where 1's indicate pull down pins
            
        Returns:
            int: Error code
        """
        return self.write_register(AD5593_PULLDOWN_CONFIG, bit_mask)

    def set_ldac_mode(self, mode):
        """
        Set LDAC mode.
        mode 0: COPY input register direct to DAC (default)
        mode 1: HOLD in input registers
        mode 2: RELEASE all input registers to DAC simultaneously
        Must be set AFTER set_external_reference.
        
        Args:
            mode: LDAC mode (0, 1, or 2)
            
        Returns:
            int: Error code
        """
        if mode > AD5593R_LDAC_RELEASE:
            return AD5593R_LDAC_ERROR
        return self.write_register(AD5593_LDAC_MODE, mode)

    def set_open_drain_mode(self, bit_mask):
        """
        Set pins to open drain mode (output mode, pull up resistor needed).
        
        Args:
            bit_mask: Bit mask where 1's indicate open drain pins
            
        Returns:
            int: Error code
        """
        return self.write_register(AD5593_GPIO_OPENDRAIN_CONFIG, bit_mask)

    #  DIGITAL IO
    def write1(self, pin, value):
        """
        Write HIGH/LOW to a single OUTPUT pin.
        
        Args:
            pin: Pin number (0-7)
            value: 0 for LOW, non-zero for HIGH
            
        Returns:
            int: Error code or result
        """
        if pin > 7:
            return AD5593R_PIN_ERROR
        
        bit_mask = self.read_config_register(AD5593_GPIO_OUTPUT)
        if value == 0:
            bit_mask &= ~(1 << pin)
        else:
            bit_mask |= (1 << pin)
        return self.write_register(AD5593_GPIO_OUTPUT, bit_mask)

    def read1(self, pin):
        """
        Read a single INPUT pin.
        
        Args:
            pin: Pin number (0-7)
            
        Returns:
            int: 1 for HIGH, 0 for LOW, or error code
        """
        if pin > 7:
            return AD5593R_PIN_ERROR
        
        bit_mask = self.read_io_register(AD5593_GPIO_READ)
        return (bit_mask >> pin) & 0x01

    def write8(self, bit_mask):
        """
        Write bit mask to all pins at once.
        
        Args:
            bit_mask: 8-bit mask where 1 = HIGH, 0 = LOW
            
        Returns:
            int: Error code
        """
        return self.write_register(AD5593_GPIO_OUTPUT, bit_mask)

    def read8(self):
        """
        Read all INPUT pins at once.
        
        Returns:
            int: 8-bit mask with pin values
        """
        return self.read_io_register(AD5593_GPIO_READ) & 0xFF

    #  REFERENCE VOLTAGE VREF
    def set_external_reference(self, flag, vref):
        """
        Set external or internal reference voltage.
        Power on = internal reference 2.5V.
        
        Args:
            flag: True for external reference, False for internal
            vref: Reference voltage (only used if flag is True)
            
        Returns:
            int: Error code
        """
        bit_mask = self.read_config_register(AD5593_POWERDOWN_REF_CTRL)
        if flag:  # external
            bit_mask &= ~0x0200
            self._Vref = vref
        else:  # internal
            bit_mask |= 0x0200
            self._Vref = 2.5
        return self.write_register(AD5593_POWERDOWN_REF_CTRL, bit_mask)

    def get_vref(self):
        """
        Get current reference voltage.
        
        Returns:
            float: Reference voltage in Volts
        """
        return self._Vref

    def set_adc_range_2x(self, flag):
        """
        Configure ADC range: 1x or 2x Vref.
        
        Args:
            flag: True for 2x Vref, False for 1x Vref
            
        Returns:
            int: Error code
        """
        # Remember for read_temperature()
        if flag:
            self._gain = 2
        else:
            self._gain = 1
        
        bit_mask = self.read_config_register(AD5593_GEN_CTRL_REG)
        if flag:
            bit_mask |= 0x0020
        else:
            bit_mask &= ~0x0020
        return self.write_register(AD5593_GEN_CTRL_REG, bit_mask)

    def set_dac_range_2x(self, flag):
        """
        Configure DAC range: 1x or 2x Vref.
        
        Args:
            flag: True for 2x Vref, False for 1x Vref
            
        Returns:
            int: Error code
        """
        bit_mask = self.read_config_register(AD5593_GEN_CTRL_REG)
        if flag:
            bit_mask |= 0x0010
        else:
            bit_mask &= ~0x0010
        return self.write_register(AD5593_GEN_CTRL_REG, bit_mask)

    #  GENERAL CONTROL
    def enable_adc_buffer_pre_charge(self, flag):
        """
        Enable/disable ADC buffer pre-charge.
        
        Args:
            flag: True to enable, False to disable
            
        Returns:
            int: Error code
        """
        bit_mask = self.read_config_register(AD5593_GEN_CTRL_REG)
        if flag:
            bit_mask |= 0x0200
        else:
            bit_mask &= ~0x0200
        return self.write_register(AD5593_GEN_CTRL_REG, bit_mask)

    def enable_adc_buffer(self, flag):
        """
        Enable/disable ADC buffer.
        
        Args:
            flag: True to enable, False to disable
            
        Returns:
            int: Error code
        """
        bit_mask = self.read_config_register(AD5593_GEN_CTRL_REG)
        if flag:
            bit_mask |= 0x0100
        else:
            bit_mask &= ~0x0100
        return self.write_register(AD5593_GEN_CTRL_REG, bit_mask)

    def enable_io_lock(self, flag):
        """
        Enable/disable IO lock.
        
        Args:
            flag: True to enable, False to disable
            
        Returns:
            int: Error code
        """
        bit_mask = self.read_config_register(AD5593_GEN_CTRL_REG)
        if flag:
            bit_mask |= 0x0080
        else:
            bit_mask &= ~0x0080
        return self.write_register(AD5593_GEN_CTRL_REG, bit_mask)

    def write_all_dacs(self, flag):
        """
        Enable/disable write all DACs.
        
        Args:
            flag: True to enable, False to disable
            
        Returns:
            int: Error code
        """
        bit_mask = self.read_config_register(AD5593_GEN_CTRL_REG)
        if flag:
            bit_mask |= 0x0040
        else:
            bit_mask &= ~0x0040
        return self.write_register(AD5593_GEN_CTRL_REG, bit_mask)

    #  ANALOG IO
    def write_dac(self, pin, value):
        """
        Write value to DAC pin.
        
        Args:
            pin: Pin number (0-7)
            value: 12-bit value (0-4095), values above 4095 will be clipped
            
        Returns:
            int: Error code
        """
        if pin > 7:
            return AD5593R_PIN_ERROR
        
        # Max 12 bit == 4095 => clipping
        if value > 0x0FFF:
            value = 0x0FFF
        
        return self.write_register(AD5593_DAC_WRITE(pin), value)

    def read_dac(self, pin):
        """
        Read back last written DAC value.
        
        Args:
            pin: Pin number (0-7)
            
        Returns:
            int: 12-bit DAC value or error code
        """
        if pin > 7:
            return AD5593R_PIN_ERROR
        
        raw = self.read_io_register(AD5593_DAC_READ(pin))
        # Remove four upper bits, contain the pin
        raw &= 0x0FFF
        return raw

    def read_adc(self, pin):
        """
        Read ADC value from pin.
        
        Args:
            pin: Pin number (0-8, where 8 is temperature)
            
        Returns:
            int: 12-bit ADC value or error code
        """
        # Allow pin 8 for raw temperature
        if pin > 8:
            return AD5593R_PIN_ERROR
        
        # Add all to the sequence including temperature.
        # 0x0200 = REPeat bit
        # 0x0100 = TEMPerature include bit
        pin_mask = 1 << pin
        self.write_register(AD5593_ADC_SEQ, 0x0200 | pin_mask)
        
        # Read one ADC conversion
        raw = self.read_io_register(AD5593_ADC_READ)
        # Remove four upper bits, contain the pin
        raw &= 0x0FFF
        return raw

    def read_temperature(self):
        """
        Read temperature sensor.
        Accuracy 3C over 5 samples averaged according datasheet.
        Returns value in Celsius between -40 and +125 C.
        
        Returns:
            float: Temperature in Celsius, or -273.15 on error
        """
        # 0x0200 = REPeat bit
        # 0x0100 = TEMPerature include bit
        self.write_register(AD5593_ADC_SEQ, 0x0200 | 0x0100)
        
        # Read one ADC conversion
        raw = self.read_io_register(AD5593_ADC_READ)
        
        # If not temperature ADC
        if (raw & 0xF000) != 0x8000:
            return -273.15  # 0 Kelvin
        
        # Remove four upper bits, contain the pin
        raw &= 0x0FFF
        
        # Formulas page 19 refactored to minimize float math
        return -283.59 + raw / 6.635 * self._gain * self._Vref

    #  POWER
    def power_down(self):
        """
        Power down all functionality, Low power mode.
        
        Returns:
            int: Error code
        """
        bit_mask = self.read_config_register(AD5593_POWERDOWN_REF_CTRL)
        bit_mask |= 0x0400
        return self.write_register(AD5593_POWERDOWN_REF_CTRL, bit_mask)

    def wake_up(self):
        """
        Wake up all functionality.
        
        Returns:
            int: Error code
        """
        self._Vref = 2.5
        bit_mask = self.read_config_register(AD5593_POWERDOWN_REF_CTRL)
        bit_mask &= ~0x0400
        return self.write_register(AD5593_POWERDOWN_REF_CTRL, bit_mask)

    def power_down_dac(self, pin):
        """
        Disable a single DAC.
        
        Args:
            pin: Pin number (0-7)
            
        Returns:
            int: Error code
        """
        if pin > 7:
            return 0
        
        bit_mask = self.read_config_register(AD5593_POWERDOWN_REF_CTRL)
        bit_mask |= (1 << pin)
        return self.write_register(AD5593_POWERDOWN_REF_CTRL, bit_mask)

    def wake_up_dac(self, pin):
        """
        Enable a single DAC.
        
        Args:
            pin: Pin number (0-7)
            
        Returns:
            int: Error code
        """
        if pin > 7:
            return 0
        
        bit_mask = self.read_config_register(AD5593_POWERDOWN_REF_CTRL)
        bit_mask &= ~(1 << pin)
        return self.write_register(AD5593_POWERDOWN_REF_CTRL, bit_mask)

    #  RESET
    def reset(self):
        """
        Trigger a power on reset.
        Note: Vref is reset to internal 2.5V and gain is set to 1x.
        
        Returns:
            int: Error code
        """
        result = self.write_register(AD5593_SW_RESET, 0x0DAC)
        self._Vref = 2.5
        self._gain = 1
        return result

    #  LOW LEVEL ACCESS
    def write_register(self, reg, data):
        """
        Write to a register (low level access).
        
        Args:
            reg: Register address
            data: 16-bit data to write
            
        Returns:
            int: Error code (0 on success)
        """
        try:
            if self._bus is None:
                self._bus = SMBus(self._bus_number)
            
            # Write register address and 16-bit data (MSB first)
            self._bus.write_i2c_block_data(
                self._address,
                reg,
                [(data >> 8) & 0xFF, data & 0xFF]
            )
            self._error = AD5593R_OK
            return self._error
        except (IOError, OSError) as e:
            self._error = AD5593R_I2C_ERROR
            return self._error

    def read_io_register(self, reg):
        """
        Read from an IO register (low level access).
        
        Args:
            reg: Register address
            
        Returns:
            int: 16-bit register value, 0 on error
        """
        try:
            if self._bus is None:
                self._bus = SMBus(self._bus_number)
            
            # read_i2c_block_data writes the register then reads the data
            data = self._bus.read_i2c_block_data(self._address, reg, 2)
            self._error = AD5593R_OK
            return (data[0] << 8) | data[1]
        except (IOError, OSError) as e:
            self._error = AD5593R_I2C_ERROR
            return 0

    def read_config_register(self, reg):
        """
        Read from a config register (low level access).
        
        Args:
            reg: Register address
            
        Returns:
            int: 16-bit register value, 0 on error
        """
        try:
            if self._bus is None:
                self._bus = SMBus(self._bus_number)
            
            # Write register address with GPIO_READ_CONFIG prefix
            config_reg = AD5593_GPIO_READ_CONFIG | reg
            # read_i2c_block_data writes the register then reads the data
            data = self._bus.read_i2c_block_data(self._address, config_reg, 2)
            self._error = AD5593R_OK
            return (data[0] << 8) | data[1]
        except (IOError, OSError) as e:
            self._error = AD5593R_I2C_ERROR
            return 0

    def get_last_error(self):
        """
        Get the last error code.
        
        Returns:
            int: Error code
        """
        return self._error

    def close(self):
        """
        Close the I2C bus connection.
        """
        if self._bus is not None:
            self._bus.close()
            self._bus = None


#  -- END OF FILE --

