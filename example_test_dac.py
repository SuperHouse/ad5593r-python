#!/usr/bin/env python3
#
#    FILE: example_test_dac.py
#  AUTHOR: Converted from Arduino example
# PURPOSE: Test DAC mode
#     URL: https://github.com/RobTillaart/AD5593R
#
#  Outputs should have different output voltage
#  from zero to full scale in 8 steps.
#

import time
from AD5593R import AD5593R, AD5593R_LDAC_DIRECT, AD5593R_LIB_VERSION

# Create AD5593R instance with I2C address 0x10
# Default bus number is 1 (for Raspberry Pi)
ad = AD5593R(0x10, bus_number=1)

print("AD5593R Test DAC")
print(f"AD5593R_LIB_VERSION: {AD5593R_LIB_VERSION}")
print()

# Initialize device
print(f"Connect: {ad.is_connected()}")
print(f"Address: 0x{ad.get_address():02X}")
print()

# Set all eight pins to DAC mode
ad.set_dac_mode(0xFF)

# Use internal Vref 2.5V
ad.set_external_reference(False, 5.0)

# Do not double the output
ad.set_dac_range_2x(False)

# COPY input register direct to DAC
# Must be set after set_external_reference()
ad.set_ldac_mode(AD5593R_LDAC_DIRECT)

# Read back default values
print("Read back default values (expect all zeros):")
for pin in range(8):
    value = ad.read_dac(pin)
    print(f"0x{value:03X}", end="\t")
print()
print()

try:
    while True:
        # Write 8 different values
        print("Writing DAC values:")
        for pin in range(8):
            # Expect increasing values: 0, 585, 1170, 1755, 2340, 2925, 3510, 4095
            value = (pin * 4095) // 7
            result = ad.write_dac(pin, value)
            print(f"{result}", end="\t")
        print()

        # Read back
        print("Read back values:")
        for pin in range(8):
            value = ad.read_dac(pin)
            print(f"0x{value:03X}", end="\t")
        print()
        print()

        time.sleep(1)

except KeyboardInterrupt:
    print("\nExiting...")
    ad.close()

