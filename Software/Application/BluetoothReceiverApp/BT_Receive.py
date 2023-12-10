from bluepy.btle import UUID, Peripheral, DefaultDelegate
import time

class TempReadDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)

device_address = '28:CD:C1:03:F0:24'

TEMP_READ_UUID = "181a"
CHARACTERISTIC_UUID = "00002a6e-0000-1000-8000-00805f9b34fb" 

# Connect to the device
print("Connecting to the device...")
peripheral = Peripheral(device_address)

# Discover services and characteristics
print("Discovering services and characteristics...")
service_uuid = UUID(TEMP_READ_UUID)
characteristic_uuid = UUID(CHARACTERISTIC_UUID)

service = peripheral.getServiceByUUID(service_uuid)
characteristic = service.getCharacteristics(characteristic_uuid)[0]

# Enable notifications on the characteristic
print("Enabling notifications on the characteristic...")
peripheral.writeCharacteristic(characteristic.valHandle + 1, b"\x01\x00")

# Set up notification handling
print("Setting up notification handling...")
delegate = TempReadDelegate()
peripheral.setDelegate(delegate)

# Read the values
print("Actively reading characteristic value...")
while True:
    # Read the characteristic value
    data = characteristic.read()
    temperature = int.from_bytes(data, byteorder='little')
    print(f"Temperature: {temperature}")
    
    # Wait for some time before reading again (e.g., 5 seconds)
    time.sleep(5)

