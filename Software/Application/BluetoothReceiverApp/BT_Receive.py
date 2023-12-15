from bluepy.btle import UUID, Peripheral, DefaultDelegate
import struct

class TempReadDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)

    def handleNotification(self, cHandle, data):
        # Handle received notifications
        temperature = struct.unpack('<f', data)[0]  # Interpret data as a float
        print(f"Temperature: {temperature:.5f}")  # Print temperature with 5 decimal places

device_address = '28:CD:C1:03:F0:24'

TEMP_READ_UUID = "181a"
CHARACTERISTIC_UUID = "00002a6e-0000-1000-8000-00805f9b34fb" 

try:
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
    descriptor = characteristic.getDescriptors(forUUID=0x2902)[0]
    descriptor.write(b"\x01\x00", True)

    # Set up notification handling
    print("Setting up notification handling...")
    delegate = TempReadDelegate()
    peripheral.setDelegate(delegate)

    # Listen for notifications
    while True:
        if peripheral.waitForNotifications(1.0):
            # handleNotification will be called
            continue

except Exception as e:
    print(f"Error: {e}")
    peripheral.disconnect()
