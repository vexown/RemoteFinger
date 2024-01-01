from bluepy.btle import UUID, Peripheral
import struct
import time
import sys

PRINT_ENABLED = 1

def connect_to_bluetooth(max_attempts=5):
    global service
    global characteristic

    device_address = '28:CD:C1:03:F0:24'
    TEMP_READ_UUID = "181a"
    CHARACTERISTIC_UUID = "00002a6e-0000-1000-8000-00805f9b34fb"

    attempts = 0
    connected = False
    while attempts < max_attempts and not connected:
        try:
            print(f"Connecting to the device... Attempt {attempts + 1}")
            # Establish connection
            peripheral = Peripheral(device_address)

            # Discover services and characteristics
            print("Discovering services and characteristics...")
            service_uuid = UUID(TEMP_READ_UUID)
            characteristic_uuid = UUID(CHARACTERISTIC_UUID)

            service = peripheral.getServiceByUUID(service_uuid)
            characteristic = service.getCharacteristics(characteristic_uuid)[0]

            connected = True
        except Exception as e:
            print(f"Connection attempt failed: {e}")
            attempts += 1
            time.sleep(2)  # Wait for a while before retrying

    if connected:
        # Bluetooth connection successful, proceed with your game or other logic
        print("Bluetooth connection successful. Starting the game or other logic here...")
        # Call your game logic or other functionalities here
    else:
        print("Failed to connect to Bluetooth after multiple attempts.")
        sys.exit(1)
            
def get_accel_data():
    global accelerometer_data
    # Read the characteristic value
    data = characteristic.read()
    # Unpack the data into a tuple of floats
    MPU6050_array = struct.unpack('<' + 'f' * (len(data) // 4), data)
    # Get the first three values as the accelerometer data
    accelerometer_data = MPU6050_array[:3]
    
    if PRINT_ENABLED == 1:
        #print("x:")
        #print(accelerometer_data[0])
        #print("y")
        #print(accelerometer_data[1])
        #print("z")
        #print(accelerometer_data[2])
        #print(" ")

        print(accelerometer_data[0], ",", accelerometer_data[1], ",", accelerometer_data[2])



connect_to_bluetooth()

while True:
    get_accel_data()
    time.sleep(0.2)
    
    
    