from bluepy.btle import UUID, Peripheral, DefaultDelegate
import struct
import tkinter as tk

class MPU6050ReadDelegate(DefaultDelegate):
    def __init__(self, canvas, pointer):
        DefaultDelegate.__init__(self)
        self.canvas = canvas
        self.pointer = pointer

    def handleNotification(self, cHandle, data):
        MPU6050_array = struct.unpack('<' + 'f' * (len(data) // 4), data)
        accelerometer_data = MPU6050_array[:3]

        print("Accelerometer Data:")
        axis_labels = ['X', 'Y', 'Z']
        
        for i in range(min(3, len(accelerometer_data))):
            print(f"    Axis {axis_labels[i]}: {accelerometer_data[i]:.5f}")

        # Get the current position of the pointer
        x1, y1, x2, y2 = self.canvas.coords(self.pointer)

        # Calculate the new position of the pointer
        new_x = x1 + accelerometer_data[0] * 10
        new_y = y1 + accelerometer_data[1] * 10

        # Check if the new position is within the bounds of the canvas
        if 0 <= new_x <= 500 and 0 <= new_y <= 500:
            # Move the pointer to the new position
            self.canvas.move(self.pointer, accelerometer_data[0] * 10, accelerometer_data[1] * 10)
            self.canvas.update()


device_address = '28:CD:C1:03:F0:24'
TEMP_READ_UUID = "181a"
CHARACTERISTIC_UUID = "00002a6e-0000-1000-8000-00805f9b34fb" 

# Create a new Tkinter window
window = tk.Tk()

# Create a canvas to draw the pointer
canvas = tk.Canvas(window, width=500, height=500)
canvas.pack()

# Draw a circle on the canvas to represent the pointer
pointer = canvas.create_oval(250, 250, 260, 260, fill='black')

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
delegate = MPU6050ReadDelegate(canvas, pointer)
peripheral.setDelegate(delegate)

# Listen for notifications
while True:
    if peripheral.waitForNotifications(1.0):
        continue

# Start the Tkinter event loop
window.mainloop()
