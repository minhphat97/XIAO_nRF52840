import threading
import asyncio
import time
import struct
import matplotlib.pyplot as plt
from bleak import BleakClient
import queue
import sys

# Global flag to stop the program
stop_flag = False

# Data queue for plotting
data_queue = queue.Queue()

# Add your Bluetooth address and UUIDs here
XIAO_MAC_ADDRESS = "6A:FC:E9:88:60:5A"
CHARACTERISTIC_UUID_X = "00002a56-0000-1000-8000-00805f9b34fb"
CHARACTERISTIC_UUID_Y = "00002a57-0000-1000-8000-00805f9b34fb"
CHARACTERISTIC_UUID_Z = "00002a58-0000-1000-8000-00805f9b34fb"
times = []
values_X = []
values_Y = []
values_Z = []

def on_key_press(event):
    global stop_flag
    while True:
        if event.key == 'q':
            print("Exiting...")
            stop_flag = True
            plt.close()  # Close the plot window
            sys.exit()
        time.sleep(0.1)  # Check every 100ms
        

async def read_xiao_data():
    async with BleakClient(XIAO_MAC_ADDRESS) as client:
        print(f"Connected to {XIAO_MAC_ADDRESS}")
        start_time = time.time()

        while not stop_flag:  # Check the stop flag to exit the loop
            try:
                data_X = await client.read_gatt_char(CHARACTERISTIC_UUID_X)
                data_Y = await client.read_gatt_char(CHARACTERISTIC_UUID_Y)
                data_Z = await client.read_gatt_char(CHARACTERISTIC_UUID_Z)

                # Unpack the data
                float_value_X = struct.unpack('<f', data_X)[0]
                float_value_Y = struct.unpack('<f', data_Y)[0]
                float_value_Z = struct.unpack('<f', data_Z)[0]

                print(f"Received float value_X: {float_value_X}")
                print(f"Received float value_Y: {float_value_Y}")
                print(f"Received float value_Z: {float_value_Z}")

                # Add data to the queue
                current_time = time.time() - start_time
                data_queue.put((current_time, float_value_X, float_value_Y, float_value_Z))

            except struct.error:
                print("Error decoding data!")

            await asyncio.sleep(0.5)  # Slight delay to prevent overwhelming the system


def run_event_loop():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(read_xiao_data())

def start_data_thread():
    data_thread = threading.Thread(target=run_event_loop)
    data_thread.start()

def start_plot_thread():
    global ax, line
    fig, ax = plt.subplots()

    # Initialize three lines for X, Y, and Z values
    line_X, = ax.plot([], [], lw=2, label="Value X") # Line for X values
    line_Y, = ax.plot([], [], lw=2, label="Value Y")  # Line for Y values
    line_Z, = ax.plot([], [], lw=2, label="Value Z")  # Line for Z values

    # Add axis labels and title
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Accelerometer Values")
    ax.set_title("Real-Time Accelerometer Data")

    # Add a legend to differentiate the lines
    ax.legend()

    # Set up the key press event handler
    fig.canvas.mpl_connect('key_press_event', on_key_press)

    while not stop_flag:  # Keep updating the plot until stop flag is set
        if not data_queue.empty():
            current_time, value_X, value_Y, value_Z = data_queue.get()
            # print(f"Updating plot with time: {current_time}, value_X: {value_X}")
            times.append(current_time)
            values_X.append(value_X)
            values_Y.append(value_Y)
            values_Z.append(value_Z)

            # Keep the data to a certain length to avoid excessive memory usage
            if len(times) > 100:
                times.pop(0)
                values_X.pop(0)
                values_Y.pop(0)
                values_Z.pop(0)

            # Update the plot data
            line_X.set_data(times, values_X)
            line_Y.set_data(times, values_Y)
            line_Z.set_data(times, values_Z)

            # Recalculate axis limits and redraw the plot
            ax.relim()  # Recalculate limits
            ax.autoscale_view()  # Automatically scale the view
            plt.draw()  # Redraw the plot

        plt.pause(0.5)  # Pause to allow for plot updates

# Start both threads
data_thread = threading.Thread(target=start_data_thread)
plot_thread = threading.Thread(target=start_plot_thread)

data_thread.start()
plot_thread.start()

# Wait for both threads to finish
data_thread.join()
plot_thread.join()
