import serial
import serial.tools.list_ports
import sys
import pandas as pd
from scipy.io import savemat  
import matplotlib.pyplot as plt
from plot_utils import Plot, Log_Message, create_output_folders, get_data_path, get_test_path, message_to_string
import os
from pathlib import Path

# =================================================================
#                       FUNCTION DEFINITIONS
# =================================================================

def checkSerialPort(selected_port):
    """ Checks if the selected serial port is available.\n
    If not, kill the program """
    portList = []
    availablePorts = serial.tools.list_ports.comports()
    
    for port in availablePorts:
        portList.append(str(port))
        print(str(port))

    for port in portList:
        if selected_port in port:
            print(f"Selected port: {selected_port}")
            return selected_port
        
    sys.exit(f"{Log_Message['ERROR']}: The selected port is not available") 


# =================================================================
#                         MAIN PROGRAM
# =================================================================
# Load configuration
plot = Plot()

# Check that the selected port is available
port = checkSerialPort(plot.config['serial']['port'])

# Create the serial object with the selected configuration
serial_device = serial.Serial()
serial_device.port = port
serial_device.baudrate = plot.config['serial']['baudrate']
serial_device.open()

# Variables to store metrics
avg_sample_time  = 0.0
total_sample_time = 0
total_samples = 0
previous_time = None

# Start the loop to read the message
data = []
data_names = plot.config['main']['data_labels']
expected_length = len(plot.config['main']['data'])

try:
    while True:
        if serial_device.in_waiting:
            # Data processing
            received_message = serial_device.readline().decode('utf-8')  # Decoding the received bytes
            received_values = received_message.split(",")

            if len(received_values) != expected_length:
                print(f"{Log_Message['WARNING']}: Expected {expected_length} values but received {len(received_values)}. Skipping this message.")
                continue
            
            print(message_to_string(data_names, received_values))
                
            float_values = [float(val) for val in received_values] 
            current_time_ms = float_values[0]
            float_values[0] = round(float_values[0]/1000 , 2)     # Store time in seconds 
            data.append({data_names[i]: float_values[i] for i in range(expected_length)})

            # Data status metrics
            if previous_time is not None:
                sample_time = current_time_ms - previous_time
                total_sample_time += sample_time

            total_samples += 1
            previous_time = current_time_ms

            if(current_time_ms / 1000 > plot.config['main']['test_time'][1]):
                break

except KeyboardInterrupt:
    print(f"\n{Log_Message['ERROR']}: Stopping to save data.")

# --- After data collection ---
test_name = plot.config['main']['test_name']
test_path = get_test_path(test_name)

if total_samples > plot.config['serial']['min_samples_to_store_data']:
    avg_sample_time  = round(total_sample_time / total_samples, 2)

    if data:
        print(f"{Log_Message['INFO']}: Data storage completed successfully.")                
        df = pd.DataFrame(data)
        df = df.drop_duplicates()

        # Create folders
        create_output_folders(test_name)

        # Save DataFrame to CSV
        df.to_csv(get_data_path(test_name) / "data.csv", index=False)

        # Save DataFrame to .mat file
        mat_data = {col: df[col].values for col in df.columns}
        savemat(get_data_path(test_name) / "data.mat", mat_data)

        print(f"{Log_Message['INFO']}: Saving data to:\n- {get_data_path(test_name) / 'data.csv'}\n- {get_data_path(test_name) / 'data.mat'}")

        # Plotting
        if plot.config['main']['test_time']:
            df = df[(df["time"] >= plot.config['main']['test_time'][0]) & (df["time"] <= plot.config['main']['test_time'][1])]

        # Output path
        if plot.config['main']['output_path']:
            output_path = Path(plot.config['main']['output_path']) / plot.config['main']['test_name'] / 'Plots'
            os.makedirs(output_path, exist_ok=True)
            print(f"{Log_Message['INFO']}: Using custom output path {output_path}")
        else:
            output_path = output_path = test_path / 'Plots'  
            print(f"{Log_Message['INFO']}: Using default output path {output_path}")

        for i, col in enumerate(data_names):  
            if i > 0:  # skip 'time'
                plt.figure(figsize=(plot.width, plot.height))
                line_width = plot.config['plot'].get('line_width', 1.5)
                plt.plot(df["time"], df[col], label=col, linewidth=line_width)
                plt.ylabel(plot.config['plot']['y_label_multiple'][i-1])
        
                # --- Set y-axis limits if defined
                if 'y_limits_multiple' in plot.config['plot']:
                    y_limits_list = plot.config['plot']['y_limits_multiple']
                    if i-1 < len(y_limits_list):
                        plt.ylim(y_limits_list[i-1])
        
                # --- Set y-axis ticks if defined
                if 'y_ticks_multiple' in plot.config['plot']:
                    y_ticks_list = plot.config['plot']['y_ticks_multiple']
                    if i-1 < len(y_ticks_list):
                        plt.yticks(y_ticks_list[i-1])
        
                ax = plt.gca()
                plot.set_plot_style(ax, df)
        

                plot_path = output_path / f"{col}.{plot.config['main']['output_format']}"
                plt.savefig(plot_path, bbox_inches='tight')
                print(f"{Log_Message['INFO']}: {plot_path} stored successfully.")
                plt.close()


    

serial_device.close()
print(f"\n\n{Log_Message['INFO']}: Program finished with {total_samples} samples and {avg_sample_time} average sample time (ms).\n\n")
