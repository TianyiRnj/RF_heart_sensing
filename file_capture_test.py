import pyshark
import struct
from scipy.io import savemat
import numpy as np

# Specify the file path
file_path = r"D:\CMU\24 Fall\Heart Sensing Using RF signals RP\save_data_iso_6n.pcapng"
capture = pyshark.FileCapture(file_path)

# Specify the output .mat file path
output_mat_file = r"D:\CMU\24 Fall\Heart Sensing Using RF signals RP\output_data_iso_6.mat"

###
# Define the target advertising address
# Add more target addresses
target_addresses = ["fe:a8:9d:6a:69:6a", "ed:48:ac:a7:ed:62"]
###
gabage_data = "99:99:99:99:99:99:99:99:99:99:99:99:99:99:99:99:99:99:99:99"
data_suffix = "99:99"

# Check if capture is initialized
print(f"Capture object: {capture}")

# Dictionary to hold the data for each address
output_data = {}

# Initialize a structure for each address in target_addresses
for address in target_addresses:
    # Replace colons with underscores to create valid field names for MATLAB
    valid_address = address.replace(':', '')
    output_data[valid_address] = {
        'TS_Receiver': [],
        'TS_Local': [],
        'Accel_X': [],
        'Accel_Y': [],
        'Accel_Z': [],
        'PPG': []
    }

# Loop through each target address
for address in target_addresses:
    # Initialize previous packet info 
    prev_packet_data = None
    
    # Process packets
    for packet in capture:
        # Check if the packet has BTLE layer
        if 'btle' in packet:
            # Extract advertising address
            advertising_address = packet.btle.get_field_by_showname('Advertising Address')
            # Try extract advertising data
            try:
                advertising_address = packet.btle.get_field_by_showname('Advertising Address')
                if advertising_address and advertising_address.show == address:
                    advertising_data = packet.btle.get_field_by_showname('Data')
                    if advertising_data and advertising_data.show.endswith(data_suffix):
                        if advertising_data == gabage_data:
                            continue
                        # print(packet.NORDIC_BLE.get_field_by_showname('Timestamp').show)
                        # break
                        
                        # Get the current packet data
                        data = advertising_data.show.replace(':', '')
                        # Discard consecutive duplicate packets
                        if prev_packet_data == data:
                            continue
                        prev_packet_data = data
                        
                        # time = packet.sniff_time.strftime('%Y-%m-%d %H:%M:%S.%f') 
                        time_receive_str = packet.NORDIC_BLE.time.show  # microsec
                        time_receive_usec = int(time_receive_str)
                        time_receive_sec = time_receive_usec / 1_000_000
                        # print(f"Timestamp received: {time_receive_sec} seconds")
                        
                        # Extract the raw value; then convert hex string to int
                        ID = int(packet.btle.get_field_by_showname('Company ID').show, 16)
                        swapped_id = format(((ID & 0xFF) << 8) | (ID >> 8), '04x')
                        
                        # Remove colons from data
                        data = advertising_data.show.replace(':', '')

                        # Parsed data
                        accel_X = int(swapped_id, 16)
                        accel_Y = int(data[:4], 16)
                        accel_Z = int(data[4:8], 16)
                        PPG = int(data[8:12], 16)
                        time_sent_hex = int(data[12:18], 16)
                        
                        # flt_accelx = struct.unpack('>f', struct.pack('>I', accel_X))[0] / 1000
                        # flt_accely = struct.unpack('>f', struct.pack('>I', accel_Y))[0] / 1000
                        # flt_accelz = struct.unpack('>f', struct.pack('>I', accel_Z))[0] / 1000
                        flt_accelx = accel_X / 1000
                        flt_accely = accel_Y / 1000
                        flt_accelz = accel_Z / 1000
                        PPG_value = float(PPG)
                        time_sent = float(time_sent_hex) / 1_000
                        
                        # print(swapped_id, data[:4], data[4:8], data[8:12], data[12:18])
                        # print(accel_X, accel_Y, accel_Z, PPG, time_sent_hex)
                        # print(flt_accelx, flt_accely, flt_accelz, PPG_value, time_sent)

                        # print(f"Timestamp sent: {time_sent_hex} ms in HEX")
                        # print(f"Timestamp sent: {time_sent} seconds")
                        
                        # Append the parsed data to the output dictionary for the specific address
                        valid_address = address.replace(':', '')
                        output_data[valid_address]['TS_Receiver'].append(time_receive_sec)
                        output_data[valid_address]['TS_Local'].append(time_sent)
                        output_data[valid_address]['Accel_X'].append(flt_accelx)
                        output_data[valid_address]['Accel_Y'].append(flt_accely)
                        output_data[valid_address]['Accel_Z'].append(flt_accelz)
                        output_data[valid_address]['PPG'].append(PPG_value)
                        
                        # print(f"Packet: Address-{advertising_address}:\n Accel-X-{flt_accelx}; Accel-Y-{flt_accely}; Accel-Z-{flt_accelz}; PPG-{PPG}")
                        # print(f"Packet: Address-{advertising_address}:\n Accel-X-{accel_X}; Accel-Y-{accel_Y}; Accel-Z-{accel_Z}; PPG-{PPG}")
                        
                        # Parsed data
                        accel_X = int(data[18:22], 16)
                        accel_Y = int(data[22:26], 16)
                        accel_Z = int(data[26:30], 16)
                        PPG = int(data[30:34], 16)
                        time_sent_hex = int(data[34:36], 16)
                        
                        flt_accelx = accel_X / 1000
                        flt_accely = accel_Y / 1000
                        flt_accelz = accel_Z / 1000
                        PPG_value = float(PPG)
                        time_sent = float(time_sent_hex) / 1_000 + time_sent

                        # print(flt_accelx, flt_accely, flt_accelz, PPG_value, time_sent)
                        # print(f"Timestamp sent: {time_sent_hex} ms in HEX")
                        # print(f"Timestamp sent: {time_sent} seconds")
                        
                        # Append the parsed data to the output dictionary for the specific address
                        output_data[valid_address]['TS_Receiver'].append(time_receive_sec)
                        output_data[valid_address]['TS_Local'].append(time_sent)
                        output_data[valid_address]['Accel_X'].append(flt_accelx)
                        output_data[valid_address]['Accel_Y'].append(flt_accely)
                        output_data[valid_address]['Accel_Z'].append(flt_accelz)
                        output_data[valid_address]['PPG'].append(PPG_value)
                        
            except AttributeError as e:
                print(f"Field access error: {e}")

# Close the capture object
capture.close()

# Calculate the frequency based on TS_Receiver for each address
for address in target_addresses:
    valid_address = address.replace(':', '')
    ts_receiver = output_data[valid_address]['TS_Local']
    
    if len(ts_receiver) > 1:
        # Calculate time differences between consecutive timestamps
        time_diffs = np.diff(ts_receiver)
        np.set_printoptions(threshold=np.inf)
        # print(time_diffs)
        avg_time_diff = np.mean(time_diffs)
        print(f"Average Data Arriving time for {address}: {avg_time_diff} sec")
        # Calculate the frequency
        frequency = 1.0 / avg_time_diff
        print(f"Data frequency for {address}: {frequency} Hz")

# Save the parsed data to a .mat file
savemat(output_mat_file, output_data)

print(f"Data has been saved to {output_mat_file}")
