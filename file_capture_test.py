import pyshark
from scipy.io import savemat

# Specify the file path
file_path = r"D:\CMU\24 Fall\Heart Sensing Using RF signals RP\save_data_example.pcapng"
capture = pyshark.FileCapture(file_path)
# Define the target advertising address
target_addresses = ["fe:a8:9d:6a:69:6a"]
data_suffix = "99:99:99:99:99:99"

# Check if capture is initialized
print(f"Capture object: {capture}")

# Dictionary to hold the data for mat file
output_data = {
    'Accel_X': [],
    'Accel_Y': [],
    'Accel_Z': [],
    'PPG': []
}

# Process packets
for packet in capture:
    # Check if the packet has BTLE layer
    if 'btle' in packet:
        # Extract advertising address
        advertising_address = packet.btle.get_field_by_showname('Advertising Address')
        # Try extract advertising data
        try:
            advertising_address = packet.btle.get_field_by_showname('Advertising Address')
            if advertising_address and advertising_address.show in target_addresses:
                advertising_data = packet.btle.get_field_by_showname('Data')
                if advertising_data and advertising_data.show.endswith(data_suffix):    
                    # Extract the raw value; then convert hex string to int
                    ID = int(packet.btle.get_field_by_showname('Company ID').show, 16)
                    swapped_id = format(((ID & 0xFF) << 8) | (ID >> 8), '04x')
                    # Remove colons from data
                    data = advertising_data.show.replace(':', '')
                    
                    # Parsed data
                    accel_X = swapped_id + data[:4]
                    accel_Y = data[4:12]
                    accel_Z = data[12:20]
                    PPG = data[20:24]
                    
                    # Append the parsed data to the output dictionary
                    output_data['Accel_X'].append(accel_X)
                    output_data['Accel_Y'].append(accel_Y)
                    output_data['Accel_Z'].append(accel_Z)
                    output_data['PPG'].append(PPG)
                    
                    # print(f"Packet: Address-{advertising_address}:\n Accel-X-{accel_X}; Accel-Y-{accel_Y}; Accel-Z-{accel_Z}; PPG-{PPG}")
        except AttributeError as e:
            print(f"Field access error: {e}")

# Close the capture object
capture.close()

# Specify the output .mat file path
output_mat_file = r"D:\CMU\24 Fall\Heart Sensing Using RF signals RP\output_data.mat"

# Save the parsed data to a .mat file
savemat(output_mat_file, output_data)

print(f"Data has been saved to {output_mat_file}")
