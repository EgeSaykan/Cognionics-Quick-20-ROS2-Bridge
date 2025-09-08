import sqlite3
from datetime import datetime
import sys
import struct

# Replace with your actual bag path
if len(sys.argv) != 2:
    print(f"Usage: {sys.argv[0]} <path_to_db>")
    sys.exit(1)

db_path = sys.argv[1]

conn = sqlite3.connect(db_path)
cursor = conn.cursor()

def decode_string_msg(data):
    """Decode std_msgs/String message"""
    try:
        # Skip CDR header (4 bytes) and string length (4 bytes)
        if len(data) > 8:
            string_length = struct.unpack('<I', data[4:8])[0]
            string_data = data[8:8+string_length-1]  # -1 to remove null terminator
            return string_data.decode('utf-8', errors='ignore')
    except:
        pass
    return f"Raw bytes: {data}"

def decode_neuro_frame(data):
    """Attempt to decode NeuroFrame message with comprehensive analysis"""
    try:
        offset = 0
        decoded_info = {}
        
        # Skip CDR header (4 bytes)
        offset += 4
        
        # Try to extract timestamp (assuming it's early in the message)
        if len(data) >= offset + 8:
            timestamp_candidate = struct.unpack('<Q', data[offset:offset+8])[0]
            if timestamp_candidate > 1e15:  # Reasonable nanosecond timestamp
                decoded_info['timestamp'] = timestamp_candidate
                offset += 8
        
        # Extract readable strings (electrode names, device info, etc.)
        readable_parts = []
        electrode_names = []
        i = offset
        while i < len(data) - 1:
            if data[i] >= 32 and data[i] <= 126:  # Printable ASCII
                start = i
                while i < len(data) and data[i] >= 32 and data[i] <= 126:
                    i += 1
                if i - start > 1:  # Include strings longer than 1 char
                    text = data[start:i].decode('ascii')
                    readable_parts.append(text)
                    # Common EEG electrode names
                    if text in ['F7', 'Fp1', 'Fp2', 'F8', 'F3', 'Fz', 'F4', 'C3', 'Cz', 'C4', 'P3', 'Pz', 'P4', 'P7', 'P8', 'T3', 'T4', 'O1', 'O2']:
                        electrode_names.append(text)
            else:
                i += 1
        
        # More comprehensive search for float data
        all_floats = []
        non_zero_floats = []
        reasonable_eeg_floats = []  # Values that could be EEG data
        
        # Scan entire message for float values
        for start_pos in range(4, len(data) - 3, 1):  # Start after CDR header, check every byte
            try:
                float_val = struct.unpack('<f', data[start_pos:start_pos+4])[0]
                # Check if it's a valid float
                if not (float('inf') == float_val or float('-inf') == float_val or 
                       float_val != float_val):  # Not inf or NaN
                    all_floats.append((start_pos, float_val))
                    if abs(float_val) > 0.001:  # Non-zero values
                        non_zero_floats.append((start_pos, float_val))
                        # Filter for reasonable EEG values (typically -1000 to +1000 microvolts)
                        if -1000 < float_val < 1000:
                            reasonable_eeg_floats.append((start_pos, float_val))
            except:
                continue
        
        # Look for the largest continuous section of reasonable EEG values
        if reasonable_eeg_floats:
            # Find groups of consecutive positions (at 4-byte intervals)
            eeg_groups = []
            current_group = [reasonable_eeg_floats[0]]
            
            for i in range(1, len(reasonable_eeg_floats)):
                pos, val = reasonable_eeg_floats[i]
                prev_pos = current_group[-1][0]
                
                # If positions are 4 bytes apart, continue the group
                if pos - prev_pos == 4:
                    current_group.append((pos, val))
                else:
                    # Start a new group
                    if len(current_group) >= 10:  # Keep groups with at least 10 values
                        eeg_groups.append(current_group)
                    current_group = [(pos, val)]
            
            # Don't forget the last group
            if len(current_group) >= 10:
                eeg_groups.append(current_group)
            
            # Use the largest group as our EEG data
            best_eeg_group = max(eeg_groups, key=len) if eeg_groups else []
        else:
            best_eeg_group = []
        
        # Build detailed output
        result = []
        if 'timestamp' in decoded_info:
            result.append(f"Timestamp: {decoded_info['timestamp']}")
        
        if electrode_names:
            result.append(f"Electrodes ({len(electrode_names)}): {', '.join(electrode_names)}")
        
        # Report on EEG data analysis
        if best_eeg_group:
            eeg_values = [val for pos, val in best_eeg_group]
            result.append(f"EEG data ({len(eeg_values)} samples) starting at offset {best_eeg_group[0][0]}: [{eeg_values[0]:.3f}, {eeg_values[1]:.3f}, {eeg_values[2]:.3f}, ..., {eeg_values[-1]:.3f}]")
        elif reasonable_eeg_floats:
            # Show first few reasonable values even if not consecutive
            sample_values = [val for pos, val in reasonable_eeg_floats[:19]]
            result.append(f"Sample EEG values: {[round(v, 3) for v in sample_values]}")
        elif non_zero_floats:
            result.append(f"Non-zero floats found: {len(non_zero_floats)} (first few: {[round(f[1], 3) for f in non_zero_floats[:5]]})")
        
        if 'uV' in readable_parts:
            result.append("Units: microvolts")
        
        # Debug info
        result.append(f"Debug: Total floats: {len(all_floats)}, Non-zero: {len(non_zero_floats)}, Reasonable EEG: {len(reasonable_eeg_floats)}")
        
        return "; ".join(result) if result else f"Binary data ({len(data)} bytes)"
        
    except Exception as e:
        return f"Decode error: {str(e)} - Raw data size: {len(data)} bytes"

cursor.execute("""
    SELECT 
        m.timestamp, 
        t.name as topic_name, 
        t.type as topic_type,
        length(m.data) as data_size,
        m.data
    FROM messages m 
    JOIN topics t ON m.topic_id = t.id 
    ORDER BY m.timestamp DESC 
    LIMIT 5
""")

print("Last 5 entries:")
for row in cursor.fetchall():
    timestamp_ns = row[0]
    timestamp_s = timestamp_ns / 1e9
    human_time = datetime.fromtimestamp(timestamp_s)
    data_content = row[4]
    topic_type = row[2]
    
    print(f"Time: {human_time}, Topic: {row[1]}, Type: {topic_type}, Size: {row[3]} bytes")
    
    # Decode based on message type
    if 'String' in topic_type:
        decoded_data = decode_string_msg(data_content)
        print(f"Decoded: {decoded_data}")
    elif 'NeuroFrame' in topic_type:
        decoded_data = decode_neuro_frame(data_content)
        print(f"Decoded: {decoded_data}")
    else:
        print(f"Raw Data: {data_content}")
    
    print("-" * 80)

conn.close()