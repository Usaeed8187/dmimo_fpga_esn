from gnuradio import digital
from gnuradio import blocks
import numpy as np

def calculate_crc8(data_32bit):
    """
    Calculate CRC8 for 32-bit input data
    Args:
        data_32bit (int): 32-bit integer input
    Returns:
        int: 8-bit CRC value
    """
    # Convert 32-bit integer to bytes
    data_bytes = data_32bit.to_bytes(4, byteorder='big')
    
    # Create CRC8 block
    crc8_block = digital.crc8_bb.make(check=False)
    
    # Convert input to numpy array of bytes
    input_data = np.frombuffer(data_bytes, dtype=np.uint8)
    
    # Process the data through CRC8 block
    output_data = crc8_block.process_byte(input_data)
    
    # The last byte is the CRC8
    crc8_value = output_data[-1]
    
    return crc8_value

# Example usage
if __name__ == "__main__":
    # Test with a sample 32-bit number
    test_data = 0x12345678
    crc = calculate_crc8(test_data)
    print(f"CRC8 for 0x{test_data:08X} is 0x{crc:02X}")