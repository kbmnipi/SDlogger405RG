import struct
import zlib
import argparse
import datetime
import numpy as np
import matplotlib.pyplot as plt

def parse_file(f):
    data = f.read()
    if len(data) < 256 + 16:
        raise ValueError("File too short")

    # 1. Читаем заголовок
    header_bytes = data[:256]
    if header_bytes[0:4] != b'VIBL':
        raise ValueError("Invalid VIBL magic")
    
    header = struct.unpack('<4sHIBB16sbfhhhQII201s', header_bytes)
    
    crc_bytes = bytearray(header_bytes[:252])
    crc_bytes[51:55] = b'\x00\x00\x00\x00'
    if zlib.crc32(crc_bytes) != header[13]:
        print(f"⚠️ Warning: Header CRC mismatch!")
    else:
        print("✅ Header CRC: OK")

    header_dict = {
        'version': header[1], 'sample_rate': header[2], 'axes': header[3],
        'bytes_per_sample': header[4], 'sensor': header[5].strip(b'\x00').decode('ascii', errors='ignore'),
        'range_g': header[6], 'gain': header[7], 'offset_x': header[8], 'offset_y': header[9], 'offset_z': header[10],
        'start_ts': header[11], 'record_count': header[12], 'header_crc': header[13]
    }

    # 2. Отрезаем трейлер (последние 16 байт) и парсим данные
    payload_data = data[256:-16]
    trailer_bytes = data[-16:]
    
    chunks = []
    pos = 0
    while pos < len(payload_data):
        if len(payload_data) - pos < 6:
            raise ValueError("Unexpected EOF in chunk header")
        
        chunk_hdr = payload_data[pos:pos+6]
        pos += 6
        seq, count = struct.unpack('<IH', chunk_hdr)
        
        if count > 2048:
            raise ValueError(f"Chunk {seq} has invalid sample_count {count}")
            
        if len(payload_data) - pos < count * 6:
            raise ValueError(f"Unexpected EOF inside chunk {seq} data")
            
        data_bytes = payload_data[pos:pos+count*6]
        pos += count * 6
        
        records = []
        for i in range(count):
            x, y, z = struct.unpack('<hhh', data_bytes[i*6:(i+1)*6])
            records.append({'x': x, 'y': y, 'z': z})
            
        chunks.append({'seq_number': seq, 'sample_count': count, 'records': records})

    # 3. Читаем трейлер
    end_ts, total_records, file_crc32 = struct.unpack('<QII', trailer_bytes)
    calc_crc = zlib.crc32(payload_data) # Python zlib делает всё правильно автоматически
    
    trailer_dict = {
        'end_ts': end_ts, 'total_records': total_records,
        'file_crc32': file_crc32, 'calc_file_crc': calc_crc
    }
    return header_dict, chunks, trailer_dict

def format_ts(ts_ms):
    if ts_ms == 0: return "Unknown"
    dt = datetime.datetime.fromtimestamp(ts_ms / 1000.0, tz=datetime.timezone.utc)
    return dt.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3] + " UTC"

def main():
    parser = argparse.ArgumentParser(description='Parse VIBL binary files.')
    parser.add_argument('file', type=str, help='Path to the VIBL file')
    parser.add_argument('--plot', action='store_true', help='Plot the data using matplotlib')
    args = parser.parse_args()

    with open(args.file, 'rb') as f:
        try:
            header, chunks, trailer = parse_file(f)
        except Exception as e:
            print(f"❌ Parsing error: {e}")
            return

    print(f"\n--- HEADER ---")
    print(f"Sensor: {header['sensor']} | Sample Rate: {header['sample_rate']} Hz")
    print(f"Record Count (Header): {header['record_count']}")
    
    print(f"\n--- PAYLOAD ---")
    print(f"Number of Chunks: {len(chunks)}")
    print(f"Total Records (Chunks): {sum(c['sample_count'] for c in chunks)}")
    
    gaps = False
    for prev, curr in zip(chunks[:-1], chunks[1:]):
        if curr['seq_number'] - prev['seq_number'] > 1:
            gaps = True
            print(f"  ⚠️ Gap detected! Seq jumped from {prev['seq_number']} to {curr['seq_number']}")
    print(f"Gaps Found: {'Yes' if gaps else 'No'}")
    
    if trailer:
        print(f"\n--- TRAILER ---")
        print(f"Total Records (Trailer): {trailer['total_records']}")
        print(f"File CRC32: {trailer['file_crc32']:08X}")
        print(f"Calc CRC32: {trailer['calc_file_crc']:08X} -> {'✅ MATCH' if trailer['file_crc32'] == trailer['calc_file_crc'] else '❌ MISMATCH'}")
    
    if args.plot and chunks:
        print("\nRendering plot...")
        x_data = np.array([r['x'] for c in chunks for r in c['records']])
        y_data = np.array([r['y'] for c in chunks for r in c['records']])
        z_data = np.array([r['z'] for c in chunks for r in c['records']])

        fig, axs = plt.subplots(3, 1, figsize=(10, 8))
        axs[0].plot(x_data, color='red'); axs[0].set_title('X Axis'); axs[0].grid(True)
        axs[1].plot(y_data, color='green'); axs[1].set_title('Y Axis'); axs[1].grid(True)
        axs[2].plot(z_data, color='blue'); axs[2].set_title('Z Axis'); axs[2].grid(True)
        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    main()