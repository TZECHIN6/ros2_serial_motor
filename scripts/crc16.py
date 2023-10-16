def calculate_crc16(data: list[int]):
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001  # 0xA001 is the bit-reversed form of the polynomial 0x8005
                else:
                    crc >>= 1
        crc_bytes = [hex(crc & 0xFF), hex((crc >> 8) & 0xFF)]
        return crc_bytes

def main():
     data = [0x01, 0x03, 0x00, 0x26, 0x00, 0x01]
     crc = calculate_crc16(data)
     print(f'crc: {crc}')
     print('done')


if __name__ == '__main__':
     main()
