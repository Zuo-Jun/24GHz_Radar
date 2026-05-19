import struct
import time

# C sends RADAR_PROTO_SOF(0x55AA) via put_u16_le(), so the wire order is AA 55.
SOF = b"\xAA\x55"
PORT = "COM8"
BAUDRATE = 115200


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc


def build_ack(dev_id: int, seq: int) -> bytes:
    body = struct.pack("<BHHB", 0x81, dev_id, seq, 0)
    crc = crc16_modbus(body)
    return SOF + body + struct.pack("<H", crc)


def main() -> None:
    import serial

    ser = serial.Serial(PORT, BAUDRATE, timeout=0.5)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    buffer = bytearray()

    print(f"Listening on {PORT} at {BAUDRATE} baud")

    while True:
        chunk = ser.read(128)
        if not chunk:
            continue

        buffer += chunk

        while True:
            idx = buffer.find(SOF)
            if idx < 0:
                buffer.clear()
                break

            if idx > 0:
                del buffer[:idx]

            # Min frame: SOF(2) + TYPE(1) + DEV_ID(2) + SEQ(2) + LEN(1) + CRC(2)
            if len(buffer) < 10:
                break

            frame_type = buffer[2]
            dev_id = struct.unpack_from("<H", buffer, 3)[0]
            seq = struct.unpack_from("<H", buffer, 5)[0]
            payload_len = buffer[7]

            total_len = 2 + 1 + 2 + 2 + 1 + payload_len + 2
            if len(buffer) < total_len:
                break

            frame = bytes(buffer[:total_len])
            del buffer[:total_len]

            body = frame[2:-2]
            recv_crc = struct.unpack_from("<H", frame, total_len - 2)[0]
            calc_crc = crc16_modbus(body)

            if recv_crc != calc_crc:
                print("CRC error:", frame.hex(" "))
                continue

            if frame_type == 0x01:
                payload = frame[8:-2]

                if payload_len >= 10:
                    status, wl_mm, wv_mms = struct.unpack_from("<Hii", payload, 0)
                    print(
                        f"RX DEV={dev_id}, SEQ={seq}, ST=0x{status:04X}, "
                        f"WL={wl_mm / 1000:.3f} m, WV={wv_mms / 1000:.3f} m/s"
                    )
                else:
                    print(f"RX DEV={dev_id}, SEQ={seq}, short payload len={payload_len}")

                ack = build_ack(dev_id, seq)
                # time.sleep(0.005)  # 预留一点时间给单片机处理，避免ACK太快被丢弃
                ser.write(ack)
                ser.flush()
                print("TX ACK:", ack.hex(" "))

            elif frame_type == 0x81:
                print("RX ACK:", frame.hex(" "))
            else:
                print(f"RX unknown type=0x{frame_type:02X}:", frame.hex(" "))


if __name__ == "__main__":
    main()
