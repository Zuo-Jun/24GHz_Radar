import struct
import time
from datetime import datetime

SOF = b"\xAA\x55"
PORT = "COM5"
BAUDRATE = 115200
READ_TIMEOUT_S = 0.01
ACK_DELAY_S = 0.02


def now_ms() -> str:
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


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

    # 关键修改：timeout 设为 0，完全非阻塞轮询
    ser = serial.Serial(PORT, BAUDRATE, timeout=0, write_timeout=0.2)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    buffer = bytearray()

    print(f"[{now_ms()}] Listening on {PORT} at {BAUDRATE} baud")

    while True:
        # ── 关键修改：只读缓冲区里已有的全部数据，不阻塞 ──
        chunk = ser.read(ser.in_waiting)

        if chunk:
            print(
                f"[{now_ms()}] [DEBUG] ser.read() returned {len(chunk)} bytes, "
                f"head=0x{chunk[0]:02X}",
                flush=True,
            )

        if not chunk:
            # 没有数据时让出 CPU 1ms，避免空转占满一个核心
            time.sleep(0.001)
            continue

        buffer += chunk

        while True:
            idx = buffer.find(SOF)
            if idx < 0:
                buffer.clear()
                break

            if idx > 0:
                print(
                    f"[{now_ms()}] [DEBUG] Discarded {idx} bytes before SOF: "
                    f"{buffer[:idx].hex(' ')}",
                    flush=True,
                )
                del buffer[:idx]

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
                print(f"[{now_ms()}] CRC error: {frame.hex(' ')}", flush=True)
                continue

            if frame_type == 0x01:
                payload = frame[8:-2]

                if payload_len >= 10:
                    status, wl_mm, wv_mms = struct.unpack_from("<Hii", payload, 0)
                    print(
                        f"[{now_ms()}] RX DEV={dev_id}, SEQ={seq}, ST=0x{status:04X}, "
                        f"WL={wl_mm / 1000:.3f} m, WV={wv_mms / 1000:.3f} m/s",
                        flush=True,
                    )
                else:
                    print(
                        f"[{now_ms()}] RX DEV={dev_id}, SEQ={seq}, short payload len={payload_len}",
                        flush=True,
                    )

                ack = build_ack(dev_id, seq)
                if ACK_DELAY_S > 0:
                    time.sleep(ACK_DELAY_S)
                ser.write(ack)
                ser.flush()
                print(f"[{now_ms()}] TX ACK: {ack.hex(' ')}", flush=True)

            elif frame_type == 0x81:
                print(f"[{now_ms()}] RX ACK: {frame.hex(' ')}", flush=True)
            else:
                print(
                    f"[{now_ms()}] RX unknown type=0x{frame_type:02X}: {frame.hex(' ')}",
                    flush=True,
                )


if __name__ == "__main__":
    main()