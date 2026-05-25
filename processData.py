import argparse
import struct
import time
from datetime import datetime

SOF = b"\xAA\x55"
DEFAULT_PORT = "COM5"
DEFAULT_BAUDRATE = 115200
READ_TIMEOUT_S = 0.01

FRAME_TYPE_DATA = 0x01
FRAME_TYPE_ACK = 0x81

STATUS_FLAGS = (
    (0, "WL_VALID"),
    (1, "WV_VALID"),
    (2, "ATT_VALID"),
    (3, "PLL_OK"),
    (4, "ADC_OK"),
    (5, "SIGNAL_WEAK"),
    (6, "RANGE_OVER"),
    (7, "VEL_ABNORMAL"),
)


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


def status_to_text(status: int) -> str:
    names = [name for bit, name in STATUS_FLAGS if status & (1 << bit)]
    return "|".join(names) if names else "0"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Receive and parse RadarProtocol_SendData frames without sending ACK."
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"serial port, default: {DEFAULT_PORT}")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUDRATE, help=f"baudrate, default: {DEFAULT_BAUDRATE}")
    parser.add_argument("--raw", action="store_true", help="print raw frame bytes")
    return parser.parse_args()


def handle_frame(frame: bytes, show_raw: bool) -> None:
    frame_type = frame[2]
    dev_id = struct.unpack_from("<H", frame, 3)[0]
    seq = struct.unpack_from("<H", frame, 5)[0]
    payload_len = frame[7]

    if show_raw:
        print(f"[{now_ms()}] RAW: {frame.hex(' ')}", flush=True)

    if frame_type != FRAME_TYPE_DATA:
        name = "ACK" if frame_type == FRAME_TYPE_ACK else f"0x{frame_type:02X}"
        print(f"[{now_ms()}] RX non-data type={name}, DEV={dev_id}, SEQ={seq}", flush=True)
        return

    payload = frame[8:-2]
    if payload_len != 10:
        print(
            f"[{now_ms()}] RX DEV={dev_id}, SEQ={seq}, unexpected payload len={payload_len}",
            flush=True,
        )
        return

    status, water_level_mm, water_velocity_mms = struct.unpack_from("<Hii", payload, 0)
    print(
        f"[{now_ms()}] RX DEV={dev_id}, SEQ={seq}, "
        f"ST=0x{status:04X} [{status_to_text(status)}], "
        f"WL={water_level_mm / 1000:.3f} m, "
        f"WV={water_velocity_mms / 1000:.3f} m/s",
        flush=True,
    )


def main() -> None:
    import serial

    args = parse_args()
    ser = serial.Serial(args.port, args.baud, timeout=READ_TIMEOUT_S)
    ser.reset_input_buffer()

    buffer = bytearray()
    print(f"[{now_ms()}] Listening on {args.port} at {args.baud} baud")

    while True:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            time.sleep(0.001)
            continue

        buffer += chunk

        while True:
            idx = buffer.find(SOF)
            if idx < 0:
                if buffer.endswith(SOF[:1]):
                    if len(buffer) > 1:
                        dropped = buffer[:-1]
                        print(
                            f"[{now_ms()}] Drop {len(dropped)} bytes without SOF: {dropped.hex(' ')}",
                            flush=True,
                        )
                        del buffer[:-1]
                    break

                if buffer:
                    print(f"[{now_ms()}] Drop {len(buffer)} bytes without SOF: {buffer.hex(' ')}", flush=True)
                buffer.clear()
                break

            if idx > 0:
                print(f"[{now_ms()}] Drop {idx} bytes before SOF: {buffer[:idx].hex(' ')}", flush=True)
                del buffer[:idx]

            if len(buffer) < 10:
                break

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
                print(
                    f"[{now_ms()}] CRC error: recv=0x{recv_crc:04X}, "
                    f"calc=0x{calc_crc:04X}, frame={frame.hex(' ')}",
                    flush=True,
                )
                continue

            handle_frame(frame, args.raw)


if __name__ == "__main__":
    main()
