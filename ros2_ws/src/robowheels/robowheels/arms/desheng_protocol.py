"""Desheng v4.01 framing and read-only transactions; no ROS/serial imports."""

def packet(sid, command, params=b''):
    payload = bytes([sid, len(params) + 2, command]) + params
    return b'\xff\xff' + payload + bytes([(~sum(payload)) & 255])


def query(port, sid, command, params=b''):
    port.reset_input_buffer()
    port.write(packet(sid, command, params))
    port.flush()
    data = port.read(64)
    for start in range(len(data) - 5):
        if data[start:start + 2] != b'\xff\xf5':
            continue
        length = data[start + 3] + 4
        frame = data[start:start + length]
        if len(frame) == length and frame[2] == sid and sum(frame[2:]) & 255 == 255:
            return frame
    if data:
        print(f'ID {sid}: unrecognized RX {data.hex()}', flush=True)
    return None


