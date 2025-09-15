import time, zmq, msgpack

ctx = zmq.Context()
sock = ctx.socket(zmq.PUB)
sock.bind("tcp://127.0.0.1:35000")

# Give subscribers a moment to connect
time.sleep(0.2)

i = 0
while True:
    # Minimal “telemetry” dict you can parse later
    payload = {
        "info": {"rtk_fix": (i % 2 == 0)},
        "navigation_gnss": {"latitude": 42.0 + i*1e-6, "longitude": -71.0},
    }
    sock.send(msgpack.packb(payload, use_bin_type=True))
    i += 1
    time.sleep(0.2)  # 5 Hz