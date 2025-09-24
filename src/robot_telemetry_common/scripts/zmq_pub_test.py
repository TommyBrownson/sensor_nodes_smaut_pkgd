import time, zmq, msgpack, random

# telemetry PUB for testing
ctx = zmq.Context()
sock = ctx.socket(zmq.PUB)
sock.bind("tcp://127.0.0.1:35000")

# radar PUB for testing
sock2 = ctx.socket(zmq.PUB)
sock2.bind("tcp://127.0.0.1:35001")

# Give subscribers a moment to connect
time.sleep(0.2)

i = 0
while True:
    # Minimal “telemetry” dict you can parse later
    payload = {
        "robot_status": {"rtk_fix": (i % 2 == 0)},
        "navigation_gnss": {
            "lat": 42.0 + i*1e-6,
            "lon": -71.0,
            "alt": 0.0,
            "accuracy": 0.0,
            "num_sats": 0,
            "pdop": 0.0,
            "hdop": 0.0,
            "vdop": 0.0,
            "geoid_height": 0.0,
            "x_local": 0.0,
            "y_local": 0.0,
            "heading": 0.0,
            "rel_pos_n": 0.0,
            "rel_pos_e": 0.0,
            "rel_pos_d": 0.0,
            "acc_n": 0.0,
            "acc_e": 0.0,
            "acc_d": 0.0
             },
    }
    sock.send(msgpack.packb(payload, use_bin_type=True))

    radar_points = []
    for sensor_id in range(8):
        radar_points.append({
            "timestamp": time.time(),
            "id_board": 1,
            "id_sensor": sensor_id,
            "id_deployment": 0,
            "x": random.uniform(-5, 5),
            "y": random.uniform(-5, 5),
            "z": 0.0,
            "velocity": random.uniform(-1, 1),
            "cov_mat_xx": 0.1,
            "cov_mat_yy": 0.1,
            "cov_mat_zz": 0.1,
            "cov_mat_vv": 0.1,
            "snr": random.uniform(10, 30),            
        })
    sock2.send(msgpack.packb(radar_points, use_bin_type=True))



    i += 1
    time.sleep(0.2)  # 5 Hz