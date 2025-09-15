from telemetry_io import TelemetrySubscriber
import time

sub = TelemetrySubscriber(endpoint="tcp://127.0.0.1:35000", topic="")  # "" == all
print("Subscriber started. Ctrl+C to stop.")
try:
    while True:
        data = sub.receive_zmq()
        if data is not None:
            print("RECV:", data)
        else:
            # Nothing this tick; sleep a touch so we don't spin
            time.sleep(0.05)
except KeyboardInterrupt:
    pass
finally:
    sub.close()
    print("Closed.")