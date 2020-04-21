#!/usr/bin/evn python3

from sds011.sds011 import SDS011
import time
import sys

if __name__ == "__main__":
    port = '/dev/ttyUSB0'
    if len(sys.argv) > 1:   # the second arg is treated as a port
        port = sys.argv[1]

    sensor = SDS011(port)

    while True:
        q = sensor.query()
        t = time.localtime()
        current_t = time.strftime("%H:%M:%S", t)
        print("{}: {}".format(current_t, q))
        # sensor.sleep(sleep=True)
        time.sleep(25)


