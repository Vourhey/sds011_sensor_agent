import time
import uuid
import json
import requests
import sds011

BROADCASTER_VERSION = "v0.1.0"


class SdsMeasurement:
    def __init__(self, pm25, pm10):
        self.pm25 = pm25
        self.pm10 = pm10

    def __str__(self):
        return f"{{PM2.5: {self.pm25}, PM10: {self.pm10}}}"

class StationData:
    def __init__(self, ver: str, mac: str, uptime: float, meas: SdsMeasurement):
        self.version = ver
        self.mac = mac
        self.uptime = uptime
        self.meas = meas

    def to_json(self) -> dict:
        ret = {
            "software_version": self.version,
            "sensordatavalues": [
              { "value_type": "P1", "value": self.meas.pm25 },
              { "value_type": "P2", "value": self.meas.pm10 }
            ]
        }

        print(ret)
        return ret

    def __str__(self):
        return f"{{MAC: {self.mac}, Uptime: {self.uptime}, M: {self.meas}}}"

class LufdatenFeeder:
    def __init__(self):
        self.apiServerUrl = "https://api.luftdaten.info/v1/push-sensor-data/"

    def feed(self, data: StationData):
        sensor_id = f"raspi-{data.mac}"
        sensor_data = data.to_json()

        self._post_data(sensor_id, 1, sensor_data)

    def _post_data(self, sensor_id: str, pin: int, data: dict):
        headers = {
                "Content-Type": "application/json",
                "X-Pin": str(pin),
                "X-Sensor": sensor_id
                }

        print("Sending data...")
        r = requests.post(self.apiServerUrl, json=data, headers=headers)
        print(f"Response {r.status_code}")

class RpiStation:
    def __init__(self, sds_sensor_port: str = "/dev/ttyUSB0"):
        self.version = f"airalab-rpi-broadcaster-{BROADCASTER_VERSION}"
        self.start_time = time.time()
        self.mac_address = ''.join(['{:02x}'.format((uuid.getnode() >> ele) & 0xff) for ele in range(0,8*6,8)][::-1])

        self.sensor = sds011.SDS011(sds_sensor_port)

    def __str__(self):
        return f"{{Version: {self.version}, Start: {self.start_time}, MAC: {self.mac_address}}}"

    def get_data(self) -> StationData:
        meas = self.sensor.query()

        return StationData(
                self.version,
                self.mac_address,
                time.time() - self.start_time,
                SdsMeasurement(meas[0], meas[1])
                )

def main():
    rpi_station = RpiStation()
    print(f"Station Info: {rpi_station}")
    feeder = LufdatenFeeder()

    while True:
        station_data = rpi_station.get_data()
        print(f"Station Data: {station_data}")
        feeder.feed(station_data)
        time.sleep(30)


if __name__ == "__main__":
    main()
