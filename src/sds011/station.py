import time
from datetime import timedelta
import netifaces
from sds011.sds011 import SDS011


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
              { "value_type": "P1", "value": self.meas.pm10 },
              { "value_type": "P2", "value": self.meas.pm25 }
            ]
        }

        print(ret)
        return ret

    def __str__(self):
        uptime = str(timedelta(seconds = self.uptime))
        return f"{{MAC: {self.mac}, Uptime: {uptime}, M: {self.meas}}}"

class RpiStation:
    def __init__(self, sds_sensor_port: str = "/dev/ttyUSB0", work_time: int = 5):
        self.version = f"airalab-rpi-broadcaster-{BROADCASTER_VERSION}"
        self.start_time = time.time()
        self.mac_address = self._get_mac()

        self.sensor = SDS011(sds_sensor_port)
        self.sensor.set_work_period(work_time=work_time)

    def _get_mac(self) -> str:
        for interface in netifaces.interfaces():
            if interface != 'lo':
                '''
                if 2 in netifaces.ifaddresses(interface):
                    _i = netifaces.ifaddresses(interface)
                    _i = _i[2][0]['addr']
                    break
                '''
                if 17 in netifaces.ifaddresses(interface):
                    _i = netifaces.ifaddresses(interface)
                    _i = _i[17][0]['addr']
                    break
                '''
                if 18 in netifaces.ifaddresses(interface):
                    _i = netifaces.ifaddresses(interface)
                    _i = _i[18][0]['addr']
                    break
                '''

        mac = _i.replace(':', '')
        return mac

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

