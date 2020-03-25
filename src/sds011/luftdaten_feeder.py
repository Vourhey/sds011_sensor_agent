import requests
import rospy
from sds011.station import StationData


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

        rospy.loginfo("Sending data...")
        r = requests.post(self.apiServerUrl, json=data, headers=headers)
        rospy.loginfo(f"Response {r.status_code}")


'''
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
'''
