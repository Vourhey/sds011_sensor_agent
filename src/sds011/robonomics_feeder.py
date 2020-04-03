import json

import rospy
from std_msgs.msg import String

from robonomics_msgs.msg import Result
from ethereum_common.msg import Address
from ipfs_common.msg import Multihash
from ipfs_common.ipfs_rosbag import IpfsRosBag

from sds011.station import StationData


class RobonomicsFeeder:
    def __init__(self, publisher: rospy.Publisher, geo: str = ""):
        self.publisher = publisher
        self.geo = geo

    def feed(self, data: StationData):
        rospy.loginfo("RobonomicsFeeder:")

        res = Result()
        res.liability = Address("0x0000000000000000000000000000000000000000")
        res.result = self._get_multihash(data)
        res.success = True

        rospy.loginfo(res)

        self.publisher.publish(res)

    def _get_multihash(self, data: StationData) -> Multihash:
        d = {
                "PM2.5": data.meas.pm25,
                "PM10": data.meas.pm10
                }
        topics = {
                "/data": [ String( json.dumps(d) ) ],
                "/geo": [ String( self.geo ) ]
                }
        bag = IpfsRosBag(messages=topics)
        return bag.multihash




