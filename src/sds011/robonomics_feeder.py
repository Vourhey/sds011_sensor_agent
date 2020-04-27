import json
import subprocess
import time

import rospy
from std_msgs.msg import String

from robonomics_msgs.msg import Result
from ethereum_common.msg import Address
from ipfs_common.msg import Multihash
from ipfs_common.ipfs_rosbag import IpfsRosBag

from sds011.station import StationData


class RobonomicsFeeder:
    def __init__(self,
                 publisher: rospy.Publisher,
                 geo: str = "",
                 robonomics: str = "",
                 suri: str = "",
                 dump: int = 60):
        self.publisher = publisher
        self.geo = geo

        self.robonomics = robonomics
        self.suri = suri
        self.dump = int(dump) * 60   # seconds
        self.last_time = time.time()

    def feed(self, data: StationData):
        rospy.loginfo("RobonomicsFeeder:")

        res = Result()
        res.liability = Address("0x0000000000000000000000000000000000000000")
        res.result = self._get_multihash(data)
        res.success = True

        rospy.loginfo(res)

        if self.robonomics != "":
            self._to_datalog(res.result.multihash)

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

    def _to_datalog(self, ipfs_hash: str):
        if (time.time() - self.last_time) > self.dump:
            prog_path = [self.robonomics, "io", "write", "datalog", "-s", self.suri, "--remote", "wss://substrate.ipci.io"]
            o = subprocess.run(prog_path, stdout=subprocess.PIPE, input=ipfs_hash.encode(), stderr=subprocess.PIPE)
            rospy.loginfo(o)

            self.last_time = time.time()




