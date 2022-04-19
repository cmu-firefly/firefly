#!/usr/bin/env python3

import rospy
import pymap3d as pm
import yaml
import rospkg

class GPS2LocalENU:
    def __init__(self):
        rospack = rospkg.RosPack()
        with open(rospack.get_path("firefly_mapping") + "/config/gps_hotspots.yaml", 'r') as f:
            self.d = yaml.load(f, Loader=yaml.FullLoader)
        
        self.lat0, self.lon0, self.h0 = rospy.get_param("local_pos_ref")

        self.gps2local()

    def gps2local(self):
        all_keys = list(self.d.keys())
        xys = []
        for i in range(0, len(all_keys), 3):
            x, y, _ = pm.enu.geodetic2enu(self.d[all_keys[i]], self.d[all_keys[i+1]], self.d[all_keys[i+2]], self.lat0, self.lon0, self.h0)
            xys.extend([x.item(), y.item()])
        
        rospy.set_param("gt_locs", xys)


if __name__ == "__main__":
    rospy.init_node("gps_to_local", anonymous=True)
    while not rospy.has_param("local_pos_ref"): continue
    g2l = GPS2LocalENU()
    #Check if param is still on server after shutting down node #UPDATE: It is still on server
    rospy.signal_shutdown("Set ground truth fire locations")