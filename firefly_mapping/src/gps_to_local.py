#!/usr/bin/env python3

import rospy
import pymap3d as pm
import yaml
import rospkg
from nav_msgs.msg import OccupancyGrid

if __name__ == "__main__":
    rospy.init_node("gps_to_local", anonymous=True)
    pub = rospy.Publisher("gt_map", OccupancyGrid, queue_size=100)
    rospy.sleep(1)

    rospack = rospkg.RosPack()
    with open(rospack.get_path("firefly_mapping") + "/config/gps_hotspots.yaml", 'r') as f:
        d = yaml.load(f, Loader=yaml.FullLoader)

    while not rospy.has_param("local_pos_ref"):
        continue
    lat0, lon0, h0 = rospy.get_param("local_pos_ref")

    all_keys = list(d.keys())
    xys = []
    for i in range(0, len(all_keys), 3):
        x, y, _ = pm.enu.geodetic2enu(d[all_keys[i]], d[all_keys[i + 1]], d[all_keys[i + 2]], lat0, lon0, h0)
        xys.extend([x.item(), y.item()])

    rospy.set_param("gt_locs", xys)

    resolution = 0.5
    minX = -100
    maxX = 100
    minY = -100
    maxY = 100

    output = OccupancyGrid()
    output.header.frame_id = "world"
    output.info.resolution = 0.5
    output.info.width = 400
    output.info.height = 400
    output.info.origin.position.x = -100
    output.info.origin.position.y = -100
    output.data = [0] * (400 * 400)

    for x, y in xys:

        gtRow = int((y-minY)/resolution)
        gtCol = int((x-minX)/resolution)

        gtBin = gtCol + gtRow * 400

        output.data[gtBin] = 100

    pub.publish(output)
    rospy.sleep(1)

    rospy.signal_shutdown("Set ground truth fire locations")