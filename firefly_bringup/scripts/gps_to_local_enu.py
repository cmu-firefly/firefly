#!/usr/bin/env python2

import rospy
import tf
from geometry_msgs.msg import QuaternionStamped
from sensor_msgs.msg import NavSatFix
from firefly_telemetry.srv import SetLocalPosRef, SetLocalPosRefResponse, SetLocalPosRefRequest
import pymap3d as pm


class GPS2LocalENU:
    def __init__(self):
        rospy.Subscriber('dji_sdk/attitude', QuaternionStamped, self.attitude_callback)
        rospy.Subscriber('dji_sdk/gps_position', NavSatFix, self.gps_pos_callback)
        rospy.Service('set_local_pos_ref', SetLocalPosRef, self.set_local_pos_ref_handler)

        self.br = tf.TransformBroadcaster()

        self.attitude = None
        self.attitude_stamp = None

        self.lat = None
        self.lon = None
        self.h = None
        self.gps_stamp = None

        self.lat0 = None
        self.lon0 = None
        self.h0 = None

        self.x = None
        self.y = None
        self.z = None

        if rospy.has_param('~lat0rtk') and rospy.has_param('~lon0rtk'):
            self.lat0rtk = rospy.get_param('~lat0rtk')
            self.lon0rtk = rospy.get_param('~lon0rtk')
        else:
            self.lat0rtk = None
            self.lon0rtk = None

    def attitude_callback(self, data):
        self.attitude = data.quaternion
        self.attitude_stamp = data.header.stamp
        self.publish_tf()

    def gps_pos_callback(self, data):
        self.lat = data.latitude
        self.lon = data.longitude
        self.h = data.altitude
        self.gps_stamp = data.header.stamp
        self.publish_tf()

    def set_local_pos_ref_handler(self, req):
        self.lat0 = self.lat
        self.lon0 = self.lon
        self.h0 = self.h

        response = SetLocalPosRefResponse()
        if self.lat0rtk is not None and self.lon0rtk is not None:
            response.latitude = self.lat0rtk
            response.longitude = self.lon0rtk
            response.altitude = self.h0
        else:
            response.latitude = self.lat0
            response.longitude = self.lon0
            response.altitude = self.h0  

        self.publish_tf() 
        return response

    def publish_tf(self):
        if self.attitude is None \
                or self.lat is None \
                or self.lon is None \
                or self.h is None \
                or self.lat0 is None \
                or self.lon0 is None \
                or self.h0 is None:
            return

        # TODO: Check that both GPS and attitude time stamps are recent
        # TODO: Change TF time stamp to be latest between GPS and attitude time stamps
        self.x, self.y, self.z = pm.enu.geodetic2enu(self.lat, self.lon, self.h, self.lat0, self.lon0, self.h0)
        self.br.sendTransform((self.x, self.y, self.z),
                              (self.attitude.x, self.attitude.y, self.attitude.z, self.attitude.w),
                              rospy.Time.now(),
                              "base_link",
                              "world")


if __name__ == '__main__':
    rospy.init_node('gps_to_local_enu', anonymous=True)
    enu_pub = GPS2LocalENU()

    rospy.spin()
