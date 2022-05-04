#!/usr/bin/env python2
from mavros_controller import ControllerNode 
import rospy 
import numpy as np
from controller.msg import WaypointList

class WaypointController(ControllerNode):
    def __init__(self):
        super(WaypointController, self).__init__()
        self.waypoints_sub = rospy.Subscriber(
            "controller/waypoints_in", WaypointList, self.cb_waypoints_in
        )

    def cb_waypoints_in(self, waypoints_msg):
        rospy.loginfo(waypoints_msg)
        self.reach_waypoint_list(waypoints_msg.data)

    def reach_waypoint_list(self, waypoint_list):
        """
        Reaches a WaypointList one by one.
        """

        rospy.loginfo("WP List:" + str(waypoint_list))

        # Sort waypoints
        waypoint_list.sort(key=lambda x: x.time)
        rospy.loginfo("WP List Sorted:" + str(waypoint_list))

        self.wait_for_nodes("mocap_forwarder")
        self.report_diagnostics(level=0, message="Normal. Reaching waypoints.")
        self.takeoff()
        self.hold_position(pz=1)

        # We may already be behind schedule. determine where we are in the
        # waypoint list and skip to the waypoint that we should be at.
        current_wp = None
        current_wp_idx = -1
        for idx, wp in enumerate(waypoint_list):
            t_now = rospy.get_time()
            if t_now > wp.time:
                current_wp = wp  # Then we should already be at this waypoint.
                current_wp_idx = idx
                self.report_diagnostics(
                    level=0,
                    message="Beyond WP #"
                    + str(current_wp_idx)
                    + " time: "
                    + str(wp.time)
                    + " current time: "
                    + str(t_now),
                )

        # If there is a waypoint that we should already be at
        if current_wp is not None:
            wp = current_wp

            self.report_diagnostics(
                level=0,
                message="Reaching WP #"
                + str(current_wp_idx)
                + " time: "
                + str(wp.time)
                + " x: "
                + str(wp.x)
                + " y: "
                + str(wp.y)
                + " z: "
                + str(wp.z)
                + "yaw: "
                + str(wp.yaw),
            )

            # Go to that waypoint
            self.reach_waypoint(wp, timeout=10)

        current_wp_idx = current_wp_idx + 1

        # Now go to each remaining waypoint one by one
        remaining_wp_list = waypoint_list[current_wp_idx:]
        for wp in remaining_wp_list:
            self.report_diagnostics(
                level=0,
                message="Holding until WP #"
                + str(current_wp_idx)
                + " time: "
                + str(wp.time)
                + " x: "
                + str(wp.x)
                + " y: "
                + str(wp.y)
                + " z: "
                + str(wp.z)
                + " yaw: "
                + str(wp.yaw),
            )

            # Stall until time of next waypoint
            time_until_wp = float(wp.time - rospy.get_time())
            while time_until_wp > 0:
                # This is the main stalling loop, so should implement checks here.
                rospy.sleep(time_until_wp)
                time_until_wp = float(wp.time - rospy.get_time())

            self.report_diagnostics(
                level=0,
                message="Reaching WP #"
                + str(current_wp_idx)
                + " time: "
                + str(wp.time)
                + " x: "
                + str(wp.x)
                + " y: "
                + str(wp.y)
                + " z: "
                + str(wp.z)
                + " yaw: "
                + str(wp.yaw),
            )

            self.reach_waypoint(wp, timeout=10)

            current_wp_idx = current_wp_idx + 1

        # Ideally need feedback logic on reaching waypoint
        rospy.sleep(3)
        self.land()
        self.report_diagnostics(level=0, message="Normal. Idle.")

    def reach_waypoint(self, wp, timeout):
        """
        Reaches a single Waypoint and confirms with state estimate feedback.

        Will trigger a failsafe if the distance to waypoint is greater than 1.5
        meters from the initial distance.
        """
        reach_threshold = 0.5  # distance in meters
        wp_pos = np.array([wp.x, wp.y, wp.z])
        pos = self.pose.pose.position
        my_pos = np.array([pos.x, pos.y, pos.z])
        starting_d = np.linalg.norm(wp_pos - my_pos)

        self.set_position_command(wp.x, wp.y, wp.z, wp.yaw)

        d = starting_d
        loop_freq = 10
        rate = rospy.Rate(10)

        for i in range(timeout * loop_freq):
            if rospy.is_shutdown():
                break
            if d < reach_threshold:
                rospy.loginfo("Successfully reached waypoint.")
                break

            pos = self.pose.pose.position
            my_pos = np.array([pos.x, pos.y, pos.z])
            d = np.linalg.norm(wp_pos - my_pos)
            if d > starting_d + 1.5:
                self.report_diagnostics(level=2, message="ERROR: Failing to reach WP")
                self.kill_mission()

if __name__ == "__main__":
    node = WaypointController()
    rospy.spin()