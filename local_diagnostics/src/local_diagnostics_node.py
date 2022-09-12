#!/usr/bin/env python3
from threading import local
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Bool
from local_diagnostics.srv import GetNodeLevel, GetNodeLevelResponse

"""
The purpose of this node is to provide aggregate diagnostic information from 
various nodes on a single agent. As such, this is where each node can report 
their overall health, whether they are initialized or not, and any other 
diagnostic information.

All nodes publish to the local_diagnostics/in topic, which this node subcribes
to. This node then publishes a summary of overall system health.

This node can be used as a mechanism to get nodes to "wait" for other nodes to 
be initialized before starting a procedure.
"""

class LocalDiagnosticsNode(object):
    def __init__(self):
        rospy.init_node("local_diagnostics")

        # Subscribers
        self.sub = rospy.Subscriber(
            "local_diagnostics/in", DiagnosticArray, self.cb_diagnostics, queue_size=10
        )

        # Publishers
        self.killswitch_pub = rospy.Publisher(
            "local_diagnostics/killswitch", Bool, queue_size=1
        )

        self.summary_pub = rospy.Publisher(
            "local_diagnostics/summary", DiagnosticArray, queue_size=1
        )

        self.diagnostics_pub = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=1
        )

        # Services
        self.node_level_srv = rospy.Service(
            "local_diagnostics/get_node_level", GetNodeLevel, self.get_node_level
        )

        self.node_data = {}

    def cb_diagnostics(self, diagnostic_array):
        """
        Upon recieving a diagnostics message from any node, update the entry
        in the current self.node_summary dictionary.
        """
        for status in diagnostic_array.status:
            self.node_data[status.name] = {}
            self.node_data[status.name]["status_msg"] = status
            self.node_data[status.name]["stamp"] = rospy.get_rostime()
        #     }

    def get_node_level(self, request):
        """
        Service that returns the diagnostic status level of a particular node.
        """
        if request.node_name in self.node_data:
            status = self.node_data[request.node_name]["status_msg"]
            level = int(status.level)
            is_valid = True
        else:
            level = 4
            is_valid = False
        response = GetNodeLevelResponse()
        response.level = level
        response.is_valid = is_valid
        return response

    def refresh_ages(self):
        """
        Loops through all the node statuses and updates the age of last update.
        """
        stale_threshold = rospy.Duration(secs = 3) 
        for node_name in self.node_data.keys():
            
            temp = self.node_data[node_name]
            age = rospy.get_rostime() - temp["stamp"]
            temp["age"] = age
            if age > stale_threshold:
                temp["status_msg"].level = DiagnosticStatus.STALE

            last_updated_kv = KeyValue("Last Updated (s)", str(temp["stamp"].to_sec()))
            age_kv = KeyValue("Age (s)", str(age.to_sec())) 
            temp["status_msg"].values = [last_updated_kv, age_kv]

            self.node_data[node_name] = temp

    def start(self):
        """
        Periodically publishes the node summary.
        """
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if len(self.node_data) > 0:

                self.refresh_ages()

                # Concat everything into array.
                msg = DiagnosticArray()
                msg.header.stamp = rospy.Time.now()
                for value in self.node_data.values():
                    msg.status.append(value["status_msg"])

                # Publish
                self.summary_pub.publish(msg)
                self.diagnostics_pub.publish(msg)

                try:  # prevent garbage in console output when thread is killed
                    rate.sleep()
                except rospy.ROSInterruptException:
                    pass

    def cb_shutdown(self):
        self.summary_pub.unregister()


if __name__ == "__main__":
    local_diagnostics = LocalDiagnosticsNode()
    rospy.on_shutdown(local_diagnostics.cb_shutdown)
    local_diagnostics.start()
    rospy.spin()  # Should only get here on shutdown
