#!/usr/bin/env python2
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Bool
from local_diagnostics.srv import GetNodeLevel, GetNodeLevelResponse

"""
The purpose of this node is to provide aggregate diagnostic information from 
various nodes on a single agent. As such, this is where each node can report 
their overall health, whether they are initialized or not, and any other 
diagnostic information.

All nodes publish to the local_diagnostics/raw topic, which this node subcribes
to. This node then publishes a summary of overall system health.

This node can be used as a mechanism to get nodes to "wait" for other nodes to 
be initialized before starting a procedure.
"""
def print_dictionary(summary):
    """
    Essentially just prints a once-nested dictionary with identicle keys as a 
    table.
    """
    if summary: # Check if empty
        keys1 = summary.keys()
        nested_dict = summary[keys1[0]]
        keys2 = nested_dict.keys()

        column_widths = {'first':len(max(keys1, key=len))}

        # Extract the maximum column width
        for key2 in keys2:
            max_len = len(str(key2))
            for key in keys1:
                if len(str(summary[key][key2])) > max_len:
                    max_len = len(str(summary[key][key2]))
            column_widths[key2] = max_len + 2
        
        # Construct header. Give space for top-level key.
        header = ' '*column_widths['first'] 
        for key2 in keys2:
            header += '{:^{width}}'.format(key2, width=column_widths[key2])

        seperator = '-'*len(header)
        print(seperator)
        print(header)
        print(seperator)
        
        # Populate the table
        for key in keys1:
            line_string = '{:<{width}}'.format(key, width=column_widths['first'] )
            nested_dict = summary[key]
            for key2 in nested_dict.keys():
                line_string += '{:^{width}}'.format(nested_dict[key2], width=column_widths[key2])
            print(line_string)
        print(seperator)

class LocalDiagnosticsNode(object):

    def __init__(self):
        rospy.init_node('local_diagnostics')
        super(LocalDiagnosticsNode, self).__init__()

        # Subscribers
        self.sub = rospy.Subscriber('local_diagnostics/in',DiagnosticArray,
                                    self.cb_diagnostics, queue_size=10)

        # Publishers
        self.killswitch_pub = rospy.Publisher(
            'local_diagnostics/killswitch', Bool, queue_size=1
        )

        # TODO: Publish periodic summary. need to define a message.
        # self.summary_pub = rospy.Publisher(
        #     'local_diagnostics/summary', TODO, queue_size=1
        # )

        # Services
        self.node_level_srv = rospy.Service(
            'local_diagnostics/get_node_level', GetNodeLevel, self.get_node_level
        )

        self.node_summary  = {}

    def cb_diagnostics(self, diag_msg):      
        """
        Upon recieving a diagnostics message from any node, update the entry
        in the current self.node_summary dictionary.        
        """  
        for diag_status in diag_msg.status:
            
            # KILL MISSION. Send message to all nodes.
            # TODO. Needs to be implemented on node side.
            if diag_status.level == 4:
                killswitch_msg = Bool()
                killswitch_msg.data = True
                self.killswitch_pub(killswitch_msg)

            self.node_summary[diag_status.name] = {
                'last_updated':diag_msg.header.stamp.to_sec(),
                'name':diag_status.name,
                'level':diag_status.level,
                'message':diag_status.message,
                'id':diag_status.hardware_id,
                'values':diag_status.values,
                'age': round(rospy.get_time() - diag_msg.header.stamp.to_sec(),1)
             }
        #print_dictionary(self.node_summary)

    def get_node_level(self, request):
        """
        Service that returns the diagnostic status level of a particular node.
        """
        if request.node_name in self.node_summary:
            node_status = self.node_summary[request.node_name]
            level = int(node_status['level'])
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
        for node_name in self.node_summary:
            temp = self.node_summary[node_name]
            age = rospy.get_time() - temp['last_updated']
            temp['age'] = age
            self.node_summary[node_name] = temp

    def print_summary(self):
        """
        Prints the node summary to screen in a table.
        """
        self.refresh_ages()
        print_dictionary(self.node_summary)
    
    def start(self):
        rate = rospy.Rate(0.5)
        while True:
            self.print_summary()
            #self.summary_pub.publish(self.node_summary) # TODO
            rate.sleep()

if __name__ == "__main__":
    local_diagnostics = LocalDiagnosticsNode()
    local_diagnostics.start()

    # Should never get here.
    rospy.spin()