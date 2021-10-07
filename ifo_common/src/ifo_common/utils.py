#!/usr/bin/env python2
from tf.transformations import quaternion_matrix
from geometry_msgs.msg import Quaternion, Pose

class DictObj:
    """
    Converts a potentially nested dictionary into an object with properties.
    For example:

        x = {'key1':'value1', 'key2':'value2', 'key3':{'nested_key1':nested_val1}}
        x_obj = DictObj(x)

        # Now we can access with dot syntax
        x_obj.key1
        x_obj.key2
        x_obj.key3.nested_key1

    """
    def __init__(self, in_dict):
    	assert isinstance(in_dict, dict)
        for key, val in in_dict.items():
            if isinstance(val, (list, tuple)):
               setattr(self, key, 
                      [DictObj(x) if isinstance(x, dict) else x for x in val])
            else:
               setattr(self, key, 
                       DictObj(val) if isinstance(val, dict) else val)

def quat_msg_to_matrix(quaternion):
    """
    Takes a quaternion message from geometry_msgs/Quaternion and converts it
    to a 3x3 numpy array rotation matrix/DCM.
    """
    if isinstance(quaternion, Quaternion):
        T = quaternion_matrix([
            quaternion.w,
            quaternion.x,
            quaternion.y,
            quaternion.z
        ])
        return T[0:3,0:3]
    else:
        raise TypeError()

def pose_msg_to_matrix(pose):
    """
    Takes a pose message from geometry_msgs/Pose and converts it
    to a 4x4 numpy array transformation matrix.
    """
    # TODO: handle PoseStamped
    if isinstance(pose, Pose):
        quaternion = pose.orientation
        T = quaternion_matrix(
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        )
        T[0,3] = pose.position.x
        T[1,3] = pose.position.y
        T[2,3] = pose.position.z
        return T
    else:
        raise TypeError()


