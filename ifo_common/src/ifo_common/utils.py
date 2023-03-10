from geometry_msgs.msg import (
    Quaternion,
    Pose,
    PoseStamped,
    PoseWithCovarianceStamped,
    PoseWithCovariance,
)
import numpy as np
import rospy


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
        if not isinstance(in_dict, dict):
            raise RuntimeError("Argument must be a dictionary.")

        for key, val in in_dict.items():
            if isinstance(val, (list, tuple)):
                setattr(
                    self, key, [DictObj(x) if isinstance(x, dict) else x for x in val]
                )
            else:
                setattr(self, key, DictObj(val) if isinstance(val, dict) else val)


def cross_matrix(v):
    v = v.flatten()
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def quat_msg_to_matrix(quaternion):
    """
    Takes a quaternion message from geometry_msgs/Quaternion and converts it
    to a 3x3 numpy array rotation matrix/DCM.
    """
    if isinstance(quaternion, Quaternion):
        eps = np.array([quaternion.x, quaternion.y, quaternion.z]).reshape((-1, 1))

        eta = quaternion.w
        return (
            (1 - 2 * np.matmul(np.transpose(eps), eps)) * np.eye(3)
            + 2 * np.matmul(eps, np.transpose(eps))
            - 2 * eta * cross_matrix(eps)
        )
    else:
        raise TypeError()


def pose_msg_to_matrix(pose):
    """
    Takes a pose message from geometry_msgs/Pose and converts it
    to a 4x4 numpy array transformation matrix.
    """
    if isinstance(pose, PoseStamped) or isinstance(pose, PoseWithCovariance):
        pose = pose.pose

    if isinstance(pose, PoseWithCovarianceStamped):
        pose = pose.pose.pose

    if isinstance(pose, Pose):
        quaternion = pose.orientation
        T = np.zeros((4, 4))
        R = quat_msg_to_matrix(quaternion)
        T[0:3, 0:3] = np.transpose(R)
        T[0, 3] = pose.position.x
        T[1, 3] = pose.position.y
        T[2, 3] = pose.position.z
        return T
    else:
        raise TypeError()


def parametrized(dec):
    def layer(*args, **kwargs):
        def repl(f):
            return dec(f, *args, **kwargs)

        return repl

    return layer


@parametrized
def retry_if_false(func, timeout=1, freq=1, log_msg=""):
    """
    Decorator which will make the function re-execute if it returns a False.
    Will add `timeout` and `freq` to kwargs.
    """
    default_timeout = timeout
    default_freq = freq

    def wrapper(*args, **kwargs):
        timeout = kwargs.pop("timeout", default_timeout)
        freq = kwargs.pop("freq", default_freq)
        rate = rospy.Rate(freq)
        N = timeout * freq
        success = False
        for i in range(N):
            try:
                success = func(*args, **kwargs)
            except rospy.ServiceException as e:
                rospy.logerr(e)
            success_string = "success" if success else "fail"

            rospy.loginfo(
                log_msg + ": attempt {0} of {1} = ".format(i, N) + str(success_string)
            )
            if success == True:
                break
            rate.sleep()
        return success

    return wrapper
