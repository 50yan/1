#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import DeleteModel


def main():
    rospy.init_node("delete_existing_model", anonymous=True)
    model_name = rospy.get_param("~model_name", "smart_car")
    timeout = float(rospy.get_param("~timeout", 10.0))
    try:
        rospy.wait_for_service("/gazebo/delete_model", timeout=timeout)
        delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        result = delete_model(model_name)
        if result.success:
            rospy.loginfo("Deleted existing Gazebo model: %s", model_name)
        else:
            rospy.loginfo("No existing Gazebo model deleted for %s: %s", model_name, result.status_message)
    except Exception as exc:
        rospy.logwarn("Skipping existing model cleanup for %s: %s", model_name, exc)


if __name__ == "__main__":
    main()