#!/usr/bin/env python3
import rospy
import roslaunch
from std_srvs.srv import Trigger, TriggerResponse
import threading
import subprocess

# Dictionary to hold active launch processes
launch_processes = {}

#function to launch a launch file
def launch_file(launch_file_name, key):
    rospy.loginfo(f"Launching {key} nodes in thread: {threading.current_thread().name}")
    try:

        if key in launch_processes:
            return TriggerResponse(success=False, message=f"{key} is already running.")
        

        #subprocess to spawn separate process for roslaunch
        process = subprocess.Popen(['roslaunch', launch_file_name])


        #Store process
        launch_processes[key] = process
        rospy.loginfo(f"{key} nodes have been launched successfully.")
        return TriggerResponse(success=True, message=f"{key} launched.")
    except Exception as e:
        rospy.logerr(f"Error launching {key}: {str(e)}")
        return TriggerResponse(success=False, message=str(e))

#function to stop a launch file
def stop_file(key):
    rospy.loginfo(f"Stopping {key} nodes in thread: {threading.current_thread().name}")
    try:
        if key not in launch_processes:
            return TriggerResponse(success=False, message=f"{key} is not running.")


        #Kill process 
        process = launch_processes[key]
        process.terminate()
        process.wait()

        #Remove from the dictionary
        del launch_processes[key]
        rospy.loginfo(f"{key} nodes have been stopped successfully.")
        return TriggerResponse(success=True, message=f"{key} stopped.")
    except Exception as e:
        rospy.logerr(f"Error stopping {key}: {str(e)}")
        return TriggerResponse(success=False, message=str(e))



def launch_line_follower(req):
    return launch_file("/home/mag/catkin_ws_pfe/src/line_follower_robot/launch/line_follower_robot.launch", "line_follower_robot.launch")

def stop_line_follower(req):
    return stop_file("line_follower_robot.launch")

def launch_camera_qr(req):
    return launch_file("/home/mag/catkin_ws_pfe/src/camera/launch/line_with_qr.launch", "line_with_qr.launch")

def stop_camera_qr(req):
    return stop_file("line_with_qr.launch")

def launch_obstacle_avoidance(req):
    return launch_file("/home/mag/catkin_ws_pfe/src/line_with_ultrasound/launch/avoid_obstacle.launch", "avoid_obstacle.launch")

def stop_obstacle_avoidance(req):
    return stop_file("avoid_obstacle.launch")

def launch_line_with_avoidance(req):
    return launch_file("/home/mag/catkin_ws_pfe/src/line_with_ultrasound/launch/line_with_ultrasound.launch", "line_with_ultrasound.launch")

def stop_line_with_avoidance(req):
    return stop_file("line_with_ultrasound.launch")

def launch_teleop_twist(req):
    return launch_file("/home/mag/catkin_ws_pfe/src/line_follower_robot/launch/teleop_twist.launch", "teleop_twist.launch")

def stop_teleop_twist(req):
    return stop_file("teleop_twist.launch")

if __name__ == "__main__":
    rospy.init_node('multi_launch_stop_service')

    #services for launching and stopping each launch files
    rospy.Service('/launch_line_follower', Trigger, launch_line_follower)
    rospy.Service('/stop_line_follower', Trigger, stop_line_follower)

    rospy.Service('/launch_camera_qr', Trigger, launch_camera_qr)
    rospy.Service('/stop_camera_qr', Trigger, stop_camera_qr)

    rospy.Service('/launch_obstacle_avoidance', Trigger, launch_obstacle_avoidance)
    rospy.Service('/stop_obstacle_avoidance', Trigger, stop_obstacle_avoidance)

    rospy.Service('/launch_line_with_avoidance', Trigger, launch_line_with_avoidance)
    rospy.Service('/stop_line_with_avoidance', Trigger, stop_line_with_avoidance)

    rospy.Service('/launch_teleop_twist', Trigger, launch_teleop_twist)
    rospy.Service('/stop_teleop_twist', Trigger, stop_teleop_twist)

    rospy.loginfo("Ready to launch and stop multiple node groups.")
    rospy.spin()
