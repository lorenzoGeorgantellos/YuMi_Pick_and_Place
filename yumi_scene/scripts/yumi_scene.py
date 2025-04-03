#!/usr/bin/env python
import rospy
import moveit_commander
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
import os
import sys

def load_mesh(scene, name, pose, filename):
    scene.add_mesh(name, pose, filename)

def main():
    rospy.init_node('add_objects_to_scene', anonymous=True)
    
    # Inizializzazione MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = PlanningSceneInterface()
    
    # Tempo per l'inizializzazione della scena
    rospy.sleep(2)
    
    # Caricamento e posizionamento degli oggetti nella scena
    mesh_directory = "/home/lorenzo/YuMiRobotics1/src/yumi_description/meshes/fuller"
    
    objects = [
        {
            "name": "table1",
            "position": [0.15, 0.6, 0.441],
            "orientation": [0, 0, 0, 1],
            "filename": "Table_M230384.stl"
        },
        {
            "name": "table2",
            "position": [0.15,-0.6, 0.441],
            "orientation": [0, 0, 0, 1],
            "filename": "Table_M230384.stl"
        }
    ]
    
    for obj in objects:
        pose = PoseStamped()
        pose.header.frame_id = robot.get_planning_frame()
        pose.pose.position.x = obj["position"][0]
        pose.pose.position.y = obj["position"][1]
        pose.pose.position.z = obj["position"][2]
        pose.pose.orientation.x = obj["orientation"][0]
        pose.pose.orientation.y = obj["orientation"][1]
        pose.pose.orientation.z = obj["orientation"][2]
        pose.pose.orientation.w = obj["orientation"][3]
        
        filename = os.path.join(mesh_directory, obj["filename"])
        load_mesh(scene, obj["name"], pose, filename)
        print("Oggetto ",obj["name"]," importato")
    
    rospy.sleep(2)
    
if __name__ == "__main__":
    main()
