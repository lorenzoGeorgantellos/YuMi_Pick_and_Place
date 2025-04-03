#!/usr/bin/env python
# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import yaml
import os
import math
import tf.transformations
import pickle
from datetime import datetime
import time

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True
# MoveGroupPythonInterfaceTutorial

class YuMiMoveGroupPythonInterface(object):
    """YuMiMoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(YuMiMoveGroupPythonInterface, self).__init__()
        self.label_names = []  # Inizializza l'attributo label_names

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("yumi_mg_py", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "left_arm"
        group_name2 = "right_arm"
        group_name3 = "left_gripper"
        group_name4 = "right_gripper"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        # move_group.set_planner_id("FMTkConfigDefault")
        move_group2 = moveit_commander.MoveGroupCommander(group_name2)
        # move_group2.set_planner_id("FMTkConfigDefault")
        move_group3 = moveit_commander.MoveGroupCommander(group_name3)
        move_group4 = moveit_commander.MoveGroupCommander(group_name4)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        # eef_link = "L7"
        eef_link2 = move_group2.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.move_group2 = move_group2
        self.move_group3 = move_group3
        self.move_group4 = move_group4
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.eef_link2 = eef_link2
        self.group_names = group_names

    def go_to_joint_state(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = -0.663225
        joint_goal[1] = -1.15192
        joint_goal[2] = 1.06465
        joint_goal[3] = -0.0698132
        joint_goal[4] = -0.0174533
        joint_goal[5] = 0.593412
        joint_goal[6] = 0.855211

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def start_orientation(self):
        move_group = self.move_group
        current_pose = move_group.get_current_pose().pose
        orientation = current_pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        print("Current orientation (quaternion):", quaternion)  # Stampa i valori del quaternione
        return quaternion
    

    def plan_cartesian_path(self, scale, asse):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = move_group.get_current_pose().pose

        print(f"Posizione attuale del robot: x={wpose.position.x}, y={wpose.position.y}, z={wpose.position.z}")

        waypoints.append(copy.deepcopy(wpose))
        
        if asse == "x":
            wpose.position.x += scale 
        elif asse == "y":
            wpose.position.y += scale 
        elif asse == "z":
            wpose.position.z += scale 
        print(f"Posizione attuale del robot: x={wpose.position.x}, y={wpose.position.y}, z={wpose.position.z}")
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,       # eef_step
            0.0        # jump_threshold
            )
          
        print("Eseguito")

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction
    
    
    def plan_traj1(self, scale=1):
        move_group = self.move_group

        ## Percorsi Cartesiani
        waypoints = []

        # Ottieni la posa attuale
        wpose = move_group.get_current_pose().pose

        # Waypoint 1: posizione attuale
        waypoints.append(copy.deepcopy(wpose))

        # Crea un nuovo set di waypoints per includere il movimento verso la configurazione dei giunti finali
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = -0.506145
        joint_goal[1] = -0.802851
        joint_goal[2] = 1.291544
        joint_goal[3] = 0.017453
        joint_goal[4] = 0.925025
        joint_goal[5] = 0.715585
        joint_goal[6] = 0.261799

        move_group.set_joint_value_target(joint_goal)
        # Pianifica il percorso per raggiungere la configurazione dei giunti finali
        joint_plan = move_group.plan()[1]  # Estrai il piano dal tuple
        print("Planner di Default:", move_group.get_planner_id())
        return joint_plan
    
    def plan_traj2(self, scale=1):
        move_group = self.move_group

        ## Percorsi Cartesiani
        waypoints = []

        # Ottieni la posa attuale
        wpose = move_group.get_current_pose().pose

        # Waypoint 1: posizione attuale
        waypoints.append(copy.deepcopy(wpose))

        # Crea un nuovo set di waypoints per includere il movimento verso la configurazione dei giunti finali
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = -0.925025
        joint_goal[1] = -1.32645
        joint_goal[2] = 1.064651
        joint_goal[3] = 0.558505
        joint_goal[4] = 0.174533
        joint_goal[5] = 1.012291
        joint_goal[6] = 0.383972

        move_group.set_joint_value_target(joint_goal)
        # Pianifica il percorso per raggiungere la configurazione dei giunti finali
        joint_plan = move_group.plan()[1]  # Estrai il piano dal tuple
        print("Planner di Default:", move_group.get_planner_id())

        return joint_plan
    
    def plan_traj1_r(self, scale=1):
        move_group = self.move_group2

        ## Percorsi Cartesiani
        waypoints = []

        # Ottieni la posa attuale
        wpose = move_group.get_current_pose().pose

        # Waypoint 1: posizione attuale
        waypoints.append(copy.deepcopy(wpose))

        # Crea un nuovo set di waypoints per includere il movimento verso la configurazione dei giunti finali
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0.680678
        joint_goal[1] = -0.750492
        joint_goal[2] = -1.186824 
        joint_goal[3] = 0.593412 
        joint_goal[4] = 0.558505
        joint_goal[5] = 0.959931
        joint_goal[6] = -1.047198

        move_group.set_joint_value_target(joint_goal)
        # Pianifica il percorso per raggiungere la configurazione dei giunti finali
        joint_plan = move_group.plan()[1]  # Estrai il piano dal tuple
        print("Planner di Default:", move_group.get_planner_id())

        return joint_plan
    
    def plan_traj2_r(self, scale=1):
        move_group = self.move_group2

        ## Percorsi Cartesiani
        waypoints = []

        # Ottieni la posa attuale
        wpose = move_group.get_current_pose().pose

        # Waypoint 1: posizione attuale
        waypoints.append(copy.deepcopy(wpose))

        # Crea un nuovo set di waypoints per includere il movimento verso la configurazione dei giunti finali
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0.610865 
        joint_goal[1] = -0.471239 
        joint_goal[2] = -1.850049
        joint_goal[3] = -0.20944
        joint_goal[4] = 0.15708 
        joint_goal[5] = 0.575959
        joint_goal[6] = -1.064651

        move_group.set_joint_value_target(joint_goal)
        # Pianifica il percorso per raggiungere la configurazione dei giunti finali
        joint_plan = move_group.plan()[1]  # Estrai il piano dal tuple
        print("Planner di Default:", move_group.get_planner_id())
        return joint_plan
    
    def combine_plans(self, plan1, plan2):
        combined_trajectory = plan1.joint_trajectory
        combined_trajectory.points.extend(plan2.joint_trajectory.points)
        combined_plan = plan1
        combined_plan.joint_trajectory = combined_trajectory
        return combined_plan

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL
    

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Received
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node was just created (https://github.com/ros/ros_comm/issues/176),
        ## or dies before actually publishing the scene update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed.
        ## To avoid waiting for scene updates like this at all, initialize the
        ## planning scene interface with  ``synchronous = True``.
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    
    def add_object(self, timeout=4):
        scene = self.scene

        ## Aggiunta di oggetti alla scena di pianificazione
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Per prima cosa, creiamo una scatola nella scena di pianificazione alla posizione del dito sinistro:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "baselink"

        # Imposta la posizione e l'orientamento della scatola
        box_pose.pose.position.x = 0.4
        box_pose.pose.position.y = 0.45
        box_pose.pose.position.z = 0.103

        # Converti RPY in quaternione
        quaternion = tf.transformations.quaternion_from_euler(math.pi / 2, 0, math.pi / 2)
        box_pose.pose.orientation.x = quaternion[0]
        box_pose.pose.orientation.y = quaternion[1]
        box_pose.pose.orientation.z = quaternion[2]
        box_pose.pose.orientation.w = quaternion[3]

        label_name = f"box"
        self.label_names.append(label_name)
        scene.add_box(label_name, box_pose, size=(0.1, 0.2, 0.03))

        ## END_SUB_TUTORIAL
        # Copia delle variabili locali nelle variabili di classe
        self.label_names.append(label_name)
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self,box_name, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        # box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = "left_gripper"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )
    
    def attach_box_ra(self,box_name, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        # box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link2 = self.eef_link2
        group_names = self.group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = "right_gripper"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link2, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self, box_name, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)

        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )
    
    def detach_box_ra(self, box_name, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        scene = self.scene
        eef_link = self.eef_link2

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)

        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    def remove_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )

    def save_plan(self, plan, filename):
        with open(filename, 'wb') as f:
            pickle.dump(plan, f)
    def load_plan(self, filename):
        with open(filename, 'rb') as f:
            plan = pickle.load(f)
        return plan
    
    def control_gripper(self, width):
        """
        Pianifica ed esegue il movimento del gripper impostando la sua apertura a `width` (in metri).
        """
        move_group = self.move_group3
        joint_goal = move_group.get_current_joint_values()

        joint_goal[0] = width  # Imposta la posizione del giunto prismatico

        move_group.go(joint_goal, wait=True)
        move_group.stop()

        print(f"Gripper impostato a {width} m")
        return all_close(joint_goal, move_group.get_current_joint_values(), 0.01)
    
    def control_gripper_r(self, width):
        """
        Pianifica ed esegue il movimento del gripper impostando la sua apertura a `width` (in metri).
        """
        move_group = self.move_group4
        joint_goal = move_group.get_current_joint_values()

        joint_goal[0] = width  # Imposta la posizione del giunto prismatico

        move_group.go(joint_goal, wait=True)
        move_group.stop()

        print(f"Gripper impostato a {width} m")
        return all_close(joint_goal, move_group.get_current_joint_values(), 0.01)


def main():
    
        yumi = YuMiMoveGroupPythonInterface()
        yumi.start_orientation()

        yumi.add_object()
        
        # ======================================= SOLO PIANIFICAZIONE =============================================================================
        # Decommentare questa sezione se si vuole effettuare la pianificazione non utilizzando i path già salvati
        # Info utili per il salvataggio dei piani
        # if not os.path.exists('saved_plan_chomp'):
        #     os.makedirs('saved_plan_chomp')

        # # # Nome del file con la data e l'ora corrente
        # current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        # file_traj_1 = os.path.join('saved_plan_chomp', f"traj1_{current_time}.pkl")
        # file_traj_2 = os.path.join('saved_plan_chomp', f"traj2_{current_time}.pkl")

        # file_traj_1_r = os.path.join('saved_plan_chomp', f"traj1_r{current_time}.pkl")
        # file_traj_2_r = os.path.join('saved_plan_chomp', f"traj2_r{current_time}.pkl")
        # yumi.control_gripper(0.022)
        # start_plan_1_l = time.time()
        # traj_1 = yumi.plan_traj1()
        # end_plan_1_l = time.time()
        # print(f"Tempo totale di pianificazione della prima traiettoria del braccio sinistro: {end_plan_1_l-start_plan_1_l} secondi")
        
        # yumi.execute_plan(traj_1)
        # yumi.save_plan(traj_1, file_traj_1)

        # start_plan_2_l = time.time()
        # traj_2 = yumi.plan_traj2()
        # end_plan_2_l = time.time()
        # print(f"Tempo totale di pianificazione della seconda traiettoria del braccio sinistro: {end_plan_2_l-start_plan_2_l} secondi")
        
        # yumi.execute_plan(traj_2)
        # yumi.save_plan(traj_2, file_traj_2)

        # yumi.control_gripper_r(0.022)

        # start_plan_1_r = time.time()
        # traj_1_r = yumi.plan_traj1_r()
        # end_plan_1_r = time.time()
        # print(f"Tempo totale di pianificazione della prima traiettoria del braccio destro: {end_plan_1_r-start_plan_1_r} secondi")

        # yumi.execute_plan(traj_1_r)
        # yumi.save_plan(traj_1_r, file_traj_1_r)

        # start_plan_2_r = time.time()
        # traj_2_r = yumi.plan_traj2_r()
        # end_plan_2_r = time.time()
        # print(f"Tempo totale di pianificazione della seconda traiettoria del braccio destro: {end_plan_2_r-start_plan_2_r} secondi")

        # yumi.execute_plan(traj_2_r)
        # yumi.save_plan(traj_2_r, file_traj_2_r)

        # # ===========================================================================================================================================================
        # # ========================================== ESECUZIONE DI PATH GIA' PIANIFICATI ============================================================================
        traj_1 = yumi.load_plan("saved_plan_chomp/traj1_20250218_112705.pkl")
        traj_2 = yumi.load_plan("saved_plan_chomp/traj2_20250218_112705.pkl")
        traj_1_r = yumi.load_plan("saved_plan_chomp/traj1_r20250218_112705.pkl")
        traj_2_r = yumi.load_plan("saved_plan_chomp/traj2_r20250218_112705.pkl")
        
        yumi.control_gripper(0.022)

        start1 = time.time()
        yumi.execute_plan(traj_1)
        end1 = time.time()
        print(f"Tempo totale di esecuzione della prima traiettoria del braccio sinistro: {end1-start1} secondi")

        yumi.attach_box("box")

        start2 = time.time()
        yumi.execute_plan(traj_2)
        end2 = time.time()
        print(f"Tempo totale di esecuzione della seconda traiettoria del braccio sinistro: {end2-start2} secondi")
        yumi.detach_box("box")

        yumi.control_gripper_r(0.022)

        start1_r = time.time()
        yumi.execute_plan(traj_1_r)
        end1_r = time.time()
        print(f"Tempo totale di esecuzione della prima traiettoria del braccio destro: {end1_r-start1_r} secondi")
        yumi.attach_box_ra("box")

        start2_r = time.time()
        yumi.execute_plan(traj_2_r)
        end2_r = time.time()
        print(f"Tempo totale di esecuzione della seconda traiettoria del braccio destro: {end2_r-start2_r} secondi")
        yumi.detach_box_ra("box")

if __name__ == "__main__":
    main()