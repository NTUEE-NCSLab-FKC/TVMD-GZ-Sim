#!/usr/bin/env python3

import math
import sys
import threading
import xml.dom.minidom


import tf
import tf2_ros

import rospy
from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import ActuatorServos
from px4_msgs.msg import ControlAllocationMetaData
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleLocalPosition
from sensor_msgs.msg import JointState
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# For meta data visualization
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped

'''
header:
  seq: 13360
  stamp:
    secs: 1698155524
    nsecs: 675285577
  frame_id: ''
name:
  - module1_body_gimbal_joint
  - module1_gimbal_actuator_joint
  - module1_actuator_prop1_joint
  - module2_body_gimbal_joint
  - module2_gimbal_actuator_joint
  - module2_actuator_prop1_joint
  - module3_body_gimbal_joint
  - module3_gimbal_actuator_joint
  - module3_actuator_prop1_joint
  - module4_body_gimbal_joint
  - module4_gimbal_actuator_joint
  - module4_actuator_prop1_joint
position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
velocity: []
effort: []
'''

def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value

class MetaDataMarker:
    def __init_pseudo_forces(self) -> None:
        for aid in range(self.num_modules):
            # default settings
            # https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html
            self.marker_array.markers[aid].header.frame_id = "module{}_body_link".format(aid+1)
            self.marker_array.markers[aid].header.stamp = rospy.Time.now()
            self.marker_array.markers[aid].ns = "module{}".format(aid+1)
            self.marker_array.markers[aid].id = aid
            self.marker_array.markers[aid].type = Marker.LINE_STRIP
            self.marker_array.markers[aid].action = Marker.ADD
            
            self.marker_array.markers[aid].pose.position.x = 0
            self.marker_array.markers[aid].pose.position.y = 0
            self.marker_array.markers[aid].pose.position.z = 0.065
            self.marker_array.markers[aid].pose.orientation.x = 0.0
            self.marker_array.markers[aid].pose.orientation.y = 0.0
            self.marker_array.markers[aid].pose.orientation.z = 0.0
            self.marker_array.markers[aid].pose.orientation.w = 1.0

            # For LineStrips: only scale.x is used and it controls the width of the line segments.
            self.marker_array.markers[aid].scale.x = 0.01
            self.marker_array.markers[aid].scale.y = 0.01
            self.marker_array.markers[aid].scale.z = 0.01

            # How long the object should last before being automatically deleted.  0 means forever
            # self.marker_array.markers[aid].lifetime = 0.0
            
            # If this marker should be frame-locked, i.e. retransformed into its frame every timestep
            self.marker_array.markers[aid].frame_locked = True

            self.marker_array.markers[aid].points = [ Point() for i in range(self.num_modules+1) ]
            self.marker_array.markers[aid].colors = [ ColorRGBA() for i in range(self.num_modules+1)]
            for iter in range(self.num_modules):
                self.marker_array.markers[aid].points[iter].x = 0.0
                self.marker_array.markers[aid].points[iter].y = 0.0
                self.marker_array.markers[aid].points[iter].z = iter / 10.0
                self.marker_array.markers[aid].colors[iter].a = 1.0
                self.marker_array.markers[aid].colors[iter].r = 1 - iter / (self.num_modules + 1)
                self.marker_array.markers[aid].colors[iter].g = iter / (self.num_modules + 1)
                self.marker_array.markers[aid].colors[iter].b = 0.0

    @staticmethod
    def __get_default_arrow(id) -> Marker:
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "wrench"
        marker.id = id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)  
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]

        # scale.x is the shaft diameter, and 
        # scale.y is the head diameter. 
        # If scale.z is not zero, it specifies the head length.
        marker.scale.x = 0.01
        marker.scale.y = 0.02
        marker.scale.z = 0.0

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        marker.points = [Point(), Point()]
        marker.points[0].x = 0.0
        marker.points[0].y = 0.0
        marker.points[0].z = 0.0
        marker.points[1].x = 0.0
        marker.points[1].y = 0.0
        marker.points[1].z = 0.0
        return marker


    def __init__(self, _num_module: int) -> None:
        self.z_offset = 0.065
        self.scales = 0.1
        self.num_modules = _num_module
        self.marker_array = MarkerArray()

        # Pseudo forces
        self.marker_array.markers = [Marker() for i in range(self.num_modules)]
        self.__init_pseudo_forces()

        # Wrenches
        # wf_idx: wrench force index
        self.wf_idx = self.num_modules
        self.marker_array.markers.append(Marker())
        self.marker_array.markers[self.wf_idx] = MetaDataMarker.__get_default_arrow(5)

        # wt_idx: wrench torque index
        self.wt_idx = self.num_modules + 1
        self.marker_array.markers.append(Marker())
        self.marker_array.markers[self.wt_idx] = MetaDataMarker.__get_default_arrow(6)
        self.marker_array.markers[self.wt_idx].color.r = 0.0
        self.marker_array.markers[self.wt_idx].color.g = 1.0

        #wfd_idx: desired wrench force index
        self.wfd_idx = self.num_modules + 2
        self.marker_array.markers.append(Marker())
        self.marker_array.markers[self.wfd_idx] = MetaDataMarker.__get_default_arrow(7)
        self.marker_array.markers[self.wfd_idx].color.a = 0.3

        #wtd_idx: desired wrench torque index
        self.wtd_idx = self.num_modules + 3
        self.marker_array.markers.append(Marker())
        self.marker_array.markers[self.wtd_idx] = MetaDataMarker.__get_default_arrow(8)
        self.marker_array.markers[self.wtd_idx].color.a = 0.3
        self.marker_array.markers[self.wtd_idx].color.r = 0.0
        self.marker_array.markers[self.wtd_idx].color.g = 1.0

    
    """
    Update the positions of the pseudo-forces
    """
    @staticmethod
    def __num_valid_iterations(d: list) -> int:
        # The meta data is not cleared between control cycles, so the slots after the last
        # iteration hold stale values of previous cycles. The end is marked by increment = -1.
        # The slot 0 is always valid (initial guess, written before the first iteration).
        for iter in range(len(d)):
            if d[iter] < 0:
                return max(1, iter)
        return len(d)

    @staticmethod
    def __filter_point(point: Point, target: list, alpha: float) -> None:
        point.x += alpha * (target[0] - point.x)
        point.y += alpha * (target[1] - point.y)
        point.z += alpha * (target[2] - point.z)

    def update_forces(self, f_x: list, f_y: list, f_z: list, sat_indices: list, d:list, alpha: float = 1.0) -> None:
        n_iter = MetaDataMarker.__num_valid_iterations(d)

        # Target points of the pseudo-forces: points[k+1] = force after the k-th iteration
        targets = [[[0.0, 0.0, 0.0] for k in range(self.num_modules+1)] for aid in range(self.num_modules)]
        active_agent = [True for i in range(self.num_modules)]
        c = 0
        for iter in range(self.num_modules):
            if iter >= n_iter or c >= 1.0:
                # End of allocation
                active_agent = [False for i in range(self.num_modules)]

            for aid in range(self.num_modules):
                if active_agent[aid]:
                    idx = self.get_idx(iter, aid)
                    targets[aid][iter+1] = [f_x[idx] * self.scales, f_y[idx] * self.scales, f_z[idx] * self.scales]
                else:
                    targets[aid][iter+1] = targets[aid][iter]

            if iter < n_iter:
                c += d[iter]
                # Mark as saturated
                if sat_indices[iter] >= 0 and sat_indices[iter] < self.num_modules:
                    active_agent[sat_indices[iter]] = False

        for aid in range(self.num_modules):
            for k in range(1, self.num_modules+1):
                MetaDataMarker.__filter_point(self.marker_array.markers[aid].points[k], targets[aid][k], alpha)
                
    def __update_arrow(self, marker_idx: int, vec: list, alpha: float) -> None:
        MetaDataMarker.__filter_point(self.marker_array.markers[marker_idx].points[1], vec, alpha)

    def update_wrench(self, sp:list, alpha: float = 1.0) -> None:
        torque = sp[0:3]
        force = sp[3:6]
        self.__update_arrow(self.wf_idx, force, alpha)
        self.__update_arrow(self.wt_idx, torque, alpha)

    def update_desired_wrench(self, sp:list, alpha: float = 1.0) -> None:
        torque = sp[0:3]
        force = sp[3:6]
        self.__update_arrow(self.wfd_idx, force, alpha)
        self.__update_arrow(self.wtd_idx, torque, alpha)

    def get_idx(self, iter: int, agent: int) -> int:
        assert iter >= 0 and iter < self.num_modules
        assert agent >= 0 and agent < self.num_modules
        return iter * self.num_modules + agent
    
    def pretty_print(self, data: ControlAllocationMetaData):
        for aid in range(self.num_modules):
            print("Agent {}".format(aid), end=': ')
            for iter in range(self.num_modules):
                idx = self.get_idx(iter, aid)
                print("({:5.2f}, {:5.2f}, {:5.2f}) \t".format(
                    data.f_x[idx], data.f_y[idx], data.f_z[idx]), end='')
            print()
        print()


class Republisher:
    def __init_urdf(self, robot):
        # https://github.com/ros/joint_state_publisher/blob/noetic-devel/joint_state_publisher/src/joint_state_publisher/__init__.py
        robot = robot.getElementsByTagName('robot')[0]
        # Find all non-fixed joints
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype in ['fixed', 'floating', 'planar']:
                    continue
                name = child.getAttribute('name')
                self.joint_list.append(name)
                if jtype == 'continuous':
                    minval = -math.pi
                    maxval = math.pi
                else:
                    try:
                        limit = child.getElementsByTagName('limit')[0]
                        minval = float(limit.getAttribute('lower'))
                        maxval = float(limit.getAttribute('upper'))
                    except:
                        rospy.logwarn("%s is not fixed, nor continuous, but limits are not specified!" % name)
                        continue

                safety_tags = child.getElementsByTagName('safety_controller')
                if self.use_small and len(safety_tags) == 1:
                    tag = safety_tags[0]
                    if tag.hasAttribute('soft_lower_limit'):
                        minval = max(minval, float(tag.getAttribute('soft_lower_limit')))
                    if tag.hasAttribute('soft_upper_limit'):
                        maxval = min(maxval, float(tag.getAttribute('soft_upper_limit')))

                mimic_tags = child.getElementsByTagName('mimic')
                if self.use_mimic and len(mimic_tags) == 1:
                    tag = mimic_tags[0]
                    entry = {'parent': tag.getAttribute('joint')}
                    if tag.hasAttribute('multiplier'):
                        entry['factor'] = float(tag.getAttribute('multiplier'))
                    if tag.hasAttribute('offset'):
                        entry['offset'] = float(tag.getAttribute('offset'))

                    self.dependent_joints[name] = entry
                    continue

                if name in self.dependent_joints:
                    continue

                zeroval = get_param("zeros/" + name)
                if not zeroval:
                    if minval > 0 or maxval < 0:
                        zeroval = (maxval + minval)/2
                    else:
                        zeroval = 0

                joint = {'min': minval, 'max': maxval, 'zero': zeroval}
                if self.pub_def_positions:
                    joint['position'] = zeroval
                if self.pub_def_vels:
                    joint['velocity'] = 0.0
                if self.pub_def_efforts:
                    joint['effort'] = 0.0

                if jtype == 'continuous':
                    joint['continuous'] = True
                self.free_joints[name] = joint

    def __init_baselink_transform(self):
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        # Pose of base_link (FLU) in world_frame (NWU: x-North, y-West, z-Up)
        transform.header.frame_id = "world_frame"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0

        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)  
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        return transform

    def __init__(self) -> None:
        rospy.init_node('tvmd_republisher')
        self.num_modules = 4
        # Callbacks run in separate threads, and the publishing timer in another one
        self.lock = threading.Lock()

        self.free_joints = {}
        self.joint_list = [] # for maintaining the original order of the joints
        self.dependent_joints = get_param("dependent_joints", {})
        self.use_mimic = get_param('use_mimic_tags', True)
        self.use_small = get_param('use_smallest_joint_limits', True)

        self.pub_def_positions = get_param("publish_default_positions", True)
        self.pub_def_vels = get_param("publish_default_velocities", False)
        self.pub_def_efforts = get_param("publish_default_efforts", False)

        description = get_param('robot_description')
        if description is None:
            raise RuntimeError('The robot_description parameter is required and not set.')
        robot = xml.dom.minidom.parseString(description)
        self.__init_urdf(robot)

        # joint state message
        self.joint_msg = JointState()
        self.joint_msg.position = [0.0 for i in range(3 * self.num_modules)]
        # joint_names = ["_body_gimbal_joint", "_gimbal_actuator_joint", "_actuator_prop1_joint", "_actuator_prop2_joint"]
        # self.joint_msg.name = ["module{}{}".format(i+1, n) for i in range(self.num_modules) for n in joint_names]
        self.joint_msg.name = [str(name) for name in self.joint_list]

        # joint state publisher
        self.pub = rospy.Publisher("joint_states", JointState, queue_size=1)

        # TODO: Load parameters from urdf file, and calculate the scales for joints

        # baselink pose (attitude + position) transformation broadcaster.
        # The pose changes over time, so a (non-static) TransformBroadcaster is required.
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.baselink_transform = self.__init_baselink_transform()

        topic = 'visualization_marker_array'
        self.marker_pub = rospy.Publisher(topic, MarkerArray, queue_size=1)
        self.marker_data = MetaDataMarker(self.num_modules)
        self.marker_updated = False

        # First-order low-pass filters for smoother visualization, time constants [s] (0: disabled).
        # They run on every incoming message (before the down-sampling to `rate`), using the PX4 timestamps.
        self.force_tau = float(get_param("force_filter_tau", 0.05))    # pseudo-forces and wrenches
        self.servo_tau = float(get_param("servo_filter_tau", 0.03))    # gimbal joint angles
        self.last_meta_timestamp = 0
        self.last_servo_timestamp = 0

        # Trajectory of base_link in world_frame
        self.path_pub = rospy.Publisher("trajectory", Path, queue_size=1)
        self.path_msg = Path()
        self.path_msg.header.frame_id = "world_frame"
        self.path_max_length = int(get_param("path_max_length", 5000))
        self.path_min_dist = float(get_param("path_min_distance", 0.01))   # [m]
        self.last_position_timestamp = 0

    @staticmethod
    def filter_gain(tau: float, timestamp: int, last_timestamp: int) -> float:
        dt = (timestamp - last_timestamp) * 1e-6
        # No filter, first sample, or a jump in time (e.g., seeking in the replay)
        if tau <= 0.0 or last_timestamp == 0 or dt <= 0.0 or dt > 0.5:
            return 1.0
        return 1.0 - math.exp(-dt / tau)

    def data_update(self, data, mode):
        if mode == "servos":
            alpha = Republisher.filter_gain(self.servo_tau, data.timestamp, self.last_servo_timestamp)
            self.last_servo_timestamp = data.timestamp
            # Handle servos: gimbal (idx 0) and actuator (idx 1) for each module
            for i in range(4):
                for j in range(2):
                    # joint order: body_gimbal_joint, gimbal_actuator_joint, actuator_prop1_joint
                    # j=0 -> gimbal (idx 0), j=1 -> actuator (idx 1)
                    joint_idx = 3*i + (1-j)
                    ctrl_idx = 2*i + j
                    joint_name = self.joint_list[joint_idx]
                    joint = self.free_joints[joint_name]

                    factor = (joint['max'] - joint['min']) / 2
                    offset = (joint['max'] + joint['min']) / 2
                    target = data.control[ctrl_idx] * factor + offset
                    self.joint_msg.position[joint_idx] += alpha * (target - self.joint_msg.position[joint_idx])
        else:
            # Handle motors: only prop1 (idx 2) for each module
            for i in range(4):
                joint_idx = 3*i + 2  # prop1 is at index 2 within each module
                ctrl_idx = i  # Only 4 motors now (one per module)
                self.joint_msg.position[joint_idx] += data.control[ctrl_idx]
        # Published by publish_callback() at a fixed rate
    
    def motor_listener(self, data: ActuatorMotors):        
        with self.lock:
            self.data_update(data, "motors")
    
    def servo_listener(self, data: ActuatorServos):
        with self.lock:
            self.data_update(data, "servos")
    
    def meta_data_listener(self, data: ControlAllocationMetaData):
        # print(data.saturated_idx)
        # print(data.increment)
        # self.marker_data.pretty_print(data)
        
        # update marker data, published by publish_callback()
        with self.lock:
            alpha = Republisher.filter_gain(self.force_tau, data.timestamp, self.last_meta_timestamp)
            self.last_meta_timestamp = data.timestamp
            self.marker_data.update_forces(data.f_x, data.f_y, data.f_z, data.saturated_idx, data.increment, alpha)
            self.marker_data.update_wrench(data.allocated_control, alpha)
            self.marker_data.update_desired_wrench(data.control_sp, alpha)
            self.marker_updated = True

    def vehicle_attitude_listener(self, data: VehicleAttitude):
        # Quaternion rotation from the FRD body frame to the NED earth frame, wxyz -> xyzw
        q = [data.q[i] for i in [1, 2, 3, 0]]

        # Note: q is already the corrected estimate, delta_q_reset must NOT be applied here.
        # delta_q_reset is only the change of q at the last estimator reset (e.g., yaw alignment,
        # EKF instance switch), and it is kept until the next reset. It is used by PX4 only when
        # quat_reset_counter changes (see mc_att_control). Applying it to every sample results in
        # a constant body-frame rotation offset (a wrong yaw, or flipped roll/pitch for a 180 deg reset).

        # FRD->NED attitude expressed as FLU->NWU, i.e., conjugated by a 180 deg rotation about x.
        with self.lock:
            self.baselink_transform.transform.rotation.x = q[0]
            self.baselink_transform.transform.rotation.y = -q[1]
            self.baselink_transform.transform.rotation.z = -q[2]
            self.baselink_transform.transform.rotation.w = q[3]

    def vehicle_local_position_listener(self, data: VehicleLocalPosition):
        with self.lock:
            # Replay restarted (e.g., looping in PlotJuggler): clear the trajectory
            if data.timestamp < self.last_position_timestamp:
                self.path_msg.poses = []
            self.last_position_timestamp = data.timestamp

            # NED -> NWU
            translation = self.baselink_transform.transform.translation
            if data.xy_valid:
                translation.x = data.x
                translation.y = -data.y
            if data.z_valid:
                translation.z = -data.z

    def __append_path(self, stamp):
        translation = self.baselink_transform.transform.translation
        if len(self.path_msg.poses) > 0:
            last = self.path_msg.poses[-1].pose.position
            dist = math.sqrt((translation.x - last.x)**2 + (translation.y - last.y)**2 + (translation.z - last.z)**2)
            if dist < self.path_min_dist:
                return

        pose = PoseStamped()
        pose.header.frame_id = "world_frame"
        pose.header.stamp = stamp
        pose.pose.position.x = translation.x
        pose.pose.position.y = translation.y
        pose.pose.position.z = translation.z
        rotation = self.baselink_transform.transform.rotation
        pose.pose.orientation.x = rotation.x
        pose.pose.orientation.y = rotation.y
        pose.pose.orientation.z = rotation.z
        pose.pose.orientation.w = rotation.w
        self.path_msg.poses.append(pose)
        if len(self.path_msg.poses) > self.path_max_length:
            del self.path_msg.poses[:len(self.path_msg.poses) - self.path_max_length]

    def publish_callback(self, event=None):
        # Publish the joint states, the base_link pose, and the markers with the SAME stamp,
        # such that Rviz renders the base_link and the modules (agents) at a consistent time.
        stamp = rospy.Time.now()
        with self.lock:
            self.joint_msg.header.stamp = stamp
            self.pub.publish(self.joint_msg)

            self.baselink_transform.header.stamp = stamp
            self.broadcaster.sendTransform(self.baselink_transform)

            self.__append_path(stamp)
            self.path_msg.header.stamp = stamp
            self.path_tick += 1
            if self.path_tick >= self.path_decimation:
                self.path_tick = 0
                self.path_pub.publish(self.path_msg)

            if self.marker_updated or event is None:
                for marker in self.marker_data.marker_array.markers:
                    marker.header.stamp = stamp
                self.marker_pub.publish(self.marker_data.marker_array)
                self.marker_updated = False

    def run(self):
        # Rate of publishing joint states, TF and markers to Rviz.
        # Incoming data is buffered (latest only) and republished at this rate,
        # independent of the (possibly much higher) rates of the PX4 topics.
        hz = float(get_param("rate", 50))
        path_hz = float(get_param("path_rate", 10))
        self.path_decimation = max(1, int(round(hz / path_hz)))
        self.path_tick = 0

        # Allow Rviz to initialize robot model
        r = rospy.Rate(10)
        for k in range(10):
            self.publish_callback()
            r.sleep()
        
        print("Start republishing")
        # queue_size=1: always process the latest message only, otherwise the unbounded rospy
        # queues pile up (e.g., control_allocation_meta_data is logged at full rate),
        # and the visualization lags behind more and more.
        rospy.Subscriber("px4/actuator_motors", ActuatorMotors, self.motor_listener, queue_size=1)
        rospy.Subscriber("px4/actuator_servos", ActuatorServos, self.servo_listener, queue_size=1)
        rospy.Subscriber("px4/control_allocation_meta_data", ControlAllocationMetaData, self.meta_data_listener, queue_size=1)
        rospy.Subscriber("px4/vehicle_attitude", VehicleAttitude, self.vehicle_attitude_listener, queue_size=1)
        rospy.Subscriber("px4/vehicle_local_position", VehicleLocalPosition, self.vehicle_local_position_listener, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / hz), self.publish_callback)
        rospy.spin()
        return

if __name__ == '__main__':
    rep = Republisher()
    rep.run()