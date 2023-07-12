import rclpy
import os.path
import json
from rclpy.node import Node

from cslam_common_interfaces.msg import PoseGraph
from cslam_common_interfaces.msg import PoseGraphValue
from cslam_common_interfaces.msg import PoseGraphEdge
from cslam_common_interfaces.msg import MultiRobotKey
from geometry_msgs.msg import Pose

class CslamStorage(Node):

    def __init__(self):
        super().__init__('cslam_storage')
        # self.subscription  # prevent unused variable warning
        self.pose_graph_storage_subscriber = self.create_subscription(
                PoseGraph, '/cslam/viz/pose_graph', self.pose_graph_storage_callback, 10)
        self.pose_graph_to_store = {}

        self.declare_parameters(
            namespace='',
            parameters=[# TODO: test if this default value works
                        # TODO: raise exception if map_path is not given when enable_map_storage or 
                        # enable_map_reading are true, or use a default path 
                        ('map_path', ''),
                        ('pose_graph_file_name', 'pose_graph.json'), 
                        ('enable_map_storage', False),
                        ('enable_map_reading', False),]),
        self.params = {}
        self.params['map_path'] = self.get_parameter(
        'map_path').value
        self.params['pose_graph_file_name'] = self.get_parameter(
            'pose_graph_file_name').value
        self.params['enable_map_storage'] = self.get_parameter(
            'enable_map_storage').value
        self.params['enable_map_reading'] = self.get_parameter(
            'enable_map_reading').value

    def store_pose_graph(self, msg):
        # Make sure that intermediate directories exist
        os.makedirs(self.params["map_path"], exist_ok=True)

        pose_graph_path = self.params["map_path"] + "/" + self.params["pose_graph_file_name"]
        with open(pose_graph_path, "w+") as json_file:
            # TODO: handle case when there is a previous different data 
            # Read file is not empty
            # if os.path.getsize(pose_graph_path) != 0:
            #     pose_graph_to_store = json.load(json_file)

            json.dump(self.pose_graph_to_store, json_file)

    def pose_graph_value_to_dict(self, pose_graph_value):
        """ Convert cslam_common_interfaces/msg/PoseGraphValue to dict
            Attention: the "key" property is not converted 
            TODO: maybe think about add key property
        """
        return {
            "position": {
                "x": pose_graph_value.pose.position.x,
                "y": pose_graph_value.pose.position.y,
                "z": pose_graph_value.pose.position.z
            },
            "orientation": {
                "x": pose_graph_value.pose.orientation.x,
                "y": pose_graph_value.pose.orientation.y,
                "z": pose_graph_value.pose.orientation.z,
                "w": pose_graph_value.pose.orientation.w
            }
        }
    
    def pose_graph_edge_to_dict(self, edge):
        """Convert cslam_common_interfaces/msg/PoseGraphEdge to dict""" 
        return {
            "key_from": {
                "robot_id": edge.key_from.robot_id,
                "keyframe_id": edge.key_from.keyframe_id
            },
            "key_to": {
                "robot_id": edge.key_to.robot_id,
                "keyframe_id": edge.key_to.keyframe_id
            },
            "measurement": {
                "position": {
                    "x": edge.measurement.position.x,
                    "y": edge.measurement.position.y,
                    "z": edge.measurement.position.z
                },
                "orientation": {
                    "x": edge.measurement.orientation.x,
                    "y": edge.measurement.orientation.y,
                    "z": edge.measurement.orientation.z,
                    "w": edge.measurement.orientation.w,
                },
            },
            "noise_std": edge.noise_std.tolist()
        }

    # Conversion methods
    def dict_to_pose(self, dict):
        """Convert dict to geometry_msgs/msg/Pose""" 
        pose = Pose()
        pose.position.x = dict['position']['x']
        pose.position.y = dict['position']['y']
        pose.position.z = dict['position']['z']
        pose.orientation.x = dict['orientation']['x']
        pose.orientation.y = dict['orientation']['y']
        pose.orientation.z = dict['orientation']['z']
        pose.orientation.w = dict['orientation']['w']
        return pose

    def dict_to_pose_graph_value(self, dict, robot_id, keyframe_id):
        """ Convert dict to cslam_common_interfaces/msg/PoseGraphValue
            Attention: the "key" property is not converted
        """
        pose_graph_value = PoseGraphValue()
        pose_graph_value.key = MultiRobotKey()
        pose_graph_value.key.robot_id = robot_id
        pose_graph_value.key.keyframe_id = keyframe_id
        pose_graph_value.pose = self.dict_to_pose(dict)
        return pose_graph_value
    
    def dict_to_pose_graph_edge(self, dict):
        """ Convert dict to cslam_common_interfaces/msg/PoseGraphEdge """
        pose_graph_edge = PoseGraphEdge()
        pose_graph_edge.key_from = MultiRobotKey()
        pose_graph_edge.key_from.robot_id = int(dict["key_from"]["robot_id"])
        pose_graph_edge.key_from.keyframe_id = int(dict["key_from"]["keyframe_id"])
        pose_graph_edge.key_to = MultiRobotKey()
        pose_graph_edge.key_to.robot_id = int(dict["key_to"]["robot_id"])
        pose_graph_edge.key_to.keyframe_id = int(dict["key_to"]["keyframe_id"])
        pose_graph_edge.measurement = self.dict_to_pose(dict["measurement"])
        pose_graph_edge.noise_std = dict["noise_std"]                    
        return pose_graph_edge

    def pose_graph_storage_callback(self, msg):    
        # Initialize robot pose graph if it doesn't exist yet
        if msg.robot_id not in self.pose_graph_to_store:
            self.pose_graph_to_store[msg.robot_id] = {
                "edges": {},
                "values": {}
            }

        # Convert PoseGraphValue and PoseGraphEdge to dict (json) to be stored 
        for pose in msg.values:
            self.pose_graph_to_store[msg.robot_id]["values"][pose.key.keyframe_id] = self.pose_graph_value_to_dict(pose)
        self.pose_graph_to_store[msg.robot_id]["edges"] = list(map(self.pose_graph_edge_to_dict, msg.edges))

        self.store_pose_graph(msg)


def main(args=None):
    rclpy.init(args=args)

    cslam_storage = CslamStorage()

    rclpy.spin(cslam_storage)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cslam_storage.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()