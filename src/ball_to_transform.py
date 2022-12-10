#!/usr/bin/env python

"""
UPDATED DESCRIPTION: 
Publish the pose of a ball found through colour detection

ORIGINAL DESCRIPTION: 
Publish human pose TFs by converting PoseStamped messages.
You can run this node with rqt_ez_publisher to get a quick-and-dirty GUI to
add and move a virtual human pose.
You can pass in starting positions (length-three tuples) if desired:
    human_pose_publisher.py --initial_hand_pose 0 1 1
                --initial_shoulder_pose 1 0 2
Author: Felix Duvallet <felix.duvallet@epfl.ch>
"""

import yaml
import rospy
import tf
from geometry_msgs.msg import Pose, TransformStamped


class PoseToTFRebroadcaster(object):
    """
    Simple node that listens to PoseStamped messages and rebroadcasts
    corresponding TF frames.
    """

    def __init__(self, config=None):
        if config is None:
            config = {}

        self._publish_rate = config.get('publish_rate', 1)
        assert self._publish_rate > 0, 'Must have positive publishing rate.'

        self._frame_configs = config.get('frames', {})
        self._default_parent_frame = 'world'

        # One TF broadcaster
        self._tf_broadcaster = tf.TransformBroadcaster()

        # All the current pose data. Map from frame_name -> PoseStamped.
        self._pose_data = self._init_subscribers()

    def _init_subscribers(self):
        """
        Starts one subscriber per pose topic, and stores the frame name & parent
        associated with each.
        :return: dict(frame_name -> dict('pose'->pose, 'parent_frame'->frame))
        """
        pose_data = {}

        for (frame_name, frame_info) in self._frame_configs.items():
            topic = frame_info.get('pose_topic')
            parent_frame = frame_info.get(
                'parent_frame', self._default_parent_frame)

            print(topic, " ", parent_frame)
            # Parse the initial pose, if given in the config.
            frame_position = frame_info.get('initial_position', None)
            initial_pose = None
            if frame_position:
                frame_position = map(float, frame_position.split())
                initial_pose = self.make_stamped_pose(frame_position)

            # Create a new subscriber callback that receives the frame name when
            # it gets called. That subscriber will store the latest pose.
            _ = rospy.Subscriber(
                topic, Pose, self._pose_callback, frame_name)
            pose_data[frame_name] = {'parent_frame': parent_frame,
                                     'pose': initial_pose}
            rospy.logdebug(('Created TF rebroadcaster for pose topic [{}] '
                            '-> frame [{}] with parent').format(
                topic, frame_name, parent_frame))

        return pose_data

    def _pose_callback(self, data, frame_name):
        # Store the latest PoseStamped message.
        # print(frame_name)
        self._pose_data[frame_name]['pose'] = data

    @classmethod
    def pose_to_tf(cls, pose, frame_name, parent_frame, time=None):
        """
        Generate a TF from a given pose, frame, and parent.
        """
        assert pose is not None, 'Cannot have None for pose.'
        tf1 = TransformStamped()
        tf1.child_frame_id = frame_name
        if time is None:
            time = rospy.Time.now()
        tf1.header.stamp = time
        tf1.header.frame_id = parent_frame

        tf1.transform.translation = pose.position
        tf1.transform.rotation = pose.orientation

        return tf1

    def publish_transforms(self):
        """
        Publish all currently-known poses as transforms.
        """

        for (frame_name, pose_data) in self._pose_data.items():
            # print("frame_name: ", frame_name, " pose: ", pose_data)
            if not pose_data['pose']:
                # print("no pose data for ", frame_name)
                continue
            transform = self.pose_to_tf(
                pose=pose_data['pose'], frame_name=frame_name, parent_frame=pose_data['parent_frame'])
            self._tf_broadcaster.sendTransform(transform.transform.translation, transform.transform.rotation, 
                transform.header.stamp, transform.child_frame_id, transform.header.frame_id)
            rospy.logdebug('Published transform for frame {}.'.format(
                frame_name))

    @classmethod
    def make_stamped_pose(cls, position):
        """
        Parse a list of positions (that could be None or empty) into a
        PoseStamped ROS message.
        :return A PoseStamped with the identity orientation, or None if the
        input is None or if the list length is not 3.
        """
        initial_pose = None
        if position and len(position) == 3:
            initial_pose = Pose()
            initial_pose.position.x = position[0] 
            initial_pose.position.y = position[1]
            initial_pose.position.z = position[2]
            initial_pose.orientation.w = 1.0  # init quaternion properly.

        return initial_pose

    def spin(self):
        """
        Publish transforms at the requested rate.
        """
        r = rospy.Rate(self._publish_rate)
        while not rospy.is_shutdown():
            self.publish_transforms()
            r.sleep()


def load_config(config_file):
    with open(config_file, 'r') as f:
        config = yaml.load(f)

    return config


def run(argv):
    rospy.init_node('pose_to_tf_rebroadcaster', anonymous=True)

    config = load_config(argv)
    print('Have configuration: {}'.format(config))

    transform_rebroadcaster = PoseToTFRebroadcaster(config)
    rospy.loginfo('Pose to TF rebroadcaster is now running...')

    transform_rebroadcaster.spin()
    rospy.loginfo('Pose to TF rebroadcaster has finished.')

if __name__ == '__main__':
    arguments = rospy.get_param('config_file')
    # arguments = sys.argv[1:]  # argv[0] is the program name.
    run(arguments)
