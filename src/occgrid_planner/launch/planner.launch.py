# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='joy', executable='joy_node', name='joy',
            parameters=[
                {'autorepeat_rate': 10.},
                {'dev': "/dev/input/js0"},
                ],
            output='screen'),

        launch_ros.actions.Node(
            package='topic_tools', executable='mux', name='cmd_mux',
            parameters=[
                {'output_topic': '/vrep/twistCommand'},
                {'input_topics': ['/teleop/twistCommand','/mux/autoCommand']},
                ],
            output='screen'),

        launch_ros.actions.Node(
            package='vrep_ros_teleop', executable='teleop_node', name='teleop',
            parameters=[
                {'~/axis_linear_x': 1},
                {'~/axis_angular': 2},
                {'~/scale_linear_x': 1.},
                {'~/scale_angular': 1.},
                {'~/timeout': 1.0}
                ],
            remappings=[
                ('twistCommand', '/teleop/twistCommand'),
                ],
            output='screen'),

        launch_ros.actions.Node(
            package='vrep_ros_teleop', executable='teleop_mux_node', name='teleop_mux',
            parameters=[
                {'~/joystick_button': 0},
                {'~/joystick_topic': '/teleop/twistCommand'},
                {'~/auto_button': 1},
                {'~/auto_topic': '/mux/autoCommand'}
                ],
            remappings=[
                ('select', '/cmd_mux/select'),
                ],
            output='screen'),

        launch_ros.actions.Node(
            package='occgrid_planner', executable='occgrid_planner', name='occgrid_planner',
            parameters=[
                {'~/neighbourhood': 8},
                {'~/base_frame': 'bubbleRob'},
                {'~/debug': False},
                ],
            remappings=[
                ('~/occ_grid', '/map'),
                ('~/goal', '/goal_pose'),
                ],
            output='screen'),

        launch_ros.actions.Node(
            package='occgrid_planner', executable='path_optimizer', name='path_optimizer',
            parameters=[
                {'~/max_acceleration': 0.2},
                {'~/max_braking': 0.05},
                {'~/velocity': 0.5
                 
                 },
                ],
            remappings=[
                ('~/path', '/occgrid_planner/path'),
                ],
            output='screen'),

        launch_ros.actions.Node(
            package='occgrid_planner', executable='path_follower', name='path_follower',
            parameters=[
                {'~/Kx': 0.8},
                {'~/Ky': 2},
                {'~/Ktheta': 1.6},
                {'~/max_rot_speed': 1.5},
                {'~/max_velocity': 1.0},
                {'~/max_y_error': 0.3},
                {'~/max_error': 0.5},
                {'~/look_ahead': 1.0},
                {'~/base_frame': 'bubbleRob'},
                ],
            remappings=[
                ('~/traj', '/path_optimizer/trajectory'),
                ('~/twistCommand', '/mux/autoCommand'),
                ('~/goal', '/goal_pose'),
                ],
            output='screen'),


    ])
