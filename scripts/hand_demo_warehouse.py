#!/usr/bin/env python

import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander

rospy.init_node("hand_demo_warehouse")

# comment by v4hn, MoveIt maintainer 20180426
# This is the "high-level" way to run the hand through a series of named poses
# The API supports run_named_trajectory and run_named_trajectory_safe
# Both do simple joint-space interpolation and neither
# plans to avoid collisions (especially with the thumb).
# So neither is "safe" and they can lead to unwanted collisions,
# e.g. going from chinese_number_1 to chinese_number_2!!!

# At the moment, the only difference is that MoveIt will abort
# a trajectory if it takes longer than expected.
# Hopefully we will add collision checks here in the future,
# but that still would not produce a working trajectory with this function...

rospy.logerr("DON'T USE THIS - READ THE COMMENT AT THE TOP OF THIS SOURCE FILE.")

robot= SrHandCommander()

robot.run_named_trajectory(
[
{
	'name': 'chinese_number_0',
	'interpolate_time': 4.0,
        'pause_time' : 2
},
{
	'name': 'chinese_number_1',
	'interpolate_time': 4.0,
        'pause_time' : 2
},
#{
#	'name': 'chinese_number_2',
#	'interpolate_time': 4.0,
#        'pause_time' : 2
#},
#{
#	'name': 'chinese_number_3',
#	'interpolate_time': 4.0,
#        'pause_time' : 2
#},
#{
#	'name': 'chinese_number_4',
#	'interpolate_time': 4.0,
#        'pause_time' : 2
#},
{
	'name': 'chinese_number_5',
	'interpolate_time': 4.0,
        'pause_time' : 2
},
{
	'name': 'chinese_number_6',
	'interpolate_time': 4.0,
        'pause_time' : 2
},
{
	'name': 'chinese_number_7',
	'interpolate_time': 4.0,
        'pause_time' : 2
},
{
	'name': 'chinese_number_8',
	'interpolate_time': 4.0,
        'pause_time' : 2
},
{
	'name': 'chinese_number_9',
	'interpolate_time': 4.0,
        'pause_time' : 2
},
]
)
