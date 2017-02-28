# RLRS

driver

particleFilter
	input: map

ultraScan
	makes a sensor scan
	output: scan vector

turn
	makes the robot turn a specified angle
	input: angle

move
	makes the robot move a specified length
	input: length

pathPlanning
	input: position, angle, target, map
	output: path array

pathMove
	input: currentPosition, nextPosition
