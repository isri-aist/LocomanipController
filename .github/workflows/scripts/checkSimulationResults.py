#! /usr/bin/env python

import sys
import argparse
import numpy as np
import eigen as e
import mc_log_ui

exit_status = 0

# Parse arguments
parser = argparse.ArgumentParser(description="Check simulation results.")
parser.add_argument("log-filename", type=str, help="log filename")
parser.add_argument("--tilting-angle-thre", type=float, default=30.0,
                    help="tilting angle threshold [deg]")
parser.add_argument("--expected-obj-pos", nargs=3, type=float, default=None,
                    help="expected object position [m]")
parser.add_argument("--obj-pos-thre", nargs=3, type=float, default=[0.25, 0.25, 0.25],
                    help="object position threshold [m]")
args = parser.parse_args()

# Load log file
args.log_filename = sys.argv[1]
print("[checkSimulationResults.py] Load {}".format(args.log_filename))
log = mc_log_ui.read_log(args.log_filename)

# Check tilting angle
robot_quat_data_list = np.array([log["FloatingBase_orientation_w"],
                           log["FloatingBase_orientation_x"],
                           log["FloatingBase_orientation_y"],
                           log["FloatingBase_orientation_z"]]).T
obj_quat_data_list = np.array([log["ManipManager_objPose_measured_qw"],
                               log["ManipManager_objPose_measured_qx"],
                               log["ManipManager_objPose_measured_qy"],
                               log["ManipManager_objPose_measured_qz"]]).T

tilting_angle_list = []
for quat_data in robot_quat_data_list:
    # Inverse is required because the left-hand system is used in mc_rtc
    quat = e.Quaterniond(*quat_data).inverse()
    rot = quat.toRotationMatrix()
    z_axis = e.Vector3d(rot.block(0, 2, 3, 1))
    tilting_angle_list.append(np.rad2deg(np.arccos(z_axis.dot(e.Vector3d.UnitZ())))) # [deg]
for quat_data in obj_quat_data_list:
    # Inverse is required because the left-hand system is used in mc_rtc
    quat = e.Quaterniond(*quat_data).inverse()
    rot = quat.toRotationMatrix()
    z_axis = e.Vector3d(rot.block(0, 2, 3, 1))
    tilting_angle_list.append(np.rad2deg(np.arccos(z_axis.dot(e.Vector3d.UnitZ())))) # [deg]
tilting_angle_list = np.array(tilting_angle_list)
max_tilting_angle = np.max(np.abs(tilting_angle_list))

if max_tilting_angle <= args.tilting_angle_thre:
    print(u"[success][checkSimulationResults.py] max_tilting_angle is below the threshold: {:.1f} <= \u00b1 {:.1f} [deg]".format(
        max_tilting_angle, args.tilting_angle_thre))
else:
    print(u"[error][checkSimulationResults.py] max_tilting_angle exceeds the threshold: {:.1f} > \u00b1 {:.1f} [deg]".format(
        max_tilting_angle, args.tilting_angle_thre))
    exit_status = 1

# Check last position
last_obj_pos = np.array([log["ManipManager_objPose_measured_tx"][-1],
                         log["ManipManager_objPose_measured_ty"][-1],
                         log["ManipManager_objPose_measured_tz"][-1]])

if args.expected_obj_pos is not None:
    if (np.abs(last_obj_pos - np.array(args.expected_obj_pos)) < np.array(args.obj_pos_thre)).all():
        with np.printoptions(precision=2):
            print(u"[success][checkSimulationResults.py] last_obj_pos is within the expected range: {} <= {} \u00b1 {} [m]".format(
                last_obj_pos, np.array(args.expected_obj_pos), np.array(args.obj_pos_thre)))
    else:
        with np.printoptions(precision=2):
            print(u"[error][checkSimulationResults.py] last_obj_pos is outside the expected range: {} > {} \u00b1 {} [m]".format(
                last_obj_pos, np.array(args.expected_obj_pos), np.array(args.obj_pos_thre)))
        exit_status = 1

sys.exit(exit_status)
