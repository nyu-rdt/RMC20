from __future__ import division
from __future__ import print_function
from argparse import ArgumentParser
import cv2
import apriltag
import numpy
import math


# Draws an overlay of a cube over the apriltag
def draw_pose(overlay, camera_params, tag_size, pose, z_sign=1):
    opoints = numpy.array([
        -1, -1, 0,
        1, -1, 0,
        1, 1, 0,
        -1, 1, 0,
        -1, -1, -2 * z_sign,
        1, -1, -2 * z_sign,
        1, 1, -2 * z_sign,
        -1, 1, -2 * z_sign,
    ]).reshape(-1, 1, 3) * 0.5 * tag_size

    edges = numpy.array([
        0, 1,
        1, 2,
        2, 3,
        3, 0,
        0, 4,
        1, 5,
        2, 6,
        3, 7,
        4, 5,
        5, 6,
        6, 7,
        7, 4
    ]).reshape(-1, 2)

    fx, fy, cx, cy = camera_params

    K = numpy.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)

    rvec, _ = cv2.Rodrigues(pose[:3, :3])
    tvec = pose[:3, 3]

    dcoeffs = numpy.zeros(5)

    ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, dcoeffs)

    ipoints = numpy.round(ipoints).astype(int)

    ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]

    for i, j in edges:
        cv2.line(overlay, ipoints[i], ipoints[j], (0, 255, 0), 1, 16)


'''
Converts rotation matrix into radians and degrees
id is id of apriltag
If R is rotation matrix:
R = Rx times Ry times Rz
ex. Rx = [1, 0, 0]
         [0, cos(xangle), sin(xangle)]
        [0, sin(xangle), cos(xangle)]
Function taken from: https://www.learnopencv.com/rotation-matrix-to-euler-angles/
'''


def rotation_matrix_to_euler_angles(R, id):
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    if id == 0:
        return numpy.array([math.degrees(x), math.degrees(y) + 180, math.degrees(z)])
    elif id == 1 or id == 2:
        return numpy.array([math.degrees(x), math.degrees(y) + 90, math.degrees(z)])
    elif id == 3 or id == 4:
        return numpy.array([math.degrees(x), math.degrees(y), math.degrees(z)])
    elif id == 5 or id == 6:
        return numpy.array([math.degrees(x), math.degrees(y) + 270, math.degrees(z)])
    elif id == -1:
        return y


# Angular rotation of each apriltag
def find_angular_rotation(position):
    angle = math.atan(position[0] / position[2])
    return math.degrees(angle)


# From 0 - 180 degrees
def find_absolute_angular_rotation(position):
    relangle = math.degrees(math.atan(position[0] / position[2]))
    if relangle == 0:
        absangle = 90

    elif relangle < 0:
        absangle = 90 - math.fabs(relangle)

    elif relangle > 0:
        absangle = 90 + math.fabs(relangle)
    return absangle


# Average angle of 2 border apriltags
def adjusted_angular_rotation(storedangles):
    length = len(storedangles)

    if length > 2:
        ang1 = max(storedangles)
        ang2 = min(storedangles)
    else:
        ang1 = storedangles[0]
        ang2 = storedangles[1]

    adjusted_rotation_angle = math.fabs(ang1 - ang2) / 2

    if (ang1 + ang2) / 2 < 90:
        return -1 * adjusted_rotation_angle
    else:
        return adjusted_rotation_angle


# Center enter position of robot
def center_position(position, rotation_matrix, id):
    relative_orientation = rotation_matrix_to_euler_angles(rotation_matrix, -1)

    if id == 0 or id == 3:

        diag_length = 0.50
        relative_orientation = math.fabs(relative_orientation)
        x_add = diag_length * math.sin(relative_orientation)
        y_add = diag_length * math.cos(relative_orientation)
        if relative_orientation < 0:
            x_add *= -1

    elif id == 2 or id == 6 or id == 5 or id == 1:
        diag_length = 0.35

        relative_orientation = math.fabs(relative_orientation)
        if id == 2 or id == 6:
            needed_angle = 0.785398
            if relative_orientation > math.radians(45):
                new_angle = relative_orientation - needed_angle
                x_add = -1 * diag_length * math.cos(new_angle)
                y_add = diag_length * math.sin(new_angle)
            elif math.radians(-45) < relative_orientation < math.radians(45):
                new_angle = needed_angle - relative_orientation
                x_add = diag_length * math.cos(new_angle)
                y_add = diag_length * math.sin(new_angle)
            elif relative_orientation < math.radians(-45):
                new_angle = relative_orientation - needed_angle
                x_add = diag_length * math.cos(new_angle)
                y_add = -1 * diag_length * math.sin(new_angle)

        elif id == 5 or id == 1:
            needed_angle = 0.785398
            if relative_orientation > math.radians(45):
                new_angle = relative_orientation - needed_angle
                x_add = -1 * diag_length * math.cos(new_angle)
                y_add = 1 * diag_length * math.sin(new_angle)
            elif math.radians(-45) < relative_orientation < math.radians(45):
                new_angle = needed_angle - relative_orientation
                x_add = diag_length * math.cos(new_angle)
                y_add = diag_length * math.sin(new_angle)
            elif relative_orientation < math.radians(-45):
                new_angle = relative_orientation - needed_angle
                x_add = diag_length * math.cos(new_angle)
                y_add = -1 * diag_length * math.sin(new_angle)

    centered_x = position[0] + x_add
    centered_y = position[2] + y_add

    return numpy.array([centered_x, centered_y])


# Returns average of list
def avgposition(length):
    total = 0
    count = 0
    print(length)
    for measurement in length:
        total += measurement
        count += 1
    average = total / count
    return average


def debug_log(detection, pose, init_error, final_error, rotation_matrix, position, num_detections, i):
    # Converts rotation matrix of individual apriltag into degrees and radians
    print('Rotation in Degrees (pitch,yaw,roll)', rotation_matrix_to_euler_angles(rotation_matrix, detection.tag_id))
    print()
    print('Position:\n', position)
    print('Angular Rotation of individual tag:', find_angular_rotation(position))
    print('Rotation matrix\n', rotation_matrix)
    print('Center Position: \n', center_position(position, rotation_matrix, detection.tag_id))
    print('Detection {} of {}:'.format(i + 1, num_detections))
    print()
    print(detection.tostring(indent=2))
    print()
    print('pose:', pose)
    print('init error, final error:', init_error, final_error)
    print()


def main():
    # Values come from running camera calibration file (fx, fy, cx,cy).
    # Camera should be calibrated for each new computer.
    # camera_params = [1.01446618 * 10 ** 3, 1.02086461 * 10 ** 3, 6.09583146 * 10 ** 2, 3.66171174 * 10 ** 2]
    camera_params = [1.31239907 * 10 ** 3, 1.31169637 * 10 ** 3, 9.23293617 * 10 ** 2, 5.49707267 * 10 ** 2]

    # Set value for camera
    cap = cv2.VideoCapture(0)

    window = 'Camera'
    cv2.namedWindow(window)

    detector = apriltag.Detector()

    while True:
        # Stores values of each detected tag for final output
        stored_angles = []
        stored_x_values = []
        stored_y_values = []
        stored_orientation = []

        success, frame = cap.read()
        if not success:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        detections, dimg = detector.detect(gray, return_image=True)
        print()
        num_detections = len(detections)
        print('Detected {} tags.\n'.format(num_detections))

        tag_count = -1
        for i, detection in enumerate(detections):

            # Returns pose matrix
            pose, init_error, final_error = detector.detection_pose(detection, camera_params, tag_size=0.17, z_sign=1)

            rotation_matrix = numpy.array([pose[0][:3],
                                           pose[1][:3],
                                           pose[2][:3]])

            position = numpy.array([pose[0][3:], pose[1][3:], pose[2][3:]])

            # debug_log(detection, pose, init_error, final_error, rotation_matrix, position, num_detections, i)

            if num_detections > 1:
                stored_angles.append(find_absolute_angular_rotation(position))
                # print('A', centerPosition(position, rotation_matrix, detection.tag_id)[0])
                stored_x_values.append(center_position(position, rotation_matrix, detection.tag_id)[0])
                # print('B', centerPosition(position, rotation_matrix, detection.tag_id)[1])
                stored_y_values.append(center_position(position, rotation_matrix, detection.tag_id)[1])
                stored_orientation.append(rotation_matrix_to_euler_angles(rotation_matrix, detection.tag_id))

                tag_count += 1

            elif num_detections == 1:
                stored_angles.append(find_angular_rotation(position))
                stored_x_values.append(center_position(position, rotation_matrix, detection.tag_id)[0])
                stored_y_values.append(center_position(position, rotation_matrix, detection.tag_id)[1])
                stored_orientation.append(rotation_matrix_to_euler_angles(rotation_matrix, detection.tag_id))

            overlay = frame // 2 + dimg[:, :, None] // 2

            # Draws all overlays before going to next frame

            for det in range(num_detections):
                draw_pose(overlay, camera_params, 0.17, pose, z_sign=1)

            cv2.imshow(window, overlay)
            cv2.waitKey(27)

        # print('stored_angles', stored_angles)

        if num_detections > 1:
            print('Adjusted Rotation Angle: ', adjusted_angular_rotation(stored_angles))
        elif num_detections == 1:
            print('Angular Rotation: ', stored_angles[tag_count])
        if num_detections > 0:
            print('Center Position', stored_x_values[tag_count], stored_y_values[tag_count])
            print('Orientation', stored_orientation[tag_count])
            # print('X:', avgposition(stored_x_values), '\nY: ', avgposition(stored_y_values))


if __name__ == '__main__':
    main()
