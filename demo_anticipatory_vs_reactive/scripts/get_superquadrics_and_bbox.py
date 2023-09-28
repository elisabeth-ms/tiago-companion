#!/usr/bin/env python

import rospy
from companion_msgs.srv import GetBboxes  # Replace 'your_package_name' with the name of your package
from companion_msgs.msg import BoundingBoxes, BoundingBox  # Replace 'your_package_name' with the name of your package
from companion_msgs.srv import GetSuperquadrics  # Import the service type
from companion_msgs.msg import SuperquadricMultiArray, Superquadric  # Import the message types

import csv
import os
import sys
import time
def get_bounding_boxes(case_num):
    # Wait for the service to become available
    rospy.wait_for_service('/grasp_objects/get_bboxes_superquadrics')

    try:
        # Create a handle to the service
        get_boxes_service = rospy.ServiceProxy('/grasp_objects/get_bboxes_superquadrics', GetBboxes)

        # Call the service (no request arguments are needed since it's an empty request)
        response = get_boxes_service()

        save_bounding_boxes_to_csv(response.bounding_boxes, case_num)
        # Return the bounding boxes from the response
        return response.bounding_boxes

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def save_bounding_boxes_to_csv(bounding_boxes, case_num):
    # Define the directory and file path
    directory = "objects_partially_occluded/bboxes"
    if not os.path.exists(directory):
        os.makedirs(directory)
    file_path = os.path.join(directory, "case_"+str(case_num)+".csv")

    # Write to the CSV file
    with open(file_path, 'w') as csvfile:
        fieldnames = ['ID', 'Top-left x', 'Top-left y', 'Bottom-right x', 'Bottom-right y']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        writer.writeheader()
        for box in bounding_boxes.bounding_boxes:
            writer.writerow({'ID': box.id, 'Top-left x': box.tlx, 'Top-left y': box.tly, 'Bottom-right x': box.brx, 'Bottom-right y': box.bry})

def get_superquadrics(case_num):
    rospy.wait_for_service('/grasp_objects/get_superquadrics')
    try:
        get_superquadrics_service = rospy.ServiceProxy('/grasp_objects/get_superquadrics', GetSuperquadrics)
        response = get_superquadrics_service()
        save_superquadrics_to_csv(response.superquadrics, case_num)
        return response.superquadrics
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def save_superquadrics_to_csv(superquadrics, case_num):
    directory = "objects_partially_occluded/superquadrics"
    if not os.path.exists(directory):
        os.makedirs(directory)
    file_path = os.path.join(directory, "case_"+str(case_num)+".csv")

    with open(file_path, 'w') as csvfile:
        fieldnames = ['ID', 'a1', 'a2', 'a3', 'e1', 'e2', 'x', 'y', 'z', 'roll', 'pitch', 'yaw']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        for sq in superquadrics.superquadrics:
            writer.writerow({
                'ID': sq.id, 'a1': sq.a1, 'a2': sq.a2, 'a3': sq.a3, 'e1': sq.e1, 'e2': sq.e2,
                'x': sq.x, 'y': sq.y, 'z': sq.z, 'roll': sq.roll, 'pitch': sq.pitch, 'yaw': sq.yaw
            })

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('bounding_boxes_client')
    time.sleep(1)
    try:
        print("Waiting for input...")
        sys.stdout.flush()
        case_num = raw_input("Enter the case number:\n")
        print(case_num)
    except EOFError:
        print("Input error. Exiting.")
        exit()
    # Get the bounding boxes
    boxes = get_bounding_boxes(case_num)
    # Get the superquadrics
    superquadrics = get_superquadrics(case_num)

