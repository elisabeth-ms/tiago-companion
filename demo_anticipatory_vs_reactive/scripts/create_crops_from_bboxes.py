#!/usr/bin/env python2

import os
import cv2
import pandas as pd

# Base directory
base_dir = "objects_partially_occluded"

# Directories for CSVs, color images, and cropped images
csv_dir = os.path.join(base_dir, "bboxes")
color_dir = os.path.join(base_dir, "color")
crop_dir = os.path.join(base_dir, "color_crops")

# Ensure the crop directory exists
if not os.path.exists(crop_dir):
    os.makedirs(crop_dir)

# List all CSV files in the csv_dir
csv_files = [f for f in os.listdir(csv_dir) if f.endswith('.csv')]

for csv_file in csv_files:
    # Extract case number from the CSV filename
    case_num = int(csv_file.split('_')[1].split('.')[0])

    # Load the CSV data
    df = pd.read_csv(os.path.join(csv_dir, csv_file))

    # Load the corresponding color image
    img_path = os.path.join(color_dir, "color_case_{}.png".format(case_num))
    img = cv2.imread(img_path)

    # For each bounding box in the CSV, crop the image and save
    for index, row in df.iterrows():
        # Extract bounding box corners
        tlx, tly, brx, bry = int(row['Top-left x']), int(row['Top-left y']), int(row['Bottom-right x']), int(row['Bottom-right y'])
        print(tlx, tly, brx, bry)
        # Crop the image
        cropped_img = img[tly:bry, tlx:brx]

        # Save the cropped image
        crop_img_path = os.path.join(crop_dir, "crop_case_{}_id_{}.png".format(case_num, int(row['ID'])))
        cv2.imwrite(crop_img_path, cropped_img)

print("Cropping completed!")
