import json
import csv
import os
import sys
import random
import shutil
import argparse
# 读取JSON文件

source_folder_path = "detection_image"
target_folder_path = "detection_dataset"

train_radio = 7
val_radio = 2
test_radio = 1

parser = argparse.ArgumentParser()
parser.add_argument("--source_folder_path", type=str, help="")
parser.add_argument("--target_folder_path", type=str, help="")
args = parser.parse_args()
if args.source_folder_path:
	source_folder_path = args.source_folder_path
if args.target_folder_path:
	target_folder_path = args.target_folder_path

train_destination_folder = os.path.join(target_folder_path, "train")
validation_destination_folder = os.path.join(target_folder_path, "validation")
test_destination_folder = os.path.join(target_folder_path, "test")

if train_radio + val_radio + test_radio != 10:
	print (train_radio + val_radio + test_radio )
	print("Please ensure that the data ratio is correct")
	sys.exit()

if not os.path.exists(target_folder_path):
    os.makedirs(target_folder_path)
if not os.path.exists(train_destination_folder):
    os.makedirs(train_destination_folder)
if not os.path.exists(validation_destination_folder):
    os.makedirs(validation_destination_folder)
if not os.path.exists(test_destination_folder):
    os.makedirs(test_destination_folder)

train_file = open(os.path.join(target_folder_path, 'sub-train-annotations-bbox.csv'), 'w')
val_file = open(os.path.join(target_folder_path, 'sub-val-annotations-bbox.csv'), 'w')
test_file = open(os.path.join(target_folder_path, 'sub-test-annotations-bbox.csv'), 'w')

train_writer = csv.writer(train_file)
train_writer.writerow(['ImageID', 'XMin', 'XMax', 'YMin', 'YMax', 'ClassName'])
val_writer = csv.writer(val_file)
val_writer.writerow(['ImageID', 'XMin', 'XMax', 'YMin', 'YMax', 'ClassName'])
test_writer = csv.writer(test_file)
test_writer.writerow(['ImageID', 'XMin', 'XMax', 'YMin', 'YMax', 'ClassName'])
for filename in os.listdir(source_folder_path):
	if filename.endswith(".json"):
		json_path = os.path.join(source_folder_path, filename)
		base_name = os.path.splitext(filename)[0]
		jpg_path = os.path.join(source_folder_path, f"{base_name}.jpg")
		if os.path.exists(jpg_path):
			contain_rectangle = False
			rand = random.random()* 10
			with open(json_path, 'r') as json_file:
				data = json.load(json_file)
				height = data["imageHeight"]
				width = data["imageWidth"]
				shapes = data["shapes"]
				for shape in shapes:
					shape_type = shape["shape_type"]
					if shape_type == "rectangle":
						contain_rectangle = True
						points = shape["points"]
						label = shape["label"]
						num_list = []
						for point in points:
							x, y = point
							x_nor = "{:.6f}".format(x / width)
							y_nor = "{:.6f}".format(y / height)
							num_list.append(x_nor)
							num_list.append(y_nor)
							print(f"Point: ({x_nor}, {y_nor})")
						xmin, xmax = (num_list[0], num_list[2]) if num_list[0] < num_list[2] else (num_list[2], num_list[0])
						ymin, ymax = (num_list[1], num_list[3]) if num_list[1] < num_list[3] else (num_list[3], num_list[1])
						print(f"Point1111: ({base_name},{xmin}, {xmax},{ymin},{ymax}),{label}")
						
						print(rand)
						if rand < train_radio:
							train_writer.writerow([base_name, xmin, xmax, ymin, ymax, label])
						elif rand < train_radio + val_radio:
							val_writer.writerow([base_name, xmin, xmax, ymin, ymax, label])
						else:
							test_writer.writerow([base_name, xmin, xmax, ymin, ymax, label])
			if contain_rectangle == True:
				if rand < train_radio:
					shutil.copy(jpg_path, train_destination_folder)
				elif rand < train_radio + val_radio:
					shutil.copy(jpg_path, validation_destination_folder)
				else:
					shutil.copy(jpg_path, test_destination_folder)