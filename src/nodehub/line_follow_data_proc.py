import json
import os
import shutil
import sys
import argparse

source_folder_path = "line_follow_image"
target_folder_path = "line_follow_dataset"

parser = argparse.ArgumentParser()
parser.add_argument("--source_folder_path", type=str, help="")
parser.add_argument("--target_folder_path", type=str, help="")
args = parser.parse_args()
if args.source_folder_path:
	source_folder_path = args.source_folder_path
if args.target_folder_path:
	target_folder_path = args.target_folder_path

if not os.path.exists(target_folder_path):
    os.makedirs(target_folder_path)
# 读取JSON文件
for filename in os.listdir(source_folder_path):
	if filename.endswith(".json"):
		json_path = os.path.join(source_folder_path, filename)
		base_name = os.path.splitext(filename)[0]
		jpg_path = os.path.join(source_folder_path, f"{base_name}.jpg")
		if os.path.exists(jpg_path):
			with open(json_path, "r") as json_file:
				data = json.load(json_file)

			# 获取所有shapes
				shapes = data["shapes"]
				num = 0
				# 遍历每个shape
				for shape in shapes:
					shape_type = shape["shape_type"]
					if shape_type == "point":
						num = num + 1
						if num == 2:
							print("Two points appear")
							print("Please check the file " + json_path)
							sys.exit()
						points = shape["points"]
						for point in points:
							x, y = point
							new_filename = f"xy_{(int(x)):03d}_{(int(y)):03d}_{base_name}.jpg"
							destination_file_path = os.path.join(target_folder_path, new_filename)
							shutil.copy(jpg_path, destination_file_path)
							print(f"Copied: {jpg_path} --> {destination_file_path}")







