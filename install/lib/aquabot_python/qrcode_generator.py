#!/usr/bin/env python3

# This script has to be started from the scripts folder (Aquabot/aquabot_python/scripts)
# It also can be started with the following command: python3 qrcode_generator.py

import os
import qrcode
import shutil

def generate_qrcode_text(text, filepath):
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_M,
        box_size=10,
        border=4,
    )
    qr.add_data(text)
    qr.make(fit=True)
    img = qr.make_image(fill_color="black", back_color="white")
    img.save(filepath)

def duplicate_marker_model(destination_name):
    base_path = "../../aquabot_gz/models/"
    origin_name = "aquabot_marker_0_KO"
    if origin_name != destination_name:
        if os.path.exists(base_path + destination_name):
            shutil.rmtree(base_path + destination_name)
        shutil.copytree(base_path + origin_name, base_path + destination_name)
        replace_string_in_file(base_path + destination_name + "/model.sdf", origin_name, destination_name)
        replace_string_in_file(base_path + destination_name + "/model.config", origin_name, destination_name)
        replace_string_in_file(base_path + destination_name + "/meshes/Marker0.dae", origin_name + ".png", destination_name + ".png")
    images_path = "../images/"
    os.remove(base_path + destination_name + "/materials/textures/" + origin_name + ".png")
    shutil.move(images_path + destination_name + ".png", base_path + destination_name + "/materials/textures/" + destination_name + ".png")

def replace_string_in_file(file_path, origin_string, new_string):
    filedata = None
    with open(file_path, 'r') as file:
        filedata = file.read()
    filedata = filedata.replace(origin_string, new_string)
    with open(file_path, 'w') as file:
        file.write(filedata)

def main():
    path = "../images/"
    for i in range(10) :
        name = "aquabot_marker_" + str(i) + "_KO"
        generate_qrcode_text("{\"id\":" + str(i) + ",\"state\":\"KO\"}", path + name + ".png")
        duplicate_marker_model(name)
        name = "aquabot_marker_" + str(i) + "_OK"
        generate_qrcode_text("{\"id\":" + str(i) + ",\"state\":\"OK\"}", path + name + ".png")
        duplicate_marker_model(name)

if __name__ == "__main__":
    main()