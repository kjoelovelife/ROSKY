#!/usr/bin/env bash

path_kinematics="ROSKY/catkin_ws/src/rosky_base/config/baseline/calibration/kinematics"
path_keyboard_mapper_node="ROSKY/catkin_ws/src/rosky_base/config/baseline/keyboard_mapper/keyboard_mapper_node"
path_camera_intrinsic="ROSKY/catkin_ws/src/rosky_base/config/baseline//calibration/camera_intrinsic"
path_camera_extrinsic="ROSKY/catkin_ws/src/rosky_base/config/baseline//calibration/camera_extrinsic"
path_camera_info="ROSKY/catkin_ws/src/jetson_camera/camera_info/cam_info_reader_node"
path_decoder_node="ROSKY/catkin_ws/src/jetson_camera/camera_info/decoder_node"
default="default.yaml"

echo "Setting up parameter files with project ROSKY... "

if [ $# -gt 0 ]; then
        file_name=$1
	echo "You provide a hostname. The files name are: $file_name.yaml"
else
        file_name=$HOSTNAME
	echo "No hostname provided. Files name Use $HOSTNAME."

fi
echo ""
echo "Setup parameter kinematics"
if test -e ~/$path_kinematics/$file_name".yaml"; then
    echo -n "You already have the parameter about kinematics, do you want to reset it? (y/N): "
    read reset
    if [[ "$reset" == "y" ]] || [[ "$reset" == "Y" ]]; then
        cp ~/$path_kinematics/$default ~/$path_kinematics/$file_name".yaml"
        echo "Reset param kinematics."
    else
        echo "Skip set param kinematics."
    fi
else
    cp ~/$path_kinematics/$default ~/$path_kinematics/$file_name".yaml"
    echo "Add param kinematics with name \"$file_name\""
fi

echo ""
echo "Setup parameter keyboard_mapper_node"
if test -e ~/$path_keyboard_mapper_node/$file_name".yaml"; then
    echo -n "You already have the parameter about keyboard_mapper_node, do you want to reset it? (y/N): "
    read reset
    if [[ "$reset" == "y" ]] || [[ "$reset" == "Y" ]]; then
        cp ~/$path_keyboard_mapper_node/$default ~/$path_keyboard_mapper_node/$file_name".yaml"
        echo "Reset param keyboard_mapper_node."
    else
        echo "Skip set param keyboard_mapper_node."
    fi
else
    cp ~/$path_keyboard_mapper_node/$default ~/$path_keyboard_mapper_node/$file_name".yaml"
    echo "Add param keyboard_mapper_node with name \"$file_name\""
fi

echo ""
echo "Setup parameter camera_intrinsic"
if test -e ~/$path_camera_intrinsic/$file_name".yaml"; then
    echo -n "You already have the parameter about camera_intrinsic, do you want to reset it? (y/N): "
    read reset
    if [[ "$reset" == "y" ]] || [[ "$reset" == "Y" ]]; then
        cp ~/$path_camera_intrinsic/$default ~/$path_camera_intrinsic/$file_name".yaml"
        echo "Reset param camera_intrinsic."
    else
        echo "Skip set param camera_intrinsic."
    fi
else
    cp ~/$path_camera_intrinsic/$default ~/$path_camera_intrinsic/$file_name".yaml"
    echo "Add param camera_intrinsic with name \"$file_name\""
fi

echo ""
echo "Setup parameter camera_extrinsic"
if test -e ~/$path_camera_extrinsic/$file_name".yaml"; then
    echo -n "You already have the parameter about camera_extrinsic, do you want to reset it? (y/N): "
    read reset
    if [[ "$reset" == "y" ]] || [[ "$reset" == "Y" ]]; then
        cp ~/$path_camera_extrinsic/$default ~/$path_camera_extrinsic/$file_name".yaml"
        echo "Reset param camera_extrinsic."
    else
        echo "Skip set param camera_extrinsic."
    fi
else
    cp ~/$path_camera_extrinsic/$default ~/$path_camera_extrinsic/$file_name".yaml"
    echo "Add param camera_extrinsic with name \"$file_name\""
fi

echo ""
echo "Setup parameter camera_info"
if test -e ~/$path_camera_info/$file_name".yaml"; then
    echo -n "You already have the parameter about camera_info, do you want to reset it? (y/N): "
    read reset
    if [[ "$reset" == "y" ]] || [[ "$reset" == "Y" ]]; then
        cp ~/$path_camera_info/$default ~/$path_camera_info/$file_name".yaml"
        echo "Reset param camera_info."
    else
        echo "Skip set param camera_info."
    fi
else
    cp ~/$path_camera_info/$default ~/$path_camera_info/$file_name".yaml"
    echo "Add param camera_info with name \"$file_name\""
fi

echo ""
echo "Setup parameter decoder_node"
if test -e ~/$path_decoder_node/$file_name".yaml"; then
    echo -n "You already have the parameter about decoder_node, do you want to reset it? (y/N): "
    read reset
    if [[ "$reset" == "y" ]] || [[ "$reset" == "Y" ]]; then
        cp ~/$path_decoder_node/$default ~/$path_decoder_node/$file_name".yaml"
        echo "Reset param decoder_node."
    else
        echo "Skip set param decoder_node."
    fi
else
    cp ~/$path_decoder_node/$default ~/$path_decoder_node/$file_name".yaml"
    echo "Add param decoder_node with name \"$file_name\""
fi
echo ""
echo "Finish setting up."
echo ""