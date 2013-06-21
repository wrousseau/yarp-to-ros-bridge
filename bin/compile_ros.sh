# Do not call this manually !
# This script will be called by the bridge at run-time
# This script will be executed from yarp_bridge/build

if [ -n "$BASH_VERSION" ]; then
    # include .bashrc if it exists
    if [ -f "$HOME/.bashrc" ]; then
        . "$HOME/.bashrc"
    fi
fi
if [ -n "$ZSH_VERSION" ]; then
    # include .bashrc if it exists
    if [ -f "$HOME/.zshrc" ]; then
        . "$HOME/.zshrc"
    fi
fi

rm -f rosreception

# Cmake
echo "Generating the Headers (Cmake) ..."
if [ $# -eq 1 ]; then
	roscd to_ros_image_handler
	rm -rf build
	roscd to_ros_data_handler
	mkdir -p build
	cd build
	cmake .. > /dev/null
else
	roscd to_ros_data_handler
	rm -rf build
	roscd to_ros_image_handler
	mkdir -p build
	cd build
	cmake .. > /dev/null
fi

# Make 
echo "Compiling the ROS module ..."
make > /dev/null
if [ $? -eq 0 ] ; then # Everything went well
	cd ../../../bin
	rm -f lastedit.txt
	echo "$1" > lastedit.txt # Refreshing the last edit since everything went well
fi
