## [remove sudo if running from inside a docker image]
## make sure image-view ros pkg is installed:
# sudo apt-get update
# sudo apt install ros-kinetic-image-view 

## to avoid an error when calling export.launch, run:
# sudo apt-get update
# sudo apt-get upgrade

## make sure ffmpeg is installed
# sudo apt install ffmpeg

# setup the environment
source devel/setup.bash

# please make sure the bag is copied to the repo folder (./ros parent)

# create images
echo exporting images from bag file ../udacity_succesful_light_detection.bag;
echo change file if needed at launch/export.launch
echo
echo after writing images, this process may hang at writing the log file;
echo if it happens, please kill it [with Ctrl-C] - only once to avoid killing the script too
# delay for reading the note above
sleep 3
#
roslaunch launch/export.launch
rm -rf bag_imgs
mkdir bag_imgs; mv ~/.ros/*.jpg bag_imgs

# create video
echo creating video from the images
# before running the python file, make sure pandas and matplotlib are installed:
# pip install pandas
# pip install matplotlib
cd bag_imgs
ffmpeg -framerate 30 -pattern_type glob -i '*.jpg' -c:v libx264 -r 30 -pix_fmt yuv420p out.mp4
cd ..

echo Images and video are now saved at folder ./bag_imgs
