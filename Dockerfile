FROM robotgit.localdom.net:5000/ros2/eloquent-onbuild as common_base

LABEL version="0.0.2"
LABEL ros.distro=${ROS_DISTRO}
LABEL vcs-url="http://robotgit.localdom.net/ai-box/components/anyfeeder/anyfeeder_connector"
LABEL maintainer="Rasmus Lunding Henriksen <rlh@teknologisk.dk>"

ENV PACKAGE_NAME="anyfeeder_connector"
ENV LAUNCHFILE_NAME="anyfeeder_connector.launch.py"