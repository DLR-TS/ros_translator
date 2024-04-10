SHELL:=/bin/bash

.DEFAULT_GOAL := all

ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
.EXPORT_ALL_VARIABLES:
include ros_translator.mk
TAG=${ROS_TRANSLATOR_TAG}
BRIDGE_WORKSPACE=/home/${USER}/bridge_workspace

.PHONY: start
start: clean build_ros_translator start_ros_translator ## Start the ros translater, same as 'start_ros_translator'

.PHONY: stop
stop: stop_ros_translator ## Stop the ros translater, same as 'stop_ros_translator'

.PHONY: production
production:
	ln -sf FASCarE.yaml docker-compose.yaml

.PHONY: develop
develop:
	ln -sf Development.yaml docker-compose.yaml

.PHONY: up_fast
up_fast: 
	git submodule update --init --recursive 
	make up

.PHONY: up
up: clean build_ros_translator start_ros_translator

.PHONY: all
all: build_ros_translator

.PHONY: generate_bridge_packages
generate_bridge_packages:
	bash generate_bridge_messages.sh


.PHONY: build
build: generate_bridge_packages
	docker compose build

.PHONY: test
test:
	docker compose up bridge-debug-publisher-1to2

.PHONY: clean
clean:
	rm -rf bridge_workspace/src/bridge_packages/*
	docker rm $$(docker ps -a -q --filter "ancestor=${ROS_TRANSLATOR_IMAGE_NAME}") 2> /dev/null || true
	docker rmi $$(docker images -q ${ROS_TRANSLATOR_IMAGE_NAME}) 2> /dev/null || true

.PHONY: rostopic_list
rostopic_list:
	@echo "ROS 1 topics (ROS master):"
	@docker compose exec ros-master bash -c 'source ros1.env && rostopic list'
	@echo ""
	@echo "ROS 1 topics (ROS 1 bridge):"
	@docker compose exec ros_dynamic_bridge bash -c 'unset ROS_DISTRO; source ros1_bridge.env && rostopic list'
	@echo ""
	@echo "ROS 2 topics:"
	@docker compose exec ros_dynamic_bridge bash -c 'unset ROS_DISTRO; source ros1_bridge.env && ros2 topic list'

.PHONY: print_pairs
print_pairs:
	@docker compose exec ros_dynamic_bridge bash -c 'cd $${BRIDGE_WORKSPACE} && bash print_pairs.sh'

.PHONY: ros1_topic_echo
ros1_topic_echo:
	@docker compose exec ros_dynamic_bridge bash -c 'source ros1.env && rostopic echo adore'

.PHONY: ros2_topic_echo
ros2_topic_echo:
	@docker compose exec ros_dynamic_bridge bash -c 'source ros2.env && ros2 topic echo ROS_Translator_2to1'

.PHONY: ros2_debug_echo
ros2_debug_echo:
	@docker compose exec ros_dynamic_bridge bash -c 'source ros2.env && ros2 topic echo ros_bridge_heartbeat'


.PHONY: ros2_pkg_list
ros2_pkg_list:
	@docker compose exec ros_dynamic_bridge bash -c 'source ros1_bridge.env && ros2 pkg list'

.PHONY: rospack_list
rospack_list:
	@docker compose exec ros_dynamic_bridge bash -c 'source ros1_bridge.env && rospack list | cut -d " " -f1'

.PHONY: ros2_interface_list
ros2_interface_list:
	@docker compose exec ros_dynamic_bridge bash -c 'source ros1_bridge.env && ros2 interface list'

.PHONY: rosmsg_list
rosmsg_list:
	@docker compose exec ros_dynamic_bridge bash -c 'source ros1_bridge.env && rosmsg list'

.PHONY: bridge_debug
bridge_debug:
	@docker compose exec ros_dynamic_bridge bash -c 'source ros2.env && /bin/bash'

.PHONY: debug
debug:
	@docker compose exec ros_dynamic_bridge bash -c '/bin/bash'

.PHONY: bag_replay
bag_replay:
	@docker compose exec ros_dynamic_bridge bash -c 'source ros1.env && cd bridge_workspace/bags/ && while true; do rosbag play 2024-02-05-13-13-32.bag; sleep 1; done'
