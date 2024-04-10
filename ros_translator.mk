
# This Makefile contains useful targets that can be included in downstream projects.

ifeq ($(filter ros_translator.mk, $(notdir $(MAKEFILE_LIST))), ros_translator.mk)

.EXPORT_ALL_VARIABLES:
SHELL:=/bin/bash
ROS_TRANSLATOR_PROJECT:=ros_translator

ROS_TRANSLATOR_MAKEFILE_PATH:=$(shell realpath "$(shell dirname "$(lastword $(MAKEFILE_LIST))")")
ifeq ($(SUBMODULES_PATH),)
    ROS_TRANSLATOR_SUBMODULES_PATH:=${ROS_TRANSLATOR_MAKEFILE_PATH}
else
    ROS_TRANSLATOR_SUBMODULES_PATH:=$(shell realpath ${SUBMODULES_PATH})
endif
MAKE_GADGETS_PATH:=${ROS_TRANSLATOR_SUBMODULES_PATH}/make_gadgets
ifeq ($(wildcard $(MAKE_GADGETS_PATH)/*),)
    $(info INFO: To clone submodules use: 'git submodule update --init --recursive')
    $(info INFO: To specify alternative path for submodules use: SUBMODULES_PATH="<path to submodules>" make build')
    $(info INFO: Default submodule path is: ${ROS_TRANSLATOR_MAKEFILE_PATH}')
    $(error "ERROR: ${MAKE_GADGETS_PATH} does not exist. Did you clone the submodules?")
endif

ROS_TRANSLATOR_TAG:=$(shell cd "${MAKE_GADGETS_PATH}" && make get_sanitized_branch_name REPO_DIRECTORY="${ROS_TRANSLATOR_MAKEFILE_PATH}")
ROS_TRANSLATOR_IMAGE:=${ROS_TRANSLATOR_PROJECT}:${ROS_TRANSLATOR_TAG}
ROS_TRANSLATOR_PROJECT_X11_DISPLAY:=${ROS_TRANSLATOR_PROJECT}_x11_display
ROS_TRANSLATOR_IMAGE_X11_DISPLAY:=${ROS_TRANSLATOR_PROJECT_X11_DISPLAY}:${ROS_TRANSLATOR_TAG}

SOURCE_DIRECTORY?=${REPO_DIRECTORY}

DOCKER_COMPOSE_FILE?=${ROS_TRANSLATOR_MAKEFILE_PATH}/docker-compose.yaml

UID := $(shell id -u)
GID := $(shell id -g)


include ${MAKE_GADGETS_PATH}/make_gadgets.mk
include ${MAKE_GADGETS_PATH}/docker/docker-tools.mk

REPO_DIRECTORY:=${ROS_TRANSLATOR_MAKEFILE_PATH}

ROS_TRANSLATOR_SUBMODULES:=make_gadgets

.PHONY: ros_translator_check
ros_translator_check:
	@if [ -z "$$(docker images -q ${ROS_TRANSLATOR_PROJECT}:${ROS_TRANSLATOR_TAG})" ]; then \
		echo "ros_translator docker image: '${ROS_TRANSLATOR_PROJECT}:${ROS_TRANSLATOR_TAG}' does not exits in the local docker repository. "; \
        echo "Did you build ros_translator?"; \
        echo "  Hint: run 'make build' to build the ros_translator and try again."; \
        exit 1; \
    fi

.PHONY: start_ros_translator
start_ros_translator: ros_translator_check stop_ros_translator ros_translator_up ## Start ros translator 

.PHONY: stop_ros_translator
stop_ros_translator: docker_host_context_check ros_translator_down ## Stop ros_translator docker context if it is running
.PHONY: build_fast_ros_translator
build_fast_ros_translator: # build the ros_translator conte does not already exist in the docker repository. If it does exist this is a noop.
	@if [ -n "$$(docker images -q ${ROS_TRANSLATOR_PROJECT}:${ROS_TRANSLATOR_TAG})" ]; then \
        echo "Docker image: ${ROS_TRANSLATOR_PROJECT}:${ROS_TRANSLATOR_TAG} already build, skipping build."; \
    else \
        cd "${ROS_TRANSLATOR_MAKEFILE_PATH}" && make build_ros_translator;\
    fi

.PHONY: build_ros_translator
build_ros_translator: clean_ros_translator ## Builds the BOB translator docker context/image
	cd "${ROS_TRANSLATOR_MAKEFILE_PATH}" && make build 

.PHONY: clean_ros_translator 
clean_ros_translator: ## Clean ros_translator docker context 
	cd "${ROS_TRANSLATOR_MAKEFILE_PATH}" && make clean

.PHONY: ros_translator_down
ros_translator_down:
	@echo "Running ros_translator down..."
	@cd ${ROS_TRANSLATOR_MAKEFILE_PATH} && export TAG=${ROS_TRANSLATOR_TAG}; docker compose -f ${DOCKER_COMPOSE_FILE} down --remove-orphans --volumes || true
	@cd ${ROS_TRANSLATOR_MAKEFILE_PATH} && docker compose -f ${DOCKER_COMPOSE_FILE} rm -f || true

.PHONY: ros_translator_start
ros_translator_up:
	@echo "Starting ros_translator..."
	cd ${ROS_TRANSLATOR_MAKEFILE_PATH} && \
    UID=${UID} GID=${GID} docker compose -f ${DOCKER_COMPOSE_FILE} up \
      --force-recreate \
      --renew-anon-volumes \
      --detach;


.PHONY: attach_ros_translator
attach_ros_translator: ros_translator_attach

.PHONY: ros_translator_attach
ros_translator_attach:
	@echo "Running ros_translator attach..."
	docker exec -it --user ros_translator ros_translator /bin/bash

.PHONY: branch_ros_translator
branch_ros_translator: ## Returns the current docker safe/sanitized branch for ros_translator 
	@printf "%s\n" ${ROS_TRANSLATOR_TAG}

.PHONY: image_ros_translator
image_ros_translator: ## Returns the current docker image name for ros_translator
	@echo "${ROS_TRANSLATOR_IMAGE}"

.PHONY: images_ros_translator
images_ros_translator: ## Returns all docker images for ros_translator
	@echo "${ROS_TRANSLATOR_IMAGE}"

endif
