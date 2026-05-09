IMAGE ?= adohaha/fun_ros:jazzy

up:
	docker compose up

jupyter:
	docker container exec -it --user ubuntu ros_fun bash -i /home/ubuntu/run_jupyter.sh

build:
	docker build -t $(IMAGE) .

push:
	docker push $(IMAGE)

release: build push
