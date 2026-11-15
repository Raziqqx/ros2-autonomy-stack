# One-command entry points. Native targets need only g++/CMake/Ninja; docker
# targets need Docker and build everything (ROS nodes included) in a container.
IMAGE ?= ros2-autonomy-stack:jazzy
NATIVE_BUILD ?= build/native
JOBS ?= 4

.PHONY: test native-test docker-build docker-test docker-shell format

test: native-test

# Pure-maths libraries under AddressSanitizer + UndefinedBehaviorSanitizer.
native-test:
	cmake -S native -B $(NATIVE_BUILD) -G Ninja -DCMAKE_BUILD_TYPE=Debug -DSANITIZE=ON -Wno-dev
	cmake --build $(NATIVE_BUILD) -j $(JOBS)
	ctest --test-dir $(NATIVE_BUILD) --output-on-failure

docker-build:
	docker build -t $(IMAGE) .

# colcon test inside the image: gtest suites and launch_testing scenarios.
docker-test: docker-build
	docker run --rm $(IMAGE) bash -c "colcon test --event-handlers console_cohesion+ \
	  --return-code-on-test-failure && colcon test-result --verbose"

docker-shell: docker-build
	docker run --rm -it $(IMAGE) bash

format:
	clang-format -i $$(find src -name '*.cpp' -o -name '*.hpp')
