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

# ---- Evaluation and demo ----------------------------------------------------
# Runs as the calling user so results/ stays writable; HOME=/tmp for ROS logs.
DOCKER_RUN = docker run --rm --user $$(id -u):$$(id -g) -e HOME=/tmp \
	-v $(CURDIR)/results:/results $(IMAGE)
EVAL_JOBS ?= 6
EVAL_SEEDS ?= 1,2,3
# Python with numpy + matplotlib + pillow, for the GIF only.
GIF_PYTHON ?= python3

.PHONY: eval demo gif reproduce

# Full experiment grid (~10 min): results/eval_runs.csv + eval_summary.md.
# Per-run files under results/runs/ are overwritten.
eval: docker-build
	mkdir -p results
	$(DOCKER_RUN) ros2 run autonomy_bringup run_eval.py --out /results \
	  --jobs $(EVAL_JOBS) --seeds $(EVAL_SEEDS)

# One recorded demo run (blocked path + BT recovery) with a rosbag. The bag
# directory must not exist yet: delete results/demo/run/bag before re-running.
demo: docker-build
	mkdir -p results/demo
	$(DOCKER_RUN) ros2 launch autonomy_bringup navigation.launch.py seed:=7 \
	  planner:=astar controller:=pure_pursuit drop_after_m:=2.0 \
	  record_dir:=/results/demo/run record_bag:=true

gif:
	$(GIF_PYTHON) scripts/make_gif.py results/demo/run docs/demo.gif

reproduce: eval demo gif
