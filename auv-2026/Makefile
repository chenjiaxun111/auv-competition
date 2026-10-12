.PHONY: all configure clean rebuild

BUILD_DIR ?= build

all: configure
	$(MAKE) -C $(BUILD_DIR)

configure:
	cmake -E make_directory $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake ..

clean:
	cmake -E remove_directory $(BUILD_DIR)

rebuild: clean all
