ROOT_DIR := $(dir $(realpath $(lastword $(MAKEFILE_LIST))))

PortedPlugins.zip: supercollider
	@echo "creating PortedPlugins.zip..."
	@echo $(ROOT_DIR)
	rm -rf build
	rm -rf /tmp/built
	mkdir -p build
	cd build && cmake .. -DCMAKE_BUILD_TYPE='Release' -DSC_PATH=$(ROOT_DIR)/supercollider -DCMAKE_INSTALL_PREFIX=$(ROOT_DIR)
	cd build && cmake --build . --config Release
	cd build && cmake --build . --config Release --target install
	zip -r PortedPlugins.zip PortedPlugins
	@echo "PortedPlugins.zip ready!"


supercollider:
	@echo "downloading supercollider source..."
	wget -q https://github.com/supercollider/supercollider/archive/refs/tags/Version-3.13.0.tar.gz
	tar -xzf Version-3.13.0.tar.gz
	mv supercollider-Version-3.13.0 supercollider
	rm Version-3.13.0.tar.gz


clean:
	rm -rf supercollider
	rm -rf build
