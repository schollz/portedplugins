builder:
	rm -rf build
	rm -rf /tmp/built
	mkdir -p build
	mkdir -p /tmp/built
	cd build && cmake .. -DCMAKE_BUILD_TYPE='Release' -DSC_PATH=/tmp/supercollider -DCMAKE_INSTALL_PREFIX=/tmp/built
	cd build && cmake --build . --config Release
	cd build && cmake --build . --config Release --target install
