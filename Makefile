get_libraries:
#	sudo apt install libopencv-dev
	go get github.com/hschendel/stl
	go install gocv.io/x/gocv@latest
	go install github.com/fogleman/delaunay@latest

build_c_shared_lib:
	g++ -shared -o libtriangulate.so triangulate.cpp $(pkg-config --cflags --libs opencv)

build:
	go build -o bin/server -v

build_docker:
	docker build -t joram87/imagetostl .