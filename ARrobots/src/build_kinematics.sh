rm -rf build CMakeFiles CMakeCache.txt
mkdir -p build
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
