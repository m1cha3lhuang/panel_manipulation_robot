# build
mkdir -p build &&
dirs -c &&
pushd build &&
cmake .. -DCMAKE_BUILD_TYPE=Release &&
make &&
popd &&

# run
cd bin &&
./demo1
