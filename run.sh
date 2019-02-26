#rm -r build
#mkdir build
cd build
cmake ..
make -j8
echo "-------------------build finished-----------------------\n\n"
./thesis
cd ..

./scripts/visualization/path_planning_vis.py
