rm -rf src/adjustBox/build
mkdir src/adjustBox/build
cd src/adjustBox/build
cmake . ..
make
cd ../../..

rm -rf src/rec3D/build
mkdir src/rec3D/build
cd src/rec3D/build
cmake . ..
make
cd ../../..

rm -rf src/marchingCubes/build
mkdir src/marchingCubes/build
cd src/marchingCubes/build
cmake . ..
make
cd ../../..

rm -rf src/paintMesh/build
mkdir src/paintMesh/build
cd src/paintMesh/build
cmake . ..
make
cd ../../..

rm -rf binaries
mkdir binaries
mv src/adjustBox/build/adjustBox binaries
mv src/rec3D/build/rec3D binaries
mv src/marchingCubes/build/marchingCubes binaries
mv src/paintMesh/build/paintMesh binaries

rm -rf src/adjustBox/build
rm -rf src/rec3D/build
rm -rf src/marchingCubes/build
rm -rf src/paintMesh/build
