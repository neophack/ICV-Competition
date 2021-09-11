set -e

bash build_clean.sh

mkdir build_tmp
cd build_tmp
cmake .. -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_BUILD_TYPE=Release -G "Unix Makefiles"
make -j10
