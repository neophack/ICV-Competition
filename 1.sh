set -e

cd build
bash build_release.sh
cd ..
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:./bin
./bin/TEST >out.txt
python.exe 1.py