set -e

# cd build
# bash build_release.sh
# cd ..
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:./bin
./bin/TEST >out.txt 0 0 0 10 15 1.57 5 5 10 2
python.exe 1.py 5 5 10 2