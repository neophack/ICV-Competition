set -e

# cd build
# bash build_release.sh
# cd ..
obs="5 5 15 2 10 10 2 10"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:./bin
./bin/TEST >out.txt 0 0 0 5 10 3.14 ${obs}
python.exe 1.py ${obs}