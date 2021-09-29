set -e

if [ ! -d "../bin" ]; then
    echo "ERROR: Could not find files"
    exit 1
fi

zip -q -j release.zip ../bin/*
cp release.zip /mnt/c/Users/Master/Desktop
rm release.zip
echo "DONE"
