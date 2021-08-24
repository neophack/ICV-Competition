set -e

if [ -d "build_tmp" ]; then
    echo "Deleting build_tmp..."
    rm -rf build_tmp
fi

if [ -d "../bin" ]; then
    echo "Deleting bin..."
    rm -rf ../bin
fi

echo "DONE"