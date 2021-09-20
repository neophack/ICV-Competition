set -e

if [ -d "build_tmp" ]; then
    echo "Deleting build_tmp..."
    rm -rf build_tmp
fi

echo "DONE"
