set -e
mkdir tmp
cp $1 tmp/
cp lib/* tmp/
cat>tmp/entry.sh<<'END'
#!/bin/bash
export LD_LIBRARY_PATH=$PWD:$LD_LIBRARY_PATH
echo $LD_LIBRARY_PATH
END
cat>>tmp/entry.sh<<END
python $(basename $1)
END
rm release.zip
zip -q -j release.zip tmp/*
rm -r tmp/
cp release.zip /mnt/c/Users/Master/Desktop
rm release.zip
echo 'DONE'