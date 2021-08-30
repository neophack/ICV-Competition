cd ./build/
sh build_release.sh
cd ../
rm Controller.zip
zip -j Controller.zip ./bin/*
echo -e "\a"
echo Success!!!
date
