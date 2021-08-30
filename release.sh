cd ./build/
sh build_release.sh
cd ../
rm Controller.zip
zip -j Controller.zip ./bin/AEB
zip -j Controller.zip ./bin/AVP
zip -j Controller.zip ./lib/*
echo -e "\a"
echo Success!!!
date
