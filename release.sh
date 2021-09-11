cd ./build/
sh build_release.sh
cd ./build_tmp/
ln -f -s ~/Code/AutoPilot/ICV-Competition/build/build_tmp/compile_commands.json ~/Code/AutoPilot/ICV-Competition/compile_commands.json
cd ../../
rm Controller.zip
zip -j Controller.zip ./bin/*
echo -e "\a"
echo DONE!!!
date
