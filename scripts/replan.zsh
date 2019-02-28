#!/bin/zsh
basePath="/Users/zhuoyang/workspace/thesis/"
cd ${basePath}cmake-build-debug
./thesis

echo -n "Do you want to save result(Y/N)?   "
read ifsave

if [ "$ifsave" = "Y" ]
then
    echo "yes"
    echo -n "Next number"
    read num
    echo "$num"

    cp ${basePath}for_vis/environment_conf_0.txt ${basePath}for_vis/environment_conf_${num}.txt
    cp ${basePath}for_vis/environment_conf_0.txt_next ${basePath}for_vis/environment_conf_0.txt
    cp ${basePath}tmp_data/path_0.txt ${basePath}for_vis/path_${num}.txt
    cp ${basePath}tmp_data/speed_0.txt ${basePath}for_vis/speed_${num}.txt
else
    echo "no"
fi

exit 0

