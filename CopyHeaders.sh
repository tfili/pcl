#! /bin/bash

if [[ -d "pcl/" ]]; then
    echo "Removing old pcl directory"
    `rm -rf ./pcl`
    `mkdir ./pcl`
fi

printf "Copying includes."
`cp include/pcl/* ./pcl/`
for f in * ; do
    if [[ -d "$f/include/pcl/" && ! -L "$f/include/pcl/" ]]; then
        `cp -R $f/include/pcl/* ./pcl/`
        printf "."
    fi
done
printf "Complete\n"
