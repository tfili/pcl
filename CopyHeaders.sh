#! /bin/sh

if [[ ! -d "pcl/" ]]; then
    echo "Creating pcl directory"
    `mkdir pcl`
fi

printf "Copying includes."
for f in * ; do
    if [[ -d "$f/include/pcl/" && ! -L "$f/include/pcl/" ]]; then
        `cp -R "$f/include/pcl/$f/" "./pcl/"`
        printf "."
    fi
done
printf "Complete\n"
