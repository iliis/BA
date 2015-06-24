#!/bin/bash -e

frame_count=$(ls *_warped_image.png | wc -l)
echo "converting $frame_count frames ..."

for (( i=0; i<"$frame_count"; i++ )); do
    number=$(printf "%04d" $i)
    echo "converting frame $i of $frame_count"

    #mogrify -resize 256x256 "patch_$number.png"  "patch_$number.png"
    #mogrify -resize 256x256 "events_$number.png" "events_$number.png"
    #convert -colorspace RGB "world_$number.png" '(' "patch_$number.png" "events_$number.png" -append ')' -gravity center +append "animation_$number.png"
    montage \
        $number"_errors_current.png" \
        $number"_J_norm.png" \
        $number"_warped_image.png" \
        keyframe.png \
        -geometry +0+0 "animation_$number.png"
done

echo "OK"

echo "creating movie ..."
ffmpeg -framerate 30 -i "animation_%04d.png" -c:v libx264 animation.mp4
