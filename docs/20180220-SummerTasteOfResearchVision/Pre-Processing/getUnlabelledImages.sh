#!/bin/bash

prev=""
path=""
while read line
do
    if [[ $prev =~ "Enter Image Path:" && $line =~ "Enter Image Path:" ]]
    then
        path=`echo $prev | sed 's/^.*\/ToR\///;s/:.*$//'`
        # prev=`echo $prev | sed 's/Enter Image Path: //;s/: .*$//'`
        cp "$path" 'to_label/.'
        # echo $path
    fi
    prev=$line

done
