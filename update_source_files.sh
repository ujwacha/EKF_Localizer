#!/bin/bash

rm TAGS

INCLUDE_FILE="included_files.txt"

find . -type f -iname "*.[chS]" > $INCLUDE_FILE
find . -type f -iname "*.[chS][p][p]" >> $INCLUDE_FILE


cat $INCLUDE_FILE | xargs etags -a

echo "DONE MAKING TAGS"
