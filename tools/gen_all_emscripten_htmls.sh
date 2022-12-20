#!/bin/bash

# Calls gen_emscripten_html.sh for a list of files in the specified directory

EXEDIR=$1
SCRIPTDIR=`dirname "$0"`
shift
EXES=$@

if [ -z "$EXEDIR" ] || [ -z "$EXES" ];
then
    echo "Usage: gen_all_emscripten_htmls.sh directory exe1 exe2 ... exeN"
    echo "(where exe is executable name without the .js extension)"
    exit -1
fi

for EXE in $EXES
do
    echo Generating webpage for $EXEDIR/$EXE
    if ! [ -f "$EXEDIR/$EXE.js" ];
    then
        echo "   $EXEDIR/$EXE.js: File not found !"
        exit -1
    fi
    (cd $EXEDIR; ../../../tools/gen_emscripten_html.sh $EXE)
done

INDEX="$EXEDIR/index.html"

echo Generating index: $INDEX

cat tools/head.html > $INDEX
echo "<H1> Emscripten demos </H1>" >> $INDEX
echo "<ul>" >> $INDEX
for EXE in $EXES
do
    echo "<li> <a href="$EXE.html"> $EXE </a> </li>" >> index.html
done
echo "</ul>" >> $INDEX

