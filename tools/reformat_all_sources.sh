
BASEDIR=$(dirname "$0")

# Already done in indent_with_emace.sh, kept here if
# we just need to do that
function untabify_and_remove_trailing_spaces() {
    cat $1.unformatted | sed -e 's|\t|    |g' -e 's|\s\+$||g' > $1
}

function reformat() {
    echo '>>> reformatting ' $1
    cp $1 $1.before_reformatting
    $BASEDIR/indent_with_emacs.sh $1
}

for i in `find . \( -name third_party -o -name "embedded_*.cpp" -o -name "sphere_model.cpp" \) -prune \\
         -o \( -name "*.h" -o -name "*.cpp" -o -name "*.c" -o -name "*.txt" \) -print`
do
    reformat $i
done
