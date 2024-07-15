function format_spaces() {
    echo ">>>" $i
    cp $1 $1.unformatted
    cat $1.unformatted | sed -e 's|\t|    |g' -e 's|\s\+$||g' > $1
}

for i in `find . -name third_party -prune -o \( -name "*.h" -o -name "*.cpp" -o -name "*.c" -o -name "*.txt" \) -print`
do
    format_spaces $i
done
