function replace_copyright() {
    (cat tools/header.txt; awk  < $1 'BEGIN {STATE=0}{if(STATE==0 && $1 == "/*") {STATE=1} else if(STATE==1 && $1 == "*/") {STATE=2} else if(STATE==2) {print;} }') > /tmp/gloglo.txt
    mv /tmp/gloglo.txt $1
}

for i in `find . \( -name "*.h" -o -name "*.cpp" -o -name "*.c" \) -print`
do
   echo ">>>" $i
   if grep -q 'Contact: Bruno' $i; then
      echo "....replacing copyright in " $i
      replace_copyright $i
   else
      echo "....skipping " $i   
   fi
done