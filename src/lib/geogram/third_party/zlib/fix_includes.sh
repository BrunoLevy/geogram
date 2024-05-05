for i in `ls *.h *.c`
do
  cp $i $i.back
  cat $i.back | sed -e 's|#include \"\(.*\)\"|#include <geogram/third_party/zlib/\1>|g' > $i
done  
