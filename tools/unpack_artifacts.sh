# unpack artifacts.sh
# used to test generate_reports.sh manually
#
# How to use:
# download all .zip artifacts 
# $ tools/unpack_artifacts.sh
# $ tools/generate_report.sh

rm -fr artifacts

for archive in `ls *.zip`
do
    basename=`basename $archive .zip`
    mkdir -p artifacts/$basename
    (cd artifacts/$basename; unzip ../../$archive)
done
		
