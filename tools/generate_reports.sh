# generate_report.sh
# generates test reports and dashboard from github action artifacts
# (artifacts are zip files with test results)

for artifact in `ls *.zip`
do
    category=`basename $artifact .zip | sed -e 's|-.*||`
    config=`basename $artifact .zip | sed -e 's|^[^-]*-.*||`
    mkdir -p reports/$category/$config
    (cd reports/$category/$config; unzip ../../$artifact)
done
		
# debug
pwd
find reports -print
