# generate_report.sh
# generates test reports and dashboard from github action artifacts
# (artifacts are zip files with test results)

mkdir reports

for artifact in `ls *.zip`
do
  dirname=`basename $artifact .zip`
  mkdir reports/$dirname
  (cd reports/$dirname; unzip ../../$artifact)
done
		
# debug
pwd
find reports -print
