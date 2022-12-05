# generate_report.sh
# generates test reports and dashboard from github action artifacts
# (artifacts are zip files with test results)

for artifact in `ls artifacts`
do
    category=`echo $artifact | sed -e 's|-.*||'`
    config=`echo $artifact | sed -e 's|^[^-]*-||'`
    echo CATEGORY=$category
    echo CONFIG=$config
    mkdir -p reports/$category/$config
    cp -r artifacts/$artifact/* reports/$category/$config/
done
		
for category in `ls reports`
do
    output=reports/$category/index.html
    echo > $output
    echo "<H1> " >> $output
    echo "<img width=\"64\" style=\"vertical-align:middle\" src=\"https://upload.wikimedia.org/wikipedia/commons/e/e4/Robot-framework-logo.png\"/>" >> $output
    echo "$category tests - Robot Framework reports </H1>" >> $output
    echo "</H1>" >> $output
    echo "<ul>" >> $output
    for config in `ls reports/$category | egrep -v '.html$'`
    do
	echo "<li>" >> $output
	echo "<a href=\"$config/report.html\"> <img style=\"vertical-align:middle\" src=\"$config/robot_status.png\"/>$config </a>" >> $output
	echo "</li>" >> $output	
    done
    echo "</ul>" >> $output
done
