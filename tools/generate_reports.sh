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
    echo "<img width=\"40\" style=\"vertical-align:middle\" "\
         "src=\"Images/robot.png\"/>" >> $output
    echo "$category tests - Robot Framework reports </H1>" >> $output
    echo "</H1>" >> $output
    echo "<ul>" >> $output
    for config in `ls reports/$category`
    do
	if [[ -d "reports/$category/$config" ]]
	then
	    echo "<li>" >> $output
	    if [[ -f "reports/$category/$config/TESTS_SUCCESS" ]]
	    then
		echo "OK $category/$config"
		echo "<img style=\"vertical-align:middle\" " \
		     "src=\"Images/ok.png\"/>" >> $output
	    else
		echo "KO $category/$config"		
		echo "<img style=\"vertical-align:middle\" " \
		     "src=\"Images/ko.png\"/>" >> $output
	    fi
	    echo "&nbsp;" >> $output
	    echo "<a href=\"$config/report.html\">$config</a>" >> $output
	    echo "</li>" >> $output
	fi
    done
    echo "</ul>" >> $output
    # comes last, so that Images is ignored in iteration above
    cp -r tools/Images reports/$category
done


