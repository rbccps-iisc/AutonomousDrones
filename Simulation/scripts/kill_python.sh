kill -9 $(ps | grep python | grep -v grep | awk '{ print $1 }')
