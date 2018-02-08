set -x
#
h=`date +%H`
m=`date +%M`
s=`date +%S`

# 16 seconds to upload and start program running on the Arduino.
# Add 4 seconds for copy/paste from 'git bash' terminal window.
# https://docs.google.com/spreadsheets/d/19OAX2rzPMKg2Ewn-qQGKnhIQSxPEfInxWIr4pjWWSbI/edit
s=`expr $s + 16`

totalSeconds=`expr \( $h \* 60 \* 60 \) + \( $m \* 60 \) + $s`
expr $totalSeconds \* 1000
