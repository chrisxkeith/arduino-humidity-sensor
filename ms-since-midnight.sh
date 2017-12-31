set -x

h=`date +%H`
m=`date +%M`
s=`date +%S`

# assume two minutes to upload and start program running on the Arduino
m=`expr $m + 2`

totalSeconds=`expr \( $h \* 60 \* 60 \) + \( $m \* 60 \) + $s`
expr $totalSeconds \* 1000
