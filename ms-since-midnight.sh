h=`date +%H`
m=`date +%M`
s=`date +%S`
totalSeconds=`expr \( $h \* 60 \* 60 \) + \( $m \* 60 \) + $s`
expr $totalSeconds \* 1000
