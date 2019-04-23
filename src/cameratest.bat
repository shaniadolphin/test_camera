echo param[0] = %0
@echo off
adb push ./capture /data
adb push ./capture.sh /data

adb shell "chmod a+x /data/capture"
adb shell "chmod a+x /data/capture.sh"

adb shell "cd /data && ./capture.sh %1 %2 %3" 

adb shell sleep 2

adb pull /data/%2  . 


