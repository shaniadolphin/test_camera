import os
import re    
import subprocess

def getCamera(dev, file, width):
    cmdcap = "./capture.sh " + width + " "+ file + " "+ dev
    print(cmdcap)
    cmds = [
        "cd data",
        "chmod a+x /data/capture",
        "chmod a+x /data/capture.sh",
        "rm -rf *.jpg",
        "./capture.sh 1920 a.jpg video2",
        "cat /sys/class/net/wlan0/address",
        "exit",
    ]
    cmds[4] = cmdcap
    os.popen('adb push ./capture /data')
    os.popen('adb push ./capture.sh /data')    
    obj = subprocess.Popen("adb shell", shell= True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    info = obj.communicate(("\n".join(cmds) + "\n").encode('utf-8'));
    for item in info:
        if item:
            print(item.decode('gbk'))
    cmdcap = "adb pull /data/" + file + " ."
    print(cmdcap)
    os.popen(cmdcap)
    
if __name__ == '__main__':
    getCamera('video2', '123.jpg', '1920')
