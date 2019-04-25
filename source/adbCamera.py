import os
import re    
import subprocess
import cv2
import time
import matplotlib.pyplot as plt
import numpy as np



def go_split(s, symbol):
    result = [s]
    for i in symbol:
        median = []
        # 普通方法
        # for x in result:
        #   median.extend(x.split(i)
        # 列表解析
        # [median.extend(y.split(i)) for y in result if y]
        # map高阶函数,map生成可迭代对象
        for z in map(lambda x: x.split(i), result):
            median.extend(z)
        # 以上三个方法都可以解决问题
        result = median

    # 去除空字符串
    return [x for x in result if x]

def HexToByte( hexStr ):
    return bytes.fromhex(hexStr)

def adb_devices():
    """
    Do adb devices
    :return The first connected device ID
    """
    cmd = "adb devices"
    c_line = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()[0]
    #if c_line.find("List of devices attached") < 0: # adb is not working
    #    return None
    #info = c_line.split("\t")[0].split("\r\n")[-1]
    #print(info)
    #return info# This line may have different



def getMacaddress():
    cmds = [
        "cat /sys/class/net/wlan0/address",
        "exit",
    ]
    obj = subprocess.Popen("adb shell", shell= True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    info = obj.communicate(("\n".join(cmds) + "\n").encode('utf-8'));
    #obj.wait() 
    #info = obj.stdout.readlines() 
    for item in info:
        if item:
            print(item.decode('gbk'))
    cmds = [
        "cat /proc/rid",
        "exit",
    ]
    symbol = 'n;sh-3.2'
    obj = subprocess.Popen("adb shell", shell= True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    info = obj.communicate(("\n".join(cmds) + "\n").encode('utf-8'));
    cinfo = str(info)
    infoa = go_split(cinfo, symbol)
    print("info:",infoa)
    cpu_id = ""
    cpuid = "'" + infoa[3] +"'"
    y = [ord(c) for c in cpuid]
    print("list y:",y)
    for i in y:
        cpu_id += hex(i).split("0x")[1]
    print(cpu_id)
    return cpu_id
    

def grey_world(nimg):  
    nimg = nimg.transpose(2, 0, 1).astype(np.uint32)  
    avgB = np.average(nimg[0])  
    avgG = np.average(nimg[1])  
    avgR = np.average(nimg[2])  

    avg = (avgB + avgG + avgR) / 3  
    nimg[0] = np.minimum(nimg[0] * (avg / avgB), 255)  
    nimg[1] = np.minimum(nimg[1] * (avg / avgG), 255)  
    nimg[2] = np.minimum(nimg[2] * (avg / avgR), 255)  
    return nimg.transpose(1, 2, 0).astype(np.uint8)  
    
    
def getCamera(dev, file, width):
    cmdcap = "./capture.sh " + width + " "+ file + " "+ dev
    cmds = [
        "cd tmp",
        "chmod a+x /tmp/capture",
        "chmod a+x /tmp/capture.sh",
        "rm -rf *.jpg",
        "./capture.sh 1920 a.jpg video2",
        "exit",
    ]
    cmds[4] = cmdcap
    os.popen('adb push ./capture /tmp')
    os.popen('adb push ./capture.sh /tmp')    
    obj = subprocess.Popen("adb shell", shell= True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    info = obj.communicate(("\n".join(cmds) + "\n").encode('utf-8'));
    for item in info:
        if item:
            print(item.decode('gbk'))
    cmdcap = "adb pull /tmp/" + file + " ."
    os.popen(cmdcap)
    return 1

def record():
    cmds = [
        "cd tmp",
        "chmod a+x /tmp/vol.sh",
        "rm -rf *.pcm",
        "arecord -r 16000 -c 2 -d 10 -f S16_LE record.pcm",
        "exit",
    ]
    cmds[4] = cmdcap
    os.popen('adb push ./vol.sh /tmp')
    
def play(mpegfile):
    cmds = [
        "cd tmp",
        "rm /mtkac*",
        "gst-launch-1.0 playbin",
        "exit",
    ]
    cmdcap = "adb push " + mpegfile + " /tmp"
    os.popen(cmdcap)
    cmdcap = "gst-launch-1.0 playbin audio-sink='alsasink device=sub0' flags=0x42 uri=file:///tmp/" + mpegfile
    cmds[4] = cmdcap
    obj = subprocess.Popen("adb shell", shell= True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    info = obj.communicate(("\n".join(cmds) + "\n").encode('utf-8'));
    for item in info:
        if item:
            print(item.decode('gbk'))

    
if __name__ == '__main__':
    exitfun = 0
    count = 0
    while (1):
        imgnamepr = time.strftime("%Y%m%d%H%M%S")+"_"+str(count)
        imgname = imgnamepr + ".jpg"
        getCamera('video2', imgname, '1920')
        time.sleep(1)
        img = cv2.imread(imgname)
        img = cv2.resize(img, (0,0), fx=0.5, fy=0.5)
        img = grey_world(img)
        imgname = imgnamepr + ".png"
        cv2.imwrite(imgname, img)
        print("save imgfile:", imgname)
        cv2.imshow('abc', img)
        count = count+1
        while (1):
            key = cv2.waitKey(1)
            if key & 0xFF == ord(' '):
                break
            elif key & 0xFF == ord('q') or key == 27:
                exitfun = 1
                break
        if exitfun == 1:
            break
    cv2.destroyAllWindows()
    """"""
    #play(my.mp3)
    #adb_devices()
    #getMacaddress()
