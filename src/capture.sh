echo "Capture test!!!!!"
echo "shell0:$0"
echo "width:$1"
echo "name:$2"
echo "dev:$3"
echo $PWD
rm -rf *.jpg
#rm -rf *.yuv
./capture -d /dev/video2 -d /dev/$3 -i $2 -w $1 -m