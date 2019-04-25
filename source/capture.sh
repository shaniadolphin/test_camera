#echo "Capture test!!!!!"
#echo "$0"
echo "width:$1"
echo "name:$2"
#echo "dev:$3"
#echo $PWD
#rm -rf *.jpg
#rm -rf *.yuv
./capture -d /dev/$3 -i $2 -w $1 -m