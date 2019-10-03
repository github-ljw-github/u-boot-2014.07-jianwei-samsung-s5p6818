
echo $1 $2

[ -b $1 ] || { echo "$1 is not a valid block device"; exit 1; }
[ X"$1" = X"${1%%[0-9]}" ] || { echo "$1 is a partition, please use device, perhaps ${dev%%[0-9]}"; exit 1; }
[ -f $2 ] || { echo "${xboot} is not a bootloader binary file."; exit 1; }


dev="$1"
bin="$2"
sudo dd if=$bin of=$dev bs=512 seek=1 conv=sync
sync;
sync;
sync;

umount /dev/sdb
umount /dev/sdb1
sync
