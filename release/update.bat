echo off
echo ************** Step 1 *************************
echo ======== Change to Bootloader Mode 
copy /y START_BL.ACT D:
TIMEOUT  /T 6 /NOBREAK

echo ************** Step 2 ************************
echo ======== Load firmware file to U-Disk
copy /y mm32link_mini_winusb_20221130.hex D:
TIMEOUT  /T 6 /NOBREAK

echo ************** Step 3 ************************
echo ======== Configure Beep / Power
copy /y *.cfg D:

echo ************** Finish ************************
TIMEOUT  /T 4

exit