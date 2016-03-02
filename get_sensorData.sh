#!/bin/sh

# Constantes
HOST=192.168.1.1
LOGIN=Charles
PASS=a

cd /home/charles/Documents/ARDrone_RISLER/
ftp -in <<EOF
open $HOST 
user $LOGIN $PASS
get sensorData.txt
exit
EOF
echo 'ftp connection done'




