#!/bin/bash
while true
do
	modbus_client -mrtu -b115200 -pnone -s1 /dev/ttyACM0 -t0x04 -r999 -c13
done
