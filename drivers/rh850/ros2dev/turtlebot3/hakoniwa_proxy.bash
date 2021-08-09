#!/bin/bash
export RESOLVE_IPADDR=`cat /etc/resolv.conf | grep nameserver | awk '{print $2}'`
export CORE_PORTNO=50051

while [ 1 ]
do
        echo "############ START CONNECT #################"
        hakoniwa_proxy ./proxy_param.json ${RESOLVE_IPADDR}  ${CORE_PORTNO}
        echo "############ SERVER DOWN #################"
        sleep 1
done
