#!/bin/bash

#./ip.sh 网卡 网络类型 新IP 新子网 新网关
#sudo ./ip.sh eth0 static 192.168.1.3 255.255.55.0 127.0.0.1

DATE=`date +%Y%m%d%H%M`

interface=$1
type=$2
new_ipaddr=$3
new_netmask=$4
new_gateway=$5
interface_dir="/etc/network/interfaces.d"
#scan_range='/^/, $'

#-----------------------------------------
have_interface=0
for ifst in `ls /sys/class/net/|grep -v 'lo'`
do
    if [ ${interface} == ${ifst} ];then
        have_interface=1
        break
    fi 
done

if [ ${have_interface} -eq 0 ];then
    echo -e "\033[33mCannot find ${interface} Network Card！\033[0m"
    exit -1
fi
#-----------------------------------------

#-----------------------------------------
have_interface_config=0
for ifd in `ls ${interface_dir}`
do
    if [ ${interface} == ${ifd} ];then
        have_interface_config=1
        break
    fi 
done

if [ ${have_interface_config} -eq 0 ];then
    echo -e "\033[33mCannot find ${interface} Config File！\033[0m"
    exit -1
fi
#-----------------------------------------

#-----------------------------------------
ipaddr_index="address"
netmask_index="netmask"
gateway_index="gateway"
macaddr_index="hw ether"
static_index="iface ${interface} inet"

dhcp_or_static=`grep "^${static_index}" ${interface_dir}/${interface} |sed -n '1p' |awk '{print $4}'`
old_ipaddr=$(sed -n "s/^${ipaddr_index}/&/p" ${interface_dir}/${interface} |awk '{print $2}')
old_netmask=$(sed -n "s/^${netmask_index}/&/p" ${interface_dir}/${interface} |awk '{print $2}')
old_gateway=$(sed -n "s/^${gateway_index}/&/p" ${interface_dir}/${interface} |awk '{print $2}')
#old_macaddr=$(sed -n "s/.*${macaddr_index}.*/&/p" ${interface_dir}/${interface} |awk '{print $6}')

echo dhcp_or_static = ${dhcp_or_static}
echo old_ipaddr = ${old_ipaddr}
echo old_netmask = ${old_netmask}
echo old_gateway = ${old_gateway}
#echo old_macaddr = ${old_macaddr}
#-----------------------------------------

#-----------------------------------------
if [ ${type} == static ];then
    echo ${new_ipaddr}|grep -iwE '^(([1-9]?[0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])\.){3}([1-9]?[0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])$' >/dev/null 2>&1
    if [ $? -ne 0 ] ;then
        echo -e "\033[33mIP (${new_ipaddr}) define incorrect!\033[0m"
        exit -1
    fi

    ping -c 2 "${new_ipaddr}" |grep "ttl" >/dev/null 2>&1
    if [ $? -eq 0 ] ;then
        echo -e "\033[33mIP (${new_ipaddr}) conflict!\033[0m"
		exit -1
	fi

    echo ${new_netmask}|grep -iwE '^(([1-9]?[0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])\.){3}([1-9]?[0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])$' >/dev/null 2>&1
    if [ $? -ne 0 ] ;then
        echo -e "\033[33mNetmask (${new_netmask}) define incorrect!\033[0m"
        exit -1
    fi

    echo ${new_gateway}|grep -iwE '^(([1-9]?[0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])\.){3}([1-9]?[0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])$' >/dev/null 2>&1
    if [ $? -ne 0 ] ;then
        echo -e "\033[33mGateway (${new_gateway}) define incorrect!\033[0m"
        exit -1
    fi

    ping -c 2 "${new_gateway}"|grep -iE "ttl" >/dev/null 2>&1
    if [ $? -ne 0 ] ;then
        echo -e "\033[33mGateway (${new_gateway}) cannot access!\033[0m"
        exit -1
    fi

    if [ ${dhcp_or_static} == dhcp ];then
        sed -i "/^${static_index} /s/${dhcp_or_static}/${type}/" ${interface_dir}/${interface}

        sed -i "/${static_index}/a ${ipaddr_index} ${new_ipaddr}" ${interface_dir}/${interface}
        old_ipaddr=$(sed -n "s/^${ipaddr_index}/&/p" ${interface_dir}/${interface} |awk '{print $2}')
        if [ ${old_ipaddr} != ${new_ipaddr} ];then
            echo -e "\033[33mSet ${ipaddr_index} Failed!\033[0m"
            exit -1
        fi

        sed -i "/${ipaddr_index}/a ${netmask_index} ${new_netmask}" ${interface_dir}/${interface}
        old_netmask=$(sed -n "s/^${netmask_index}/&/p" ${interface_dir}/${interface} |awk '{print $2}')
        if [ ${old_netmask} != ${new_netmask} ];then
            echo -e "\033[33mSet ${netmask_index} Failed!\033[0m"
            exit -1
        fi

        sed -i "/${netmask_index}/a ${gateway_index} ${new_gateway}" ${interface_dir}/${interface}
        old_gateway=$(sed -n "s/^${gateway_index}/&/p" ${interface_dir}/${interface} |awk '{print $2}')
        if [ ${old_gateway} != ${new_gateway} ];then
            echo -e "\033[33mSet ${gateway_index} Failed!\033[0m"
            exit -1
        fi

    elif [ ${dhcp_or_static} == static ];then
        sed -i "/^${static_index} /s/${dhcp_or_static}/${type}/" ${interface_dir}/${interface}

        sed -i "s/${old_ipaddr}/${new_ipaddr}/" ${interface_dir}/${interface}
        old_ipaddr=$(sed -n "s/^${ipaddr_index}/&/p" ${interface_dir}/${interface} |awk '{print $2}')
        if [ ${old_ipaddr} != ${new_ipaddr} ];then
            echo -e "\033[33mSet ${ipaddr_index} Failed!\033[0m"
            exit -1
        fi

        sed -i "s/${old_netmask}/${new_netmask}/" ${interface_dir}/${interface}
        old_netmask=$(sed -n "s/^${netmask_index}/&/p" ${interface_dir}/${interface} |awk '{print $2}')
        if [ ${old_netmask} != ${new_netmask} ];then
            echo -e "\033[33mSet ${netmask_index} Failed!\033[0m"
            exit -1
        fi

        sed -i "s/${old_gateway}/${new_gateway}/" ${interface_dir}/${interface}
        old_gateway=$(sed -n "s/^${gateway_index}/&/p" ${interface_dir}/${interface} |awk '{print $2}')
        if [ ${old_gateway} != ${new_gateway} ];then
            echo -e "\033[33mSet ${gateway_index} Failed!\033[0m"
            exit -1
        fi
    fi
fi
#-----------------------------------------

#-----------------------------------------
if [[ ${type} == dhcp ]];then
    if [ ${dhcp_or_static} == dhcp ];then
        sed -i "/^${static_index} /s/${dhcp_or_static}/${type}/" ${interface_dir}/${interface}

    elif [ ${dhcp_or_static} == static ];then
        sed -i "/^${static_index} /s/${dhcp_or_static}/${type}/" ${interface_dir}/${interface}

        sed -i "/${ipaddr_index}/d" ${interface_dir}/${interface}
        sed -i "/${netmask_index}/d" ${interface_dir}/${interface}
        sed -i "/${gateway_index}/d" ${interface_dir}/${interface}
    fi
fi
#-----------------------------------------

echo ""
cat ${interface_dir}/${interface}

echo -e "\033[32m${interface} IP Change succeed, network rebooting......\033[0m"
#ip addr flush dev ${interface}
systemctl restart networking.service
