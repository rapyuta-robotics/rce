#!/bin/bash

#
# template script for generating ubuntu oneiric container for LXC
#
# This script is based on lxc-debian (Daniel Lezcano <daniel.lezcano@free.fr>)
#

# Copyright ¬© 2010 Wilhelm Meier
# Author: Wilhelm Meier <wilhelm.meier@fh-kl.de>
#
# With bugfixes and modifications for EC2 support by 
# Daniil Kulchenko (daniil@kulchenko.com), Copyright ¬© 2011.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2, as
# published by the Free Software Foundation.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#

configure_ubuntu()
{
    rootfs=$1
    hostname=$2

   # configure the network using the dhcp
    cat <<EOF > $rootfs/etc/network/interfaces
auto lo
iface lo inet loopback

auto eth0
iface eth0 inet dhcp
EOF

    if [ -e $rootfs/etc/dhcp/dhclient.conf ] 
    then
	sed -i "s/<hostname>/$hostname/" $rootfs/etc/dhcp/dhclient.conf
    elif [ -e $rootfs/etc/dhcp3/dhclient.conf ] 
    then
	sed -i "s/<hostname>/$hostname/" $rootfs/etc/dhcp3/dhclient.conf
    fi

    # set the hostname
    cat <<EOF > $rootfs/etc/hostname
$hostname
EOF
    # set minimal hosts
    cat <<EOF > $rootfs/etc/hosts
127.0.0.1 localhost $hostname
EOF

    cp $rootfs/etc/init/tty1.conf $rootfs/etc/init/console.conf
    rm $rootfs/etc/init/tty*
    sed -i 's/tty1/console/' $rootfs/etc/init/console.conf

    echo

    echo "Please change the root password! (currently, the password for root is root)"
    echo "root:root" | chroot $rootfs chpasswd

    return 0
}

download_ubuntu()
{
    packages=dialog,apt,apt-utils,resolvconf,iproute,inetutils-ping,net-tools,vim,dhcp3-client,ssh,lsb-release,language-pack-en,wget,python-twisted-core,python-openssl,python-imaging,build-essential
	
	### Local
	#mirror=
	
	### Rackspace
	mirror=http://mirror.rackspace.com/ubuntu/
	
	### Amacon EC2
	#mirror=http://us-east-1.ec2.archive.ubuntu.com/ubuntu/
	
	
    cache=$1
    arch=$2

    # check the mini ubuntu was not already downloaded
    mkdir -p "$cache/partial-$arch"
    if [ $? -ne 0 ]; then
	echo "Failed to create '$cache/partial-$arch' directory"
	return 1
    fi

    # download a mini ubuntu into a cache
    echo "Downloading ubuntu minimal ..."
    # Local
    debootstrap --verbose --variant=minbase --components=main,universe --arch=$arch --include=$packages precise $cache/partial-$arch $mirror
  
  if [ $? -ne 0 ]; then
	echo "Failed to download the rootfs, aborting."
	return 1
    fi

    mv "$1/partial-$arch" "$1/rootfs-$arch"
    echo "Download complete."

    return 0
}

copy_ubuntu()
{
    cache=$1
    arch=$2
    rootfs=$3

    # make a local copy of the miniubuntu
    echo -n "Copying rootfs to $rootfs ..."
    rm -rf $rootfs
    cp -a $cache/rootfs-$arch $rootfs || return 1
    return 0
}

install_ubuntu()
{
    cache="/var/cache/lxc/precise"
    rootfs=$1
    mkdir -p /var/lock/subsys/
    (
	flock -n -x 200
	if [ $? -ne 0 ]; then
	    echo "Cache repository is busy."
	    return 1
	fi

	arch=$(arch)
	if [ "$arch" == "x86_64" ]; then
	    arch=amd64
	fi

	if [ "$arch" == "i686" ]; then
	    arch=i386
	fi

	echo "Checking cache download in $cache/rootfs-$arch ... "
	if [ ! -e "$cache/rootfs-$arch" ]; then
	    download_ubuntu $cache $arch
	    if [ $? -ne 0 ]; then
		echo "Failed to download 'ubuntu base'"
		return 1
	    fi
	fi

	echo "Copy $cache/rootfs-$arch to $rootfs ... "
	copy_ubuntu $cache $arch $rootfs
	if [ $? -ne 0 ]; then
	    echo "Failed to copy rootfs"
	    return 1
	fi

	return 0

	) 200>/var/lock/subsys/lxc

    return $?
}

clean()
{
    cache="/var/cache/lxc/precise"

    if [ ! -e $cache ]; then
	exit 0
    fi

    # lock, so we won't purge while someone is creating a repository
    (
	flock -n -x 200
	if [ $? != 0 ]; then
	    echo "Cache repository is busy."
	    exit 1
	fi

	echo -n "Purging the download cache..."
	rm --preserve-root --one-file-system -rf $cache && echo "Done." || exit 1
	exit 0

    ) 200>/var/lock/subsys/lxc
}

usage()
{
    cat <<EOF
$1 -h|--help -p|--path=<path> --clean
EOF
    return 0
}

options=$(getopt -o hp:n:c -l help,path:,name:,clean -- "$@")
if [ $? -ne 0 ]; then
    usage $(basename $0)
    exit 1
fi
eval set -- "$options"

while true
do
    case "$1" in
	-h|--help)      usage $0 && exit 0;;
	-p|--path)      path=$2; shift 2;;
	-n|--name)      name=$2; shift 2;;
	-c|--clean)     clean=$2; shift 2;;
	--)             shift 1; break ;;
        *)              break ;;
    esac
done

if [ ! -z "$clean" -a -z "$path" ]; then
    clean || exit 1
    exit 0
fi

type debootstrap
if [ $? -ne 0 ]; then
    echo "'debootstrap' command is missing"
    exit 1
fi

if [ -z "$path" ]; then
    echo "'path' parameter is required"
    exit 1
fi

if [ "$(id -u)" != "0" ]; then
    echo "This script should be run as 'root'"
    exit 1
fi

mkdir -p $path

rootfs=$path/rootfs

install_ubuntu $rootfs
if [ $? -ne 0 ]; then
    echo "failed to install ubuntu"
    exit 1
fi

configure_ubuntu $rootfs $name
if [ $? -ne 0 ]; then
    echo "failed to configure ubuntu for a container"
    exit 1
fi

