#!/bin/bash

#
# template script for generating ubuntu container for LXC
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

   # Setup locales
 cat << EOF >> $rootfs/etc/environment
LANGUAGE=en_US.UTF-8
LANG=en_US.UTF-8
LC_ALL=en_US.UTF-8
LC_CTYPE=C
EOF

   # configure the network using the dhcp
    cat <<EOF > $rootfs/etc/network/interfaces
auto lo
iface lo inet loopback

auto eth0
iface eth0 inet dhcp
EOF

    if [ -e $rootfs/etc/dhcp/dhclient.conf ]; then
        sed -i "s/<hostname>/$hostname/" $rootfs/etc/dhcp/dhclient.conf
    elif [ -e $rootfs/etc/dhcp3/dhclient.conf ]; then
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
    packages=dialog,apt,apt-utils,resolvconf,iproute,inetutils-ping,net-tools,isc-dhcp-client,ssh,language-pack-en,build-essential,python-setuptools,curl,sudo

    cache=$1
    arch=$2
    baserel=$3

    echo "Setting optimum mirror from host machine."
    mirror=`cat /etc/apt/sources.list | grep '^deb ' | head -1 | tr " " "\n" | head -2 | tail -1`

    # check the mini ubuntu was not already downloaded
    mkdir -p "$cache/partial-$arch"
    if [ $? -ne 0 ]; then
        echo "Failed to create '$cache/partial-$arch' directory"
        return 1
    fi

    # download a mini ubuntu into a cache
    echo "Downloading ubuntu minimal ..."
    # Local
    echo "debootstrap --verbose --variant=minbase --components=main,universe --arch=$arch --include=$packages $baserel $cache/partial-$arch $mirror"
    debootstrap --verbose --variant=minbase --components=main,universe --arch=$arch --include=$packages $baserel $cache/partial-$arch $mirror

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
    rootfs=$1
    baserel=$2
    cache="/var/cache/lxc/$baserel"
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
            download_ubuntu $cache $arch $baserel
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
    baserel=$1
    cache="/var/cache/lxc/$baserel"
    mkdir -p /var/lock/subsys/

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
$1 -h|--help -p|--baserel=<precise|quantal|raring> --path=<path> --rosrel=<fuerte|groovy|hydro> --clean
EOF
    return 0
}

options=$(getopt -o hp:n:c -l help,path:,rosrel:,baserel:,name:,clean -- "$@")
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
        -r|--rosrel)    rosrel=$2; shift 2;;
        -b|--baserel)   baserel=$2; shift 2;;
        -n|--name)      name=$2; shift 2;;
        -c|--clean)     clean=$2; shift 2;;
        --)             shift 1; break ;;
        *)              break ;;
    esac
done

if [ -z "$baserel" ]; then
    echo "'baserel' parameter is required"
    exit 1
fi

if [ ! -z "$clean" -a -z "$path" ]; then
    clean $baserel || exit 1
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

if [ -z "$rosrel" ]; then
    echo "'rosrel' parameter is required"
    exit 1
fi

if [ "$(id -u)" != "0" ]; then
    echo "This script should be run as 'root'"
    exit 1
fi

mkdir -p $path

rootfs=$path/rootfs

install_ubuntu $rootfs $baserel
if [ $? -ne 0 ]; then
    echo "failed to install ubuntu"
    exit 1
fi

configure_ubuntu $rootfs $name
if [ $? -ne 0 ]; then
    echo "failed to configure ubuntu for a container"
    exit 1
fi

echo "Provisioning other required directories and files"
# basic source and conf files

mkdir -p $path/config
mkdir -p $path/data

mkdir -p $rootfs/opt/rce
mkdir -p $rootfs/opt/rce/packages

touch $rootfs/etc/init/rceComm.conf
touch $rootfs/etc/init/rceLauncher.conf
touch $rootfs/etc/init/rceRosapi.conf

#The rest of the provisioning.
SOURCE="${BASH_SOURCE[0]}"

while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
    DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
    SOURCE="$(readlink "$SOURCE")"
    [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done

DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"

cp $DIR/setup.sh $rootfs/opt/rce/setup.sh
sed -i "s/rosrel/$rosrel/g" $rootfs/opt/rce/setup.sh

cp $DIR/rce.conf $rootfs/etc/init/rce.conf
sed -i "s/rosrel/$rosrel/g" $rootfs/etc/init/rce.conf
