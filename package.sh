#!/bin/bash

set -e

lsb_release -a

filename=".robotchain/config.robotchain"
if [ ! -f "$filename" ]; then
    exit 0
fi

type=$(jq -r ".type" "$filename")
title=$(jq -r ".title" "$filename")
version=$(jq -r ".version" "$filename")
if [ -z "$type" ] || [ -z "$title" ] || [ -z "$version" ]; then
  exit 1
fi

if [ ! -d "debian" ]; then
    mkdir -p debian
    mkdir -p debian/DEBIAN
    mkdir -p debian/opt/robotchain/manager/board/"$title"
else
    sudo rm -rf debian
    mkdir -p debian
    mkdir -p debian/DEBIAN
    mkdir -p debian/opt/robotchain/manager/board/"$title"
fi

echo -e "\033[32mStarting to build the deb software package for you...\033[0m"

sudo cp -r .robotchain debian/opt/robotchain/manager/board/"$title"/
sudo cp -r block debian/opt/robotchain/manager/board/"$title"/
sudo cp -r doc debian/opt/robotchain/manager/board/"$title"/
sudo cp -r hardware debian/opt/robotchain/manager/board/"$title"/
sudo cp -r software debian/opt/robotchain/manager/board/"$title"/
sudo cp -r keil debian/opt/robotchain/manager/board/"$title"/
sudo cp .gitignore debian/opt/robotchain/manager/board/"$title"/
sudo cp readme.md debian/opt/robotchain/manager/board/"$title"/

echo -e "\033[32mPublishing the deb software package to the software repository server for you...\033[0m"

sudo touch debian/DEBIAN/control && sudo chmod +x debian/DEBIAN/control
standard_package_name=$(echo "$title" | tr "_" "-")
standard_package_name=$(echo "$standard_package_name" | tr '[:upper:]' '[:lower:]')
name_str="Package: $standard_package_name"
export name_str
sudo -E sh -c 'echo $name_str >> debian/DEBIAN/control'
version_str="Version: $version"
export version_str
sudo -E sh -c 'echo $version_str >> debian/DEBIAN/control'
sudo sh -c 'echo "Maintainer: GEEKROS <admin@geekros.com>" >> debian/DEBIAN/control'
sudo sh -c 'echo "Homepage: https://www.geekros.com" >> debian/DEBIAN/control'
architecture_str="Architecture: all"
export architecture_str
sudo -E sh -c 'echo $architecture_str >> debian/DEBIAN/control'
sudo sh -c 'echo "Installed-Size: 5000" >> debian/DEBIAN/control'
sudo sh -c 'echo "Section: utils" >> debian/DEBIAN/control'
sudo sh -c 'echo "Description: robotchain and robot" >> debian/DEBIAN/control'

sudo dpkg --build debian/ && dpkg-name debian.deb

# shellcheck disable=SC2035
sudo rm -rf debian && robotchain package publish && sudo rm -rf *.deb