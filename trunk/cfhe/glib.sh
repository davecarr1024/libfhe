#!/bin/bash

version=2.26
tarfile=glib-$version.0.tar.bz2

mkdir -p .glib
cd .glib

if [ ! -f $tarfile ]; then
    wget http://ftp.gnome.org/pub/gnome/sources/glib/$version/$tarfile
fi

if [ ! -d glib-$version.0 ]; then
    tar xjf $tarfile
fi

installpath=$(pwd)

cd glib-$version.0

if [ ! -f built ]; then
    ./configure --prefix=$installpath && make -j 4 && make install && touch built
fi
