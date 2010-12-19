#!/bin/bash

version=2.77
tarfile=bullet-$version.tgz

mkdir -p ext
cd ext

if [ ! -f $tarfile ]; then
    wget http://bullet.googlecode.com/files/$tarfile
fi

if [ ! -d bullet-$version ]; then
    tar xif $tarfile
fi

installpath=$(pwd)/bullet

cd bullet-$version

if [ ! -f built ]; then
    ./autogen.sh && ./configure --prefix=$installpath && make -j 4 && make install && touch built
fi
