#!/bin/bash

version=2.1.2
tarfile=Box2D_v$version.zip

mkdir -p ext
cd ext

if [ ! -f $tarfile ]; then
    wget http://box2d.googlecode.com/files/$tarfile
fi

if [ ! -d Box2D_v$version ]; then
    unzip $tarfile
fi

cd Box2D_v$version/Box2D

if [ ! -f built ]; then
    cmake . -DCMAKE_INSTALL_PREFIX=../../box2d && make -j 4 && make install && touch built
fi
