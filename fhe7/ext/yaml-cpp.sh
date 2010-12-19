#!/bin/bash

version=0.2.5
tarfile=yaml-cpp-$version.tar.gz

mkdir -p ext
cd ext

if [ ! -f $tarfile ]; then
    wget http://yaml-cpp.googlecode.com/files/$tarfile
fi

if [ ! -d yaml-cpp-$version ]; then
    tar xif $tarfile
fi

cd yaml-cpp-$version

if [ ! -f built ]; then
    cmake . -DCMAKE_INSTALL_PREFIX=../yaml-cpp && make -j 4 && make install && touch built
fi


