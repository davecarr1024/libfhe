#!/bin/bash

version=1.5.0
tarfile=gtest-$version.tar.bz2

mkdir -p ext
cd ext
if [ ! -f $tarfile ]; then
    wget http://googletest.googlecode.com/files/$tarfile
fi

if [ ! -d gtest-$version ]; then
    tar xjf $tarfile
fi

install_path=$(pwd)/gtest

cd gtest-$version

if [ ! -f built ]; then
    ./configure --prefix=$install_path && make -j 4 && make install && touch built
fi
