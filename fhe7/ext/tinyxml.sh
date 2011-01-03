#!/bin/bash

tarfile=tinyxml_2_6_1.tar.gz
dl=http://downloads.sourceforge.net/project/tinyxml/tinyxml/2.6.1/tinyxml_2_6_1.tar.gz

mkdir -p ext
cd ext

if [ ! -f $tarfile ]; then
    wget $dl
fi

if [ ! -d tinyxml ]; then
    tar xif $tarfile
fi
