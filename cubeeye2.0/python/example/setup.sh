#!/bin/bash 

LIB_PATH=../..

export LD_LIBRARY_PATH=${LIB_PATH}/lib:${LIB_PATH}/thirdparty/libusb/lib:${LIB_PATH}/thirdparty/liblive555/lib/Release:${LIB_PATH}/thirdparty/libffmpeg/lib:${LIB_PATH}/thirdparty/libopencv/lib
