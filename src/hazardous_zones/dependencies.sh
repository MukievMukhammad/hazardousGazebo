#!/bin/bash

# Qt 5.12.3 Bionic release
QT_FOLDER=/opt/qt512
if [ -d "$QT_FOLDER" ]; then
    echo "QT already exists"
else
    echo "QT does not exist"
    echo "creating..."
    yes Y | sudo add-apt-repository ppa:beineri/opt-qt-5.12.3-bionic
    sudo apt-get update
    yes Y | sudo apt-get install qt512-meta-minimal
    mkdir ~/.config/qtchooser
    touch ~/.config/qtchooser/default.conf
    printf "/opt/qt512/bin\n/opt/qt512/lib" >> ~/.config/qtchooser/default.conf
fi