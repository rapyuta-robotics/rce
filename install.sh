#!/bin/sh

install_pkg() {
    title="Install package $1"
    
    printf "%s\n" "$title"
    for i in $(seq 1 ${#title})
    do 
        printf "-"
    done
    printf "\n\n"
    
    cd $1
    rm -rf build dist *.egg-info
    python setup.py develop
    cd ..
    
    printf "\n\n"
}

install_pkg rce-util
install_pkg rce-comm
install_pkg rce-core
install_pkg rce-client
install_pkg rce-console
