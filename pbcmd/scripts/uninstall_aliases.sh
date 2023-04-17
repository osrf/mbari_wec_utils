#!/bin/bash

pushd ~/.local/bin
aliases=$(ls -1 | xargs -I{} sh -c "if [ -L {} ]; then echo {}; fi")
echo "Removing aliases:" $aliases
rm $aliases
popd
