#!/bin/bash

# Remove comments, newlines and multiple spaces. Store result in new file.
echo 'Minifying main file ...'
grep -o '^[^/]*' OmNomNomFull.java | tr -d '\n' | tr -s ' ' > OmNomNom.java

read -n1 -r -p "Press space to continue..." key

echo 'Compiling ...'
nxjc OmNomNom.java

read -n1 -r -p "Press space to continue..." key

echo 'Linking ...'
nxjlink -v -o OmNomNom.nxj OmNomNom

read -n1 -r -p "Press space to continue..." key

echo 'Uploading ...'
nxjupload OmNomNom.nxj