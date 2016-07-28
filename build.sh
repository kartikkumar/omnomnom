#!/bin/bash

# Remove comments, newlines and multiple spaces. Store result in new file.
grep -o '^[^/]*' OnNomNomFull.java | tr -d '\n' | tr -s ' ' > OnNomNom.java
nxjc OnNomNom.java
# nxjlink -v -o OnNomNom.nxj OnNomNom