#!/bin/bash

# Remove comments, newlines and multiple spaces. Store result in new file.
grep -o '^[^/]*' OmNomNomFull.java | tr -d '\n' | tr -s ' ' > OmNomNom.java
nxjc OmNomNom.java
nxjlink -v -o OmNomNom.nxj OmNomNom