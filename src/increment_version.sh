#!/bin/bash

# Read version from file
VERSION=$(sed -n '1p' version.txt)

# Extract parts of the version
MAJOR_MINOR_VERSION=$(echo $VERSION | cut -d '.' -f 1,2)
PATCH_VERSION=$(echo $VERSION | cut -d '.' -f 3)

# Calculate the new patch version
NEW_PATCH_VERSION=$((PATCH_VERSION + 1))

# Print the results
echo "The original version number is $VERSION"
echo "MAJOR_MINOR_VERSION: $MAJOR_MINOR_VERSION"
echo "The PATCH number is $PATCH_VERSION"
echo "The new PATCH number is $NEW_PATCH_VERSION"
echo "The new version number is $MAJOR_MINOR_VERSION.$NEW_PATCH_VERSION"

# Optionally, write the new version back to the file
echo "$MAJOR_MINOR_VERSION.$NEW_PATCH_VERSION" > version.txt
