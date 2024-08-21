# Use zsh for shell commands
SHELL := /bin/zsh

# Read version from file and compute parts
VERSION := $(shell sed -n '1p' version.txt)
MAJOR_MINOR_VERSION := $(shell echo $(VERSION) | cut -d '.' -f 1,2)
PATCH_VERSION := $(shell echo $(VERSION) | cut -d '.' -f 3)

# Calculate NEW_VERSION
NEW_VERSION := $(shell echo $$(($(PATCH_VERSION) + 1)))

.PHONY: inc_version
# Define a target for incrementing the version
inc_version:
	# Print the version and patch components
	echo "VERSION: $(VERSION)"; \
	echo "MAJOR_MINOR_VERSION: $(MAJOR_MINOR_VERSION)"; \
	echo "PATCH_VERSION: $(PATCH_VERSION)"; \
	# Print the result of the arithmetic operation
	echo "Calculated NEW_VERSION: $(NEW_VERSION)"; \
	# Print the final results
	echo "The original version number is $(VERSION)"; \
	echo "The PATCH number is $(PATCH_VERSION)"; \
	echo "The new PATCH number is $(NEW_VERSION)"; \
	echo "The new version number is $(MAJOR_MINOR_VERSION).$(NEW_VERSION)"
