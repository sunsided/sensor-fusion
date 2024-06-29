#!/usr/bin/env bash
set -euo pipefail

# Find all PNG files recursively in the current directory
find . -type f -name "*.png" | while IFS= read -r file; do
    echo "Optimizing: $file"
    optipng $@ "$file"
done

