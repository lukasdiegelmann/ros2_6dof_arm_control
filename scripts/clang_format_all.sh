#!/usr/bin/env bash
set -e

EXTENSIONS="*.cpp *.hpp *.h *.cc"

find . \
  -type f \( $(printf -- "-name %s -o " $EXTENSIONS | sed 's/ -o $//') \) \
  -not -path "./build/*" \
  -not -path "./install/*" \
  -not -path "./log/*" \
  -exec clang-format -i {} +
