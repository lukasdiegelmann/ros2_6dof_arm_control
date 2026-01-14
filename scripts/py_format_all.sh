#!/usr/bin/env bash
set -euo pipefail

# Imports zuerst, dann black
isort .
black .
