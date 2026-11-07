#!/usr/bin/env sh
# Fetches the Betaflight Blackbox Explorer source at a pinned commit (GPL-3.0).
set -e
COMMIT=e179902df4e38f24d4b61f114bf856a196c995be
DIR="$(dirname "$0")/../vendor/blackbox-log-viewer"
rm -rf "$DIR"
git clone --quiet https://github.com/betaflight/blackbox-log-viewer.git "$DIR"
git -C "$DIR" checkout --quiet "$COMMIT"
echo "Parser source ready at $DIR ($COMMIT)"
