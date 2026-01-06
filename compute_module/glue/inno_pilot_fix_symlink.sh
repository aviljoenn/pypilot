#!/bin/bash
set -e

BYID="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
REAL="${BYID}.real"
PTY="/dev/ttyINNOPILOT"

# If the original by-id symlink exists and the .real backup does not,
# move the original to .real once.
if [ -L "$BYID" ] && [ ! -L "$REAL" ]; then
    TARGET=$(readlink "$BYID")
    echo "Inno-Pilot: moving $BYID -> $REAL (target $TARGET)"
    mv "$BYID" "$REAL"
fi

# Now (re)point BYID to the PTY if it exists
if [ -e "$PTY" ]; then
    echo "Inno-Pilot: linking $BYID -> $PTY"
    ln -sf "$PTY" "$BYID"
else
    echo "Inno-Pilot: PTY $PTY not present; skipping link"
fi
