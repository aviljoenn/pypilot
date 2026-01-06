#!/bin/bash
set -e

BYID="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
REAL="${BYID}.real"
PTY="/dev/ttyINNOPILOT"

MAX_TRIES=30
SLEEP_SEC=1

echo "Inno-Pilot: fix_symlink starting, waiting for BYID and PTY..."

# Wait for both the BYID symlink and the PTY to exist
for i in $(seq 1 $MAX_TRIES); do
    HAVE_BYID=false
    HAVE_PTY=false

    if [ -L "$BYID" ]; then
        HAVE_BYID=true
    fi

    if [ -e "$PTY" ]; then
        HAVE_PTY=true
    fi

    if $HAVE_BYID && $HAVE_PTY; then
        echo "Inno-Pilot: found BYID=$BYID and PTY=$PTY after $i attempt(s)"
        break
    fi

    echo "Inno-Pilot: waiting for BYID=$HAVE_BYID PTY=$HAVE_PTY ($i/$MAX_TRIES)..."
    sleep "$SLEEP_SEC"
done

# If still missing, bail out quietly
if [ ! -L "$BYID" ] || [ ! -e "$PTY" ]; then
    echo "Inno-Pilot: BYID or PTY not ready after ${MAX_TRIES}s, skipping link"
    exit 0
fi

# If .real does not exist yet, move the original BYID link aside
if [ ! -L "$REAL" ]; then
    TARGET=$(readlink "$BYID")
    echo "Inno-Pilot: moving $BYID -> $REAL (target $TARGET)"
    mv "$BYID" "$REAL"
fi

# Now point BYID at the PTY
echo "Inno-Pilot: linking $BYID -> $PTY"
ln -sf "$PTY" "$BYID"
