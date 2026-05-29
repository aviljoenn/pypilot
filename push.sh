#!/usr/bin/env bash
# push.sh — Linux/macOS equivalent of push.bat
# Stage all changes, commit with the given message, and push.
set -uo pipefail

# Require a commit message.
if [ "$#" -eq 0 ] || [ -z "${1:-}" ]; then
    echo "Usage: $(basename "$0") \"Your commit message\""
    echo "Example: $(basename "$0") \"fix: prevent false motor activation in HAND mode\""
    exit 1
fi

# Build the commit message from all arguments (mirrors the .bat behavior).
MSG="$*"

# Make sure we are inside a Git repo and move to the repo root.
REPO_ROOT="$(git rev-parse --show-toplevel 2>/dev/null)"
if [ -z "$REPO_ROOT" ]; then
    echo "Error: This folder is not inside a Git repository."
    exit 1
fi

cd "$REPO_ROOT" || exit 1

echo
echo "Repo root: $REPO_ROOT"
echo
echo "Current changes:"
git status --short

# Check whether there is anything to do at all.
if [ -z "$(git status --porcelain)" ]; then
    echo
    echo "Nothing to commit."
    exit 0
fi

echo
echo "Staging all changes..."
git add -A || { echo; echo "Failed."; exit 1; }

echo
echo "Staged changes:"
git status --short

echo
echo "Committing..."
git commit -m "$MSG" || { echo; echo "Failed."; exit 1; }

echo
echo "Pushing..."
if ! git push; then
    echo "Standard push failed. Trying first-push upstream setup..."
    if ! git push -u origin HEAD; then
        echo
        echo "Failed."
        exit 1
    fi
fi

echo
echo "Done."
exit 0
