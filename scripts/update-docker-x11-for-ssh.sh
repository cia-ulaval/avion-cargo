
#!/usr/bin/env bash
set -euo pipefail

# Detect the forwarded SSH display.
if [[ ! "${DISPLAY:-}" =~ ^localhost:([0-9]+)(\.[0-9]+)?$ ]]; then
    echo "ERROR: Expected an SSH-forwarded display, got: ${DISPLAY:-unset}" >&2
    exit 1
fi

DISPLAY_NUMBER="${BASH_REMATCH[1]}"

# Determine Docker's bridge gateway.
GATEWAY="${1:-$(docker network inspect bridge \
    --format '{{(index .IPAM.Config 0).Gateway}}')}"

if [[ -z "$GATEWAY" ]]; then
    echo "ERROR: Could not determine Docker gateway." >&2
    exit 1
fi

AUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"

if [[ ! -r "$AUTHORITY" ]]; then
    echo "ERROR: Cannot read Xauthority: $AUTHORITY" >&2
    exit 1
fi

# Extract exactly one non-Unix authentication cookie.
COOKIE=$(xauth -f "$AUTHORITY" list | awk \
    -v display=":${DISPLAY_NUMBER}" '
    $1 !~ /\/unix:/ &&
    substr($1, length($1) - length(display) + 1) == display &&
    $2 == "MIT-MAGIC-COOKIE-1" {
        print $3
        exit
    }
')

if [[ -z "$COOKIE" ]]; then
    echo "ERROR: No X11 cookie found for display $DISPLAY_NUMBER." >&2
    exit 1
fi

# Register the cookie for Docker's gateway.
xauth -f "$AUTHORITY" add \
    "${GATEWAY}:${DISPLAY_NUMBER}" \
    MIT-MAGIC-COOKIE-1 "$COOKIE"

# Export values into the calling shell.
printf 'export DOCKER_X11_DISPLAY=%q\n' \
    "${GATEWAY}:${DISPLAY_NUMBER}.0"

printf 'export DOCKER_XAUTHORITY=%q\n' "$AUTHORITY"

echo "Docker X11 authentication updated." >&2
