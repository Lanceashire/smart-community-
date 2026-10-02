#!/usr/bin/env bash
# Run on the host. Restore a stale display bind without printing X11 cookies.
set -euo pipefail
container_name="${1:-aic-development}"
display_source="$(docker inspect "$container_name" --format '{{range .Mounts}}{{if eq .Destination "/home/ubuntu/.Xauthority"}}{{.Source}}{{end}}{{end}}')"
if [[ -n "$display_source" && "$display_source" != "${XAUTHORITY:-}" ]]; then
  current_auth="${XAUTHORITY:?Current desktop session has no XAUTHORITY}"
  [[ -f "$current_auth" && -d "$(dirname "$display_source")" ]] || { echo 'Display authentication path unavailable'; exit 1; }
  # Refuse unexpected destinations; the known temporary mount belongs to this user.
  [[ "$display_source" == "/run/user/$(id -u)/xauth_"* ]] || { echo 'Unexpected authentication bind path'; exit 1; }
  [[ ! -L "$display_source" ]] || { echo 'Authentication destination must not be a symlink'; exit 1; }
  if [[ ! -f "$display_source" ]] || ! cmp -s "$current_auth" "$display_source"; then
    cp --preserve=mode "$current_auth" "$display_source"
  fi
fi
docker start "$container_name"
