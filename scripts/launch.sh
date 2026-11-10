#!/usr/bin/env bash
# Launch a version of the AEGIS-V1 simulator in the browser (see VERSIONS.md).
#   bash scripts/launch.sh            latest version (highest vX.Y.Z tag)
#   bash scripts/launch.sh latest     same
#   bash scripts/launch.sh v1.0.0     a specific version
#   bash scripts/launch.sh list       list versions, newest first
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
PORT=5173
URL="http://127.0.0.1:$PORT/"
VERSIONS_DIR="$HOME/.hunch22-versions"   # worktrees for older versions (outside iCloud)
LOG_DIR="$VERSIONS_DIR/logs"
cd "$REPO"

tags() { git tag -l 'v[0-9]*' --sort=-v:refname; }

want="${1:-latest}"
if [ "$want" = "list" ]; then
  tags | while read -r t; do echo "$t  $(git log -1 --format='%ad  %s' --date=short "$t")"; done
  exit 0
fi
if [ "$want" = "latest" ]; then want="$(tags | head -1)"; fi
if [ -z "$want" ] || ! git rev-parse -q --verify "refs/tags/$want" >/dev/null; then
  echo "No such version: '${1:-latest}'. Available:"; tags; exit 1
fi

tagCommit="$(git rev-list -n 1 "$want")"
if [ "$tagCommit" = "$(git rev-parse HEAD)" ]; then
  dir="$REPO"
  if [ -n "$(git status --porcelain --untracked-files=no)" ]; then
    echo "note: running $want from $REPO, which has uncommitted changes on top of it"
  fi
else
  dir="$VERSIONS_DIR/$want"
  if [ ! -d "$dir" ]; then
    mkdir -p "$VERSIONS_DIR"
    git worktree add --detach "$dir" "$want" >/dev/null
  fi
fi

# dependencies: install if missing, or if iCloud sync mangled the esbuild binary ("esbuild 3" etc.)
if [ ! -x "$dir/node_modules/.bin/vite" ] || [ ! -e "$dir/node_modules/@esbuild/darwin-arm64/bin/esbuild" ]; then
  echo "installing dependencies in $dir ..."
  (cd "$dir" && npm ci --no-audit --no-fund)
fi

# stop whatever is serving the port (an older launch of this or another version)
pids="$(lsof -ti tcp:$PORT -sTCP:LISTEN || true)"
if [ -n "$pids" ]; then kill $pids; sleep 1; fi

mkdir -p "$LOG_DIR"
log="$LOG_DIR/$want.log"
(cd "$dir" && nohup ./node_modules/.bin/vite --host 127.0.0.1 --port $PORT --strictPort >"$log" 2>&1 &)

for _ in $(seq 1 60); do
  if curl -s -o /dev/null -m 2 "$URL"; then
    echo "AEGIS-V1 $want running at $URL (from $dir, log $log)"
    open "$URL"
    exit 0
  fi
  sleep 1
done
echo "server did not come up; last lines of $log:"; tail -20 "$log"; exit 1
