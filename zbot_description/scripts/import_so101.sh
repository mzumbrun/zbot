#!/usr/bin/env bash
# Import the SO-101 follower URDF + meshes from TheRobotStudio/SO-ARM100 (Apache-2.0)
# into zbot_description. Re-run to update; records the upstream commit it came from.
# Usage (from anywhere):  ~/zbot_ws/src/zbot/zbot_description/scripts/import_so101.sh
set -euo pipefail
PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT

git clone -q --depth 1 https://github.com/TheRobotStudio/SO-ARM100.git "$TMP/so-arm"
SRC="$TMP/so-arm/Simulation/SO101"
REV="$(git -C "$TMP/so-arm" rev-parse --short HEAD)"

mkdir -p "$PKG/urdf" "$PKG/meshes/so101"
# copy only the meshes the follower URDF actually uses
grep -o 'assets/[^"]*\.stl' "$SRC/so101_new_calib.urdf" | sort -u | while read -r m; do
  cp "$SRC/$m" "$PKG/meshes/so101/"
done
# relative mesh paths -> package:// so RViz can find them
sed 's|filename="assets/|filename="package://zbot_description/meshes/so101/|' \
  "$SRC/so101_new_calib.urdf" > "$PKG/urdf/so101.urdf"
cp "$TMP/so-arm/LICENSE" "$PKG/meshes/so101/LICENSE"
echo "TheRobotStudio/SO-ARM100 @ $REV, Simulation/SO101/so101_new_calib.urdf (Apache-2.0)" \
  > "$PKG/meshes/so101/SOURCE.txt"

echo "Imported SO-101 URDF + $(ls "$PKG"/meshes/so101/*.stl | wc -l) meshes from SO-ARM100 @ $REV"
