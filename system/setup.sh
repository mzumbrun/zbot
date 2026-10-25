#!/usr/bin/env bash
# Per-machine setup. Run on every laptop and Pi after cloning the repo. Safe to re-run.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "== udev rules"
for f in "$HERE"/udev/*.rules; do
  sudo install -m 0644 "$f" /etc/udev/rules.d/
  echo "   installed $(basename "$f")"
done
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "== dialout group"
if id -nG "$USER" | grep -qw dialout; then
  echo "   $USER is in dialout"
else
  sudo usermod -aG dialout "$USER"
  echo "   added $USER to dialout: log out and back in"
fi

echo "== brltty"
if dpkg -s brltty >/dev/null 2>&1; then
  echo "   WARNING: brltty is installed and grabs USB-serial boards. Remove with: sudo apt remove brltty"
else
  echo "   not installed (good)"
fi

echo "== devices"
ls -l /dev/so101* 2>/dev/null || echo "   no /dev/so101* links yet (plug in a board)"
