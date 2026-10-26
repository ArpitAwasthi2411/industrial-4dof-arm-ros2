#!/usr/bin/env bash
# Render README.md to docs/Project_Report.pdf (pandoc + headless Chromium).
# Re-run after adding RViz / TF tree screenshots to docs/images/.
#
# Usage: docs/build_report.sh            (from the package root)
#        CHROME=/path/to/chrome docs/build_report.sh
set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CHROME="${CHROME:-$(command -v chromium || command -v chromium-browser || command -v google-chrome || true)}"
if [[ -z "${CHROME}" ]]; then
  echo "Chromium/Chrome not found; set CHROME=/path/to/chrome" >&2
  exit 1
fi

HTML="$(mktemp --suffix=.html -p "${PKG_DIR}/docs")"
trap 'rm -f "${HTML}"' EXIT

pandoc "${PKG_DIR}/README.md" \
  --from gfm --to html5 --standalone --embed-resources --no-highlight \
  --resource-path "${PKG_DIR}" \
  --css "${PKG_DIR}/docs/report.css" \
  --metadata title="URDF Modeling of a 4-DOF Industrial Robotic Arm in ROS 2" \
  --variable title="" \
  --output "${HTML}"

"${CHROME}" --headless --disable-gpu --no-sandbox --no-pdf-header-footer \
  --print-to-pdf="${PKG_DIR}/docs/Project_Report.pdf" "file://${HTML}"

echo "Wrote ${PKG_DIR}/docs/Project_Report.pdf"
