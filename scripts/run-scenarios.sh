#!/usr/bin/env bash
# Row 4.7: run every scenario under lua/scenarios/ that sets `ci = true`,
# headless, and fail if any check fails. This replaces the CI placeholder
# "headless: pending Phase 4".
#
#   lua/scenarios/*.lua ──(ci = true?)──▶ rookery run --scenario <file> ──▶ PASS / FAIL
#
# Run locally: scripts/run-scenarios.sh   (builds rookery in release mode)
set -euo pipefail
root="$(git rev-parse --show-toplevel)"
cd "$root"

cargo build --release --quiet -p rookery
bin="$(cargo metadata --format-version 1 --no-deps | python3 -c 'import json,sys; print(json.load(sys.stdin)["target_directory"])')/release/rookery"
logs="${RUNNER_TEMP:-${TMPDIR:-/tmp}}/rookery-scenarios"

failed=0
ran=0
for s in lua/scenarios/*.lua; do
  grep -qE '^ci *= *true' "$s" || continue
  ran=$((ran + 1))
  echo "== $s"
  if ! "$bin" run --scenario "$s" --log "$logs"; then
    failed=$((failed + 1))
  fi
done
echo "scenarios: $ran run, $failed failed"
[ "$ran" -gt 0 ] && [ "$failed" -eq 0 ]
