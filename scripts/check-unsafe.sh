#!/usr/bin/env bash
# Fails if the word `unsafe` appears in any tracked .rs file outside
# crates/sim/src/ (row 1.8). Rule 4: every unsafe block lives in the sim
# facade, where each carries a `// SAFETY:` comment.
#
# The match is deliberately blunt: comments and strings count too. A keyword
# that is "only in a comment" today is one edit away from being code, and the
# check stays a one-line grep that anyone can read.
#
#   git ls-files '*.rs' ──drop crates/sim/src/──▶ grep -w unsafe ──hit──▶ exit 1
#
# Run locally: scripts/check-unsafe.sh
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"

hits="$(git ls-files -z -- '*.rs' ':!:crates/sim/src/**' | xargs -0 grep -nw -- unsafe || true)"
if [[ -n "$hits" ]]; then
  echo "error: \`unsafe\` outside crates/sim/src/ (rule 4):" >&2
  echo "$hits" >&2
  exit 1
fi
echo "ok: no \`unsafe\` outside crates/sim/src/"
