#!/usr/bin/env bash
# Exit 1 if the diff base...head touches a protected path (rule 9).
# Used by .github/workflows/gate-guard.yml and runnable locally:
#   scripts/gate-guard.sh main my-branch
set -euo pipefail
base="${1:?base ref}"; head="${2:-HEAD}"
protected='^(tests/gates/|crates/mast/src/limits\.rs$|scene/generated/)'
hits="$(git diff --name-only "$base...$head" | grep -E "$protected" || true)"
if [[ -n "$hits" ]]; then
  echo "gate-guard: protected paths touched (owner-only, see CODEOWNERS):"
  echo "$hits"
  exit 1
fi
echo "gate-guard: no protected paths touched"
