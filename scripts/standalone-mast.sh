#!/usr/bin/env bash
# Row 3.6: prove `mast` builds and tests on its own, with no workspace, no
# `sim`, no MuJoCo and no `rookery-core`. Copies crates/mast alone into a
# temp dir, gives it a self-contained Cargo.toml (the workspace-inherited
# fields resolved from the root Cargo.toml), and runs `cargo test` there.
#
#   crates/mast/ ──copy──▶ $tmp/mast/ + standalone Cargo.toml ──▶ cargo test
#
# Run locally: scripts/standalone-mast.sh
set -euo pipefail
root="$(git rev-parse --show-toplevel)"
cd "$root"

field() { # read a [workspace.package] field from the root Cargo.toml
  sed -n '/^\[workspace.package\]/,/^\[/p' Cargo.toml | sed -n "s/^$1 *= *\"\(.*\)\"/\1/p"
}
dep() { # read a [workspace.dependencies] entry verbatim
  sed -n '/^\[workspace.dependencies\]/,/^\[/p' Cargo.toml | sed -n "s/^$1 *= *//p"
}

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
cp -R crates/mast "$tmp/mast"
cp rust-toolchain.toml "$tmp/mast/"

# Replace the workspace-inherited lines with concrete values.
python3 - "$tmp/mast/Cargo.toml" "$(field version)" "$(field edition)" "$(field rust-version)" \
  "$(field license)" "$(field repository)" "$(dep serde)" <<'PY'
import re, sys
path, version, edition, rust_version, license_, repo, serde = sys.argv[1:]
s = open(path).read()
for key, val in (("version", version), ("edition", edition), ("rust-version", rust_version),
                 ("license", license_), ("repository", repo)):
    s = re.sub(rf"(?m)^{re.escape(key)}\.workspace = true$", f'{key} = "{val}"', s)
s = s.replace("serde.workspace = true", f"serde = {serde}")
s = s.replace("[lints]\nworkspace = true",
              '[lints.rust]\nunsafe_op_in_unsafe_fn = "deny"\n\n[lints.clippy]\nall = { level = "warn", priority = -1 }')
s += "\n[workspace]\n"  # its own workspace root, so it never joins a parent
if "workspace = true" in s:
    sys.exit(f"standalone-mast: unresolved workspace inheritance left in {path}")
open(path, "w").write(s)
PY

cd "$tmp/mast"
if cargo tree -e normal | grep -qE '^(sim|rookery-core) |[ ─](sim|rookery-core) v'; then
  echo "error: standalone mast pulls in sim or rookery-core" >&2
  cargo tree -e normal >&2
  exit 1
fi
cargo test --quiet
echo "ok: mast builds and tests standalone ($tmp/mast)"
