#!/usr/bin/env bash
# Regenerates VERSIONS.md at the repo root with the current machine's toolchain
# versions, and warns if this machine is behind what's already recorded there
# (e.g. from another machine, via git). Run by the SessionStart hook.
set -euo pipefail

cd "$(git rev-parse --show-toplevel 2>/dev/null || echo .)"

OUT="VERSIONS.md"
HOSTNAME_VAL="$(hostname 2>/dev/null || echo unknown)"
DATE_VAL="$(date '+%Y-%m-%d %H:%M %Z')"

# --- Read what was previously recorded (from git / another machine) before we overwrite it ---
PREV_CLI=""
PREV_PLUGINS=""
if [ -f "$OUT" ]; then
  PREV_CLI="$(grep -m1 '^- CLI version:' "$OUT" 2>/dev/null | sed 's/^- CLI version: //' | grep -oE '^[0-9]+\.[0-9]+\.[0-9]+' || true)"
  PREV_PLUGINS="$(awk '/^### Plugins/{f=1;next} f&&/^```/{c++;if(c==2)exit;next} f&&c==1{print}' "$OUT" 2>/dev/null || true)"
fi

# --- Collect current machine's values ---
CUR_CLI_RAW="$(claude --version 2>/dev/null || echo 'unknown')"
CUR_CLI="$(echo "$CUR_CLI_RAW" | grep -oE '^[0-9]+\.[0-9]+\.[0-9]+' || true)"
CUR_PLUGINS="$(claude plugin list 2>/dev/null || echo 'unavailable')"
CUR_MCP="$(claude mcp list 2>/dev/null || echo 'unavailable')"

# --- Build drift warnings ---
WARNINGS=()

if [ -n "$PREV_CLI" ] && [ -n "$CUR_CLI" ] && [ "$CUR_CLI" != "$PREV_CLI" ]; then
  NEWEST="$(printf '%s\n%s\n' "$CUR_CLI" "$PREV_CLI" | sort -V | tail -1)"
  if [ "$NEWEST" = "$PREV_CLI" ]; then
    WARNINGS+=("Claude Code CLI is behind: this machine has $CUR_CLI, VERSIONS.md last recorded $PREV_CLI. Fix: claude update")
  fi
fi

if [ -n "$PREV_PLUGINS" ] && [ "$(echo "$PREV_PLUGINS" | head -1)" != "No plugins installed. Use \`claude plugin install\` to install a plugin." ]; then
  while IFS= read -r line; do
    [ -z "$line" ] && continue
    if ! echo "$CUR_PLUGINS" | grep -qF "$line"; then
      WARNINGS+=("Plugin missing on this machine: $line. Fix: claude plugin install $line")
    fi
  done <<< "$PREV_PLUGINS"
fi

# --- Regenerate VERSIONS.md ---
{
  echo "# Environment Versions"
  echo
  echo "_Last updated: $DATE_VAL on \`$HOSTNAME_VAL\`_"
  echo
  echo "## Claude Code"
  echo
  echo "- CLI version: $CUR_CLI_RAW"
  echo
  echo "### Plugins"
  echo '```'
  echo "$CUR_PLUGINS"
  echo '```'
  echo
  echo "### MCP servers"
  echo '```'
  echo "$CUR_MCP"
  echo '```'
  echo
  echo "## FTC / Android build toolchain"
  echo "_(synced via git — if behind here, \`git pull\` is the fix, not this script)_"
  echo
  if [ -f gradle/wrapper/gradle-wrapper.properties ]; then
    GRADLE_VER=$(grep -oE 'gradle-[0-9.]+' gradle/wrapper/gradle-wrapper.properties | head -1 | sed 's/gradle-//' || true)
    echo "- Gradle: ${GRADLE_VER:-unknown}"
  fi
  if [ -f build.gradle ]; then
    AGP_VER=$(grep -oE "com\.android\.tools\.build:gradle:[0-9A-Za-z.-]+" build.gradle | head -1 | sed 's/.*gradle://' || true)
    if [ -z "$AGP_VER" ]; then
      AGP_VER=$(grep -oE "com\.android\.(application|library)'[[:space:]]+version[[:space:]]+'[0-9A-Za-z.-]+'" build.gradle | head -1 | grep -oE "[0-9][0-9A-Za-z.-]*'$" | tr -d "'" || true)
    fi
    echo "- Android Gradle Plugin: ${AGP_VER:-unknown}"
  fi
  if [ -f FtcRobotController/build.gradle ]; then
    COMPILE_SDK=$(grep -oE 'compileSdk(Version)?[[:space:]]+[0-9]+' FtcRobotController/build.gradle | head -1 | grep -oE '[0-9]+' || true)
    MIN_SDK=$(grep -oE 'minSdkVersion[[:space:]]+[0-9]+' FtcRobotController/build.gradle | head -1 | grep -oE '[0-9]+' || true)
    TARGET_SDK=$(grep -oE 'targetSdkVersion[[:space:]]+[0-9]+' FtcRobotController/build.gradle | head -1 | grep -oE '[0-9]+' || true)
    echo "- compileSdk: ${COMPILE_SDK:-unknown}"
    echo "- minSdkVersion: ${MIN_SDK:-unknown}"
    echo "- targetSdkVersion: ${TARGET_SDK:-unknown}"
  fi
  if [ -f build.dependencies.gradle ]; then
    echo
    echo "### Library versions (build.dependencies.gradle)"
    echo '```'
    grep -oE "implementation '[^']+'" build.dependencies.gradle | sed "s/implementation '//;s/'$//" || true
    echo '```'
  fi
} > "$OUT"

# --- Surface any drift as a visible session-start message ---
if [ "${#WARNINGS[@]}" -gt 0 ]; then
  MSG="This machine is behind (see VERSIONS.md):"
  for w in "${WARNINGS[@]}"; do
    MSG="$MSG
- $w"
  done
  # Escape backslashes/quotes first, then collapse real newlines to literal \n for JSON.
  ESCAPED="$(printf '%s' "$MSG" | sed 's/\\/\\\\/g; s/"/\\"/g' | sed ':a;N;$!ba;s/\n/\\n/g')"
  printf '{"systemMessage": "%s"}\n' "$ESCAPED"
fi
