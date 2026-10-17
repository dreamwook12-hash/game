#!/usr/bin/env sh
# Read-only alternate host probe; no network and no identifiers.
set -u
printf 'OS/kernel: '; uname -srm
if command -v lscpu >/dev/null 2>&1; then lscpu | sed -n '/^Architecture:/p;/^CPU(s):/p;/^Model name:/p'; fi
if command -v free >/dev/null 2>&1; then free -h; fi
df -h .
for tool in c++ clang++ g++ cmake ninja git python3 dotnet java adb xcodebuild; do
  if command -v "$tool" >/dev/null 2>&1; then command -v "$tool"; else printf '%s: NOT FOUND\n' "$tool"; fi
done
printf 'GPU/display/execution environment: UNKNOWN (use platform-specific diagnostics)\n'
