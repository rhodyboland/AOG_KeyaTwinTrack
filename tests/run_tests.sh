#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
test_dir=$(mktemp -d "${TMPDIR:-/tmp}/keya-tests.XXXXXX")
trap 'rm -rf "$test_dir"' EXIT HUP INT TERM
for suite in test_safety test_motor; do
    g++ -std=c++11 -Wall -Wextra -Werror -fsanitize=address,undefined \
        -I. "tests/$suite.cpp" -o "$test_dir/$suite"
    ASAN_OPTIONS=detect_leaks=0 "$test_dir/$suite"
done
