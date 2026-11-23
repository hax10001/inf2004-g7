#!/bin/sh
set -eu
cd "$(dirname "$0")"
cc -std=c11 -Wall -Wextra -Werror -pedantic -pthread \
 -I. buddy1_protocol.c buddy1.c host/test_tel.c -o /tmp/g7_telemetry_native_tests
/tmp/g7_telemetry_native_tests
