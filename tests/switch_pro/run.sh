#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
cc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined \
  -Itests/switch_pro/stubs -Isrc tests/switch_pro/test_switch_mode.c \
  -o /tmp/joypad-switch-pro-test
/tmp/joypad-switch-pro-test
