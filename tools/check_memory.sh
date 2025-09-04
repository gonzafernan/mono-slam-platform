#!/usr/bin/env bash
set -e

ELF=$1

FLASH_LIMIT=$((512 * 1024))   # 512 KB
RAM_LIMIT=$((128 * 1024))     # 128 KB

# Extract sizes
read text data bss dec hex filename < <(arm-none-eabi-size "$ELF" | tail -n1)

flash=$((text + data))   # flash = code + initialized data
ram=$((data + bss))      # ram = initialized + uninitialized data

echo "Flash used: $flash / $FLASH_LIMIT"
echo "RAM used:   $ram / $RAM_LIMIT"

if (( flash > FLASH_LIMIT )); then
    echo "❌ Flash limit exceeded!"
    exit 1
fi

if (( ram > RAM_LIMIT )); then
    echo "❌ RAM limit exceeded!"
    exit 1
fi

echo "✅ Memory usage within limits"

