#!/usr/bin/env bash

# Generate PlatformIO project files from STM32CubeMX .ioc file
echo "Generating PlatformIO project files from STM32CubeMX .ioc file..."
stm32pio generate

# dos2unix the generated main.c to fix line ending issues
echo "Converting line endings of main.c from DOS to UNIX format..."
dos2unix ./Src/main.c
