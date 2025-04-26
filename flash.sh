#! /bin/bash
DEFMT_LOG=off DEBUG=0 cargo size --release -- -A

cargo flash --chip STM32F103C8 --release
