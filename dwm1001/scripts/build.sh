#!/usr/bin/env bash

export RUSTFLAGS="-D warnings"

cargo build --verbose --examples --all-features
