#!/bin/sh
#make && RUST_BACKTRACE=1 target/debug/smaract-sim-rs -d
make && RUST_BACKTRACE=1 target/debug/smaract-sim-rs "$@"
