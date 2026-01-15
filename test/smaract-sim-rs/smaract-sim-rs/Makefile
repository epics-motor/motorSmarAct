


RS_SRC= \
  src/main.rs \
  netcommand.rs


# The first target
build: formatrust clippycargo buildcargo


formatrust:
	rustfmt src/*.rs src/netcommand/*.rs

buildcargo:
	cargo build

clippycargo:
	cargo clippy
