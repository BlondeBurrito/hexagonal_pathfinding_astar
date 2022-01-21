# https://github.com/casey/just
set shell := ["pwsh", "-c"]
alias t := test
alias d := doc
alias db := debug
alias b := build
# alias r := run

bt := '0'

export RUST_BACKTRACE := bt

clippy:
  cargo clippy --all-targets --all-features

# run a debug build so the compiler can call out overflow errors etc, rather than making assumptions
debug:
  cargo build

test: debug
  cargo test --release

doc:
  cargo doc --release

build: doc test
  cargo build --release

# run: build
#   cargo run --release

clean:
  cargo clean
