# https://github.com/casey/just
set shell := ["pwsh", "-c"]
alias t := test
alias d := doc
alias b := build
# alias r := run

doc:
  cargo doc --release

build: doc
  cargo build --release

test: build
  cargo test --release

# run: build
#   cargo run --release
