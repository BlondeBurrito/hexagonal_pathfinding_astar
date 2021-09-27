# https://github.com/casey/just
set shell := ["pwsh", "-c"]
alias t := test
alias d := doc
alias b := build
# alias r := run

test:
  cargo test --release

doc:
  cargo doc --release

build: test doc
  cargo build --release

# run: build
#   cargo run --release
