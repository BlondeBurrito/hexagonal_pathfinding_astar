# very useful command line runner - https://github.com/casey/just
set shell := ["pwsh", "-c"]
alias c := clippy
alias d := doc
alias db := debug
alias t := test
alias b := build
alias r := run
alias clog := changelog
# alias cn := clean

bt := '0'

export RUST_BACKTRACE := bt

clippy:
  cargo clippy --workspace --all-targets --all-features -- -D warnings -D clippy::cargo_common_metadata -D clippy::missing_docs_in_private_items -W clippy::todo -W clippy::unimplemented

# run a debug build so the compiler can call out overflow errors etc, rather than making assumptions
debug:
  cargo build

test: debug
  cargo test --release

doc:
  cargo doc --release

build: test doc
  cargo build --release

run: build
  cargo run --release

clean:
  cargo clean

push MESSAGE +BRANCH='main':
  git add .
  git commit -m "{{MESSAGE}}"
  git push origin {{BRANCH}}
# https://github.com/rust-lang/rust/issues/58154

changelog TAG:
  git cliff --tag {{TAG}} --output CHANGELOG.md

doc-coverage:
  $env:RUSTDOCFLAGS="-Z unstable-options --show-coverage"
  cargo +nightly doc --workspace --all-features --no-deps