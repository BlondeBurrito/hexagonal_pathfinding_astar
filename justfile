# very useful command line runner - https://github.com/casey/just
set windows-powershell := true
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

# print recipes
default:
  just --list
# lint the code aggressively
clippy:
  cargo clippy --workspace --all-targets --all-features -- -D warnings -D clippy::cargo_common_metadata -D clippy::missing_docs_in_private_items -W clippy::todo -W clippy::unimplemented
# run a debug build so the compiler can call out overflow errors etc, rather than making assumptions
debug:
  cargo build
# run tests
test: debug
  cargo test --release
# generate documentation
doc:
  cargo doc --release
# build release bin/lib
build: test doc
  cargo build --release
# build and execute bin
run: build
  cargo run --release
# delete `target` directory
clean:
  cargo clean
# git push with a message and optional branch target
push MESSAGE +BRANCH='main':
  git add .
  git commit -m "{{MESSAGE}}"
  git push origin {{BRANCH}}
# generate a changelog with git-cliff-based on conventional commits
changelog TAG:
  git cliff --tag {{TAG}} --output CHANGELOG.md
# evaluate documentation coverage
doc-coverage:
  $env:RUSTDOCFLAGS="-Z unstable-options --show-coverage"
  cargo +nightly doc --workspace --all-features --no-deps
  # https://github.com/rust-lang/rust/issues/58154
# install the crate from the local source rather than remote
install:
  cargo install --path .
# Useful tools
dev-tools:
  cargo install loc;
  cargo install git-cliff;
  cargo install flamegraph;
  cargo install cargo-bloat;
  cargo install cargo-deadlinks;
  cargo install cargo-geiger;
  cargo install cargo-modules;
  cargo install --locked cargo-outdated;
  cargo install cargo-watch;
  cargo install hyperfine;
  cargo install rust-script;
  rust-script --install-file-association;
  cargo install --locked cargo-deny