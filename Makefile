.DEFAULT_GOAL := help

# Optional cargo test arguments, e.g. make test ARGS=it_works
ARGS ?=

.PHONY: help default b t l doc build clean install_dev_tools fmt lint pedantic audit test cov ci

help: ## List available targets
	@awk 'BEGIN { FS = ":.*## " } /^[a-zA-Z_][a-zA-Z0-9_-]*:.*## / { printf "  %-20s %s\n", $$1, $$2 }' $(MAKEFILE_LIST)

default: help
b: build
t: test
l: lint

doc: ## Build and open documentation
	cargo doc --all-features --no-deps --open

build: ## Build with default features and all features
	cargo build
	cargo build --all-features

clean: ## Remove build artifacts
	cargo clean --verbose

install_dev_tools: ## Install cargo tools used in package maintenance
	cargo install --locked release-plz
	cargo install --locked cargo-audit
	cargo install --locked cargo-outdated
	cargo install --locked cargo-llvm-cov
	cargo install --locked cargo-expand

fmt: ## Format source code
	cargo fmt --all

lint: ## Check formatting and lint all targets with default and all features
	cargo fmt --all -- --check
	cargo check
	cargo check --all-features
	cargo clippy --all-targets -- -D warnings
	cargo clippy --all-targets --all-features -- -D warnings

pedantic: ## Lint source code with the pedantic linter
	cargo clippy --all-targets --all-features -- -W clippy::pedantic

audit: ## Audit dependencies for known vulnerabilities
	cargo audit

test: ## Run tests for all features (optional ARGS='...')
	cargo test --all-features $(ARGS) -- --show-output

cov: ## Generate and open a code coverage report
	cargo llvm-cov --all-features --open

ci: ## Run local CI checks
	$(MAKE) lint
	$(MAKE) build
	cargo test
	$(MAKE) test
