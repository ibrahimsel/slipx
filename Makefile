# Wraps the canonical build, test and CI-check commands. Run `make help`.

BUILD_DIR      ?= build
CORE_BUILD_DIR ?= build-core
PYTHON         ?= python3

.DEFAULT_GOAL := help

.PHONY: help build test pytest check dep-lint licence-scan version-check \
        conformance core-only banner figures clean

help: ## List available targets
	@grep -E '^[a-z-]+:.*##' $(MAKEFILE_LIST) | \
	  awk 'BEGIN {FS = ":.*## "} {printf "  %-14s %s\n", $$1, $$2}'

build: ## Configure and build with the Python bindings (in-place extension)
	cmake -S . -B $(BUILD_DIR) -DSLIPX_BUILD_PYTHON=ON
	cmake --build $(BUILD_DIR) -j

test: build ## Run the C++ test suite
	ctest --test-dir $(BUILD_DIR) --output-on-failure

pytest: build ## Run the Python test suite (sink tests skip without extras)
	$(PYTHON) -m pytest

check: dep-lint licence-scan version-check conformance core-only ## The five CI checks a test suite does not cover

dep-lint: ## Dependency direction
	$(PYTHON) tools/dep_lint.py

licence-scan: ## Apache-2.0, no copyleft, extras cross-check
	$(PYTHON) tools/licence_scan.py

version-check: ## The version, in four places
	$(PYTHON) tools/version_check.py

conformance: build ## Trajectory hashes against the reference rows for this build
	$(PYTHON) tools/check_conformance.py --build-dir $(BUILD_DIR)

core-only: ## The core must configure and build alone
	cmake -S . -B $(CORE_BUILD_DIR) -DSLIPX_CORE_ONLY=ON
	cmake --build $(CORE_BUILD_DIR) -j

banner: ## Regenerate the README banner (needs Pillow)
	$(PYTHON) docs/assets/make_banner.py

figures: ## Regenerate the tutorial figures (standard library only)
	$(PYTHON) docs/racing/assets/make_figures.py

clean: ## Remove build trees
	rm -rf $(BUILD_DIR) $(CORE_BUILD_DIR)
