# Human-facing entrypoints. Run from WSL/Linux/macOS (or `uv run ...` directly on Windows).

.PHONY: setup lint fmt typecheck test check

setup:            ## Install workspace + dev tools (uv required: https://docs.astral.sh/uv/)
	uv sync --all-packages

lint:             ## Static lint
	uv run ruff check .
	uv run ruff format --check .

fmt:              ## Auto-format
	uv run ruff format .
	uv run ruff check --fix .

typecheck:        ## mypy --strict on typed packages
	uv run mypy

test:             ## Unit + contract tests with coverage gate
	uv run pytest

check: lint typecheck test   ## Everything CI runs
