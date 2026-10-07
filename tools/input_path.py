"""Explicit input selection for the legacy offline inspection commands."""

import argparse
from pathlib import Path


def require_input_path(description):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("input", type=Path, help="Path to the input file to inspect")
    arguments = parser.parse_args()
    if not arguments.input.is_file():
        parser.error(f"input file does not exist: {arguments.input}")
    return arguments.input
