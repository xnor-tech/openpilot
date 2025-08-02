#!/usr/bin/bash
uv pip install "numpy<2"

export PASSIVE="0"
exec ./launch_chffrplus.sh

