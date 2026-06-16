#!/bin/bash
# serve.sh — Run π0.5 inference server for TwinNexus
# Run from: ~/TwinNexus-Admittance-Platform/70_vla/
# Usage: ./serve.sh <checkpoint_dir> <checkpoint_step>
# Example: ./serve.sh twinnexus_run_001 10000

set -e

OPENPI_DIR=~/openpi
CONFIG_NAME="pi05_twinnexus_finetune"
EXP_NAME="${1:-twinnexus_run_001}"
CKPT_STEP="${2:-latest}"

CKPT_DIR="$OPENPI_DIR/checkpoints/$CONFIG_NAME/$EXP_NAME"

echo "================================================"
echo "  TwinNexus π0.5 Policy Server"
echo "  Config:     $CONFIG_NAME"
echo "  Checkpoint: $CKPT_DIR/$CKPT_STEP"
echo "  Port:       8000"
echo "================================================"

cd "$OPENPI_DIR"

uv run scripts/serve_policy.py \
    policy:checkpoint \
    --policy.config="$CONFIG_NAME" \
    --policy.dir="$CKPT_DIR/$CKPT_STEP"