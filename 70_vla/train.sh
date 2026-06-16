#!/bin/bash
# train.sh — Fine-tune π0.5 on TwinNexus pick-and-place data
# Run from: ~/TwinNexus-Admittance-Platform/70_vla/
# Prerequisites: openpi installed at ~/openpi/

set -e

OPENPI_DIR=~/openpi
CONFIG_NAME="pi05_twinnexus_finetune"
EXP_NAME="${1:-twinnexus_run_001}"

echo "================================================"
echo "  TwinNexus π0.5 Fine-tuning"
echo "  Config:     $CONFIG_NAME"
echo "  Experiment: $EXP_NAME"
echo "  Checkpoint: $OPENPI_DIR/checkpoints/$CONFIG_NAME/$EXP_NAME"
echo "================================================"

cd "$OPENPI_DIR"

# Step 1: Compute normalization stats (only needed once per dataset)
if [ ! -f "$HOME/.cache/huggingface/lerobot/PhillippGery/pick_place_001/norm_stats.json" ]; then
    echo ""
    echo "Step 1: Computing normalization stats..."
    uv run scripts/compute_norm_stats.py --config-name "$CONFIG_NAME"
else
    echo "Step 1: Normalization stats already exist, skipping."
fi

# Step 2: Train
echo ""
echo "Step 2: Starting training..."
echo "Monitor at: https://wandb.ai"
echo ""

XLA_PYTHON_CLIENT_MEM_FRACTION=0.9 \
uv run scripts/train.py "$CONFIG_NAME" \
    --exp-name="$EXP_NAME" \
    --overwrite

echo ""
echo "Training complete."
echo "Checkpoint saved to: $OPENPI_DIR/checkpoints/$CONFIG_NAME/$EXP_NAME"