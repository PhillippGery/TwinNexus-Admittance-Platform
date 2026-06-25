# TwinNexus VLA Pipeline (`70_vla/`)

π0.5 fine-tuning and inference pipeline for the TwinNexus bimanual UR5e platform.

---

## File Structure

```
70_vla/
├── twinnexus_policy.py       ← π0.5 input/output transforms (symlinked into openpi)
├── convert_twinnexus.py      ← Dataset conversion: LeRobot v3.0 → openpi format
├── openpi_changes.patch      ← Git patch for openpi/src/openpi/training/config.py
├── train.sh                  ← One-command training script
├── serve.sh                  ← One-command inference server script
└── README.md                 ← This file
```

---

## Prerequisites

- openpi installed at `~/openpi/` (see below)
- LeRobot 0.5.2 in `~/lerobot_env/` (for recording)
- HuggingFace login: `huggingface-cli login`
- Dataset recorded and saved in `~/TwinNexus-Admittance-Platform/30_data/`

---

## One-Time Setup (New Machine)

```bash
# 1. Clone openpi
git clone --recurse-submodules git@github.com:Physical-Intelligence/openpi.git ~/openpi
cd ~/openpi
GIT_LFS_SKIP_SMUDGE=1 uv sync
GIT_LFS_SKIP_SMUDGE=1 uv pip install -e .

# 2. Apply TwinNexus patch
git apply ~/TwinNexus-Admittance-Platform/70_vla/openpi_changes.patch

# 3. Symlink policy file
ln -s ~/TwinNexus-Admittance-Platform/70_vla/twinnexus_policy.py \
      ~/openpi/src/openpi/policies/twinnexus_policy.py

# 4. HuggingFace login (required even for local datasets)
cd ~/openpi && uv run huggingface-cli login

# 5. Verify
grep -n "pi05_twinnexus_bimanual_finetune\|pi05_twinnexus_finetune" \
      ~/openpi/src/openpi/training/config.py
```

---

## Data Pipeline

### Why a conversion script?

LeRobot 0.5.2 (used for recording) produces **v3.0 format** datasets.
openpi's bundled LeRobot (0.1.0) expects **v2.1 format**.
Do NOT downgrade LeRobot — it would break the recording pipeline.
Instead, run the conversion script after each recording session.

### Convert dataset — Single arm

```bash
cd ~/openpi
uv run ~/TwinNexus-Admittance-Platform/70_vla/convert_twinnexus.py \
  --src ~/TwinNexus-Admittance-Platform/30_data/pick_place_001 \
  --repo pick_place_001_openpi \
  --task "pick up the screwdriver and place it in the paper box" \
  --overwrite
```

### Convert dataset — Bimanual

```bash
cd ~/openpi
uv run ~/TwinNexus-Admittance-Platform/70_vla/convert_twinnexus.py \
  --src ~/TwinNexus-Admittance-Platform/30_data/bimanual_box_001 \
  --repo bimanual_box_001_openpi \
  --task "pick yeallow Box and place in the red square on the Table" \
  --bimanual \
  --overwrite
```

The converter automatically handles datasets split across multiple shards
(`file-000.parquet`, `file-001.parquet`, …) produced by `--resume` recording sessions.

**Important:** Videos must be H264 encoded (not AV1) — openpi's cv2 cannot decode AV1.
The recording script uses `h264` by default. If you have AV1 videos, transcode first:

```bash
for f in $(find ~/TwinNexus-Admittance-Platform/30_data/pick_place_001 -name "*.mp4"); do
    ffmpeg -i "$f" -vcodec libx264 -crf 23 -preset fast "${f%.mp4}_h264.mp4" -y
    mv "${f%.mp4}_h264.mp4" "$f"
done
```

---

## Training

### Configs

| Config name | Mode | Dataset |
|-------------|------|---------|
| `pi05_twinnexus_finetune` | Single arm (7-dim) | `pick_place_001_openpi` |
| `pi05_twinnexus_bimanual_finetune` | Bimanual (14-dim) | `bimanual_box_001_openpi` |

Both use:
- π0.5 base weights (downloaded automatically from GCS)
- LoRA fine-tuning (`gemma_2b_lora` + `gemma_300m_lora`) — fits in 32GB VRAM
- Batch size 32

### Bimanual config parameters (`pi05_twinnexus_bimanual_finetune`)

| Parameter | Value | Notes |
|-----------|-------|-------|
| `num_train_steps` | 35,000 | ~50 epochs over 60 episodes |
| `warmup_steps` | 1,000 | 3% of total steps |
| `peak_lr` | 1e-4 | LoRA fine-tuning |
| `decay_steps` | 35,000 | Cosine decay over full training |
| `decay_lr` | 5e-6 | Decays to 5% of peak at end |
| `action_horizon` | 10 | 0.42s prediction window at 24fps |
| `save_interval` | 1,000 | Checkpoint every 1k steps (35 total) |

### Step 1 — Compute normalization stats (once per dataset)

```bash
cd ~/openpi
uv run scripts/compute_norm_stats.py --config-name pi05_twinnexus_bimanual_finetune
```

### Step 2 — Train

```bash
cd ~/openpi
XLA_PYTHON_CLIENT_MEM_FRACTION=0.9 \
uv run scripts/train.py pi05_twinnexus_bimanual_finetune \
  --exp-name=BimanualBoxPlace_v1 \
  --overwrite
```

Checkpoints saved to:
`~/openpi/checkpoints/pi05_twinnexus_bimanual_finetune/BimanualBoxPlace_v1/<step>/`

### Resume interrupted training

```bash
cd ~/openpi
XLA_PYTHON_CLIENT_MEM_FRACTION=0.9 \
uv run scripts/train.py pi05_twinnexus_bimanual_finetune \
  --exp-name=BimanualBoxPlace_v1
```

Drop `--overwrite` to continue from the last saved checkpoint.

---

## Known Issues & Solutions

### 1. LeRobot version mismatch
- **Problem:** openpi bundles LeRobot 0.1.0, recording uses 0.5.2
- **Solution:** Always run `convert_twinnexus.py` after recording

### 2. AV1 video decoding fails in openpi
- **Problem:** `cv2` in openpi's venv has no AV1 decoder
- **Solution:** Use `h264` encoder in the recording script (already set)

### 3. HuggingFace 401 error even for local datasets
- **Problem:** openpi's LeRobot calls HF Hub to check dataset versioning
- **Solution:** Always run `huggingface-cli login` before training

### 4. Conversion only shows partial episodes after `--resume`
- **Problem:** `--resume` splits data across multiple parquet/video shards;
  old converter only read `file-000`
- **Solution:** Fixed — `convert_twinnexus.py` now globs all shards in sorted order

### 5. OOM on RTX 5090 (32GB VRAM)
- **Problem:** Full π0.5 fine-tuning requires ~35GB VRAM
- **Solution:** LoRA fine-tuning (already configured) fits in ~23GB

### 6. LR schedule bugs (old single-arm config)
- **Problem:** `warmup_steps == num_train_steps` → model never trains at peak LR;
  `decay_lr == peak_lr` → no actual decay
- **Solution:** Fixed in `pi05_twinnexus_bimanual_finetune`

---

## Inference

After training completes:

```bash
# 1. Start policy server
cd ~/openpi
uv run scripts/serve_policy.py \
  policy:checkpoint \
  --policy.config=pi05_twinnexus_bimanual_finetune \
  --policy.dir=./checkpoints/pi05_twinnexus_bimanual_finetune/BimanualBoxPlace_v1/<step>

# 2. Run TwinNexus inference client
python3 ~/TwinNexus-Admittance-Platform/70_vla/twinnexus_inference.py
```

---

## Dataset Status

| Dataset | Episodes | Task | Status |
|---------|----------|------|--------|
| `pick_place_001` | 30 | Pick screwdriver → paper box | Recorded, converted |
| `pick_place_001_openpi` | 30 | Same | openpi format, in HF cache |
| `bimanual_box_001` | 60 | Pick yellow box → red square on table | Recorded (resumed after crash at ep 21) |
| `bimanual_box_001_openpi` | 60 | Same | Converted, ready for training |
