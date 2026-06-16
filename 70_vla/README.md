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
grep -n "pi05_twinnexus_finetune" ~/openpi/src/openpi/training/config.py
```

---

## Data Pipeline

### Why a conversion script?

LeRobot 0.5.2 (used for recording) produces **v3.0 format** datasets.  
openpi's bundled LeRobot (0.1.0) expects **v2.1 format**.  
Do NOT downgrade LeRobot — it would break the recording pipeline.  
Instead, run the conversion script after each recording session.

### Convert dataset

```bash
cd ~/openpi
uv run ~/TwinNexus-Admittance-Platform/70_vla/convert_twinnexus.py \
  --src ~/TwinNexus-Admittance-Platform/30_data/pick_place_001 \
  --repo pick_place_001_openpi \
  --task "pick up the screwdriver and place it in the paper box" \
  --overwrite
```

**Important:** Videos must be H264 encoded (not AV1) — openpi's cv2 cannot decode AV1.  
The recording script uses `h264_nvenc` by default. If you have AV1 videos, transcode first:

```bash
for f in $(find ~/TwinNexus-Admittance-Platform/30_data/pick_place_001 -name "*.mp4"); do
    ffmpeg -i "$f" -vcodec libx264 -crf 23 -preset fast "${f%.mp4}_h264.mp4" -y
    mv "${f%.mp4}_h264.mp4" "$f"
done
```

---

## Training

### Config name: `pi05_twinnexus_finetune`

The config uses:
- π0.5 base weights (downloaded automatically from GCS)
- LoRA fine-tuning (`gemma_2b_lora` + `gemma_300m_lora`) — fits in 32GB VRAM
- 10,000 training steps
- Batch size 64
- CosineDecay LR schedule, peak 5e-5, warmup 10k steps

### Step 1 — Compute normalization stats (once per dataset)

```bash
cd ~/openpi
uv run scripts/compute_norm_stats.py --config-name pi05_twinnexus_finetune
```

### Step 2 — Train

```bash
cd ~/openpi
XLA_PYTHON_CLIENT_MEM_FRACTION=0.9 \
uv run scripts/train.py pi05_twinnexus_finetune \
  --exp-name=ScrewdriverPickPlace_v1 \
  --overwrite
```

Checkpoints saved to: `~/openpi/checkpoints/pi05_twinnexus_finetune/<exp-name>/`

### Cluster training (A100 80GB)

Same commands — no changes needed. The A100 has enough VRAM for full fine-tuning  
(remove LoRA variants from config if desired), but LoRA is fine and much faster.

---

## Known Issues & Solutions

### 1. LeRobot version mismatch
- **Problem:** openpi bundles LeRobot 0.1.0, recording uses 0.5.2
- **Solution:** Always run `convert_twinnexus.py` after recording

### 2. AV1 video decoding fails in openpi
- **Problem:** `cv2` in openpi's venv has no AV1 decoder
- **Solution:** Use `h264_nvenc` encoder in `twinnexus_record.py` (already set)

### 3. HuggingFace 401 error even for local datasets
- **Problem:** openpi's LeRobot calls HF Hub to check dataset versioning
- **Solution:** Always run `huggingface-cli login` before training

### 4. Dataset must be tagged with codebase version on HF Hub
- **Problem:** `RevisionNotFoundError` when using HF Hub datasets
- **Solution:**
```python
from huggingface_hub import HfApi
import json
with open('path/to/info.json') as f:
    version = json.load(f)['codebase_version']
HfApi().create_tag('PhillippGery/pick_place_001', tag=version, repo_type='dataset')
```

### 5. OOM with full fine-tuning on RTX 5090 (32GB)
- **Problem:** Full π0.5 fine-tuning requires ~35GB VRAM
- **Solution:** LoRA fine-tuning (already configured) fits in ~23GB

### 6. RepackTransform key direction
- **Problem:** Keys must map `{internal_key: dataset_key}` not the other way
- **Solution:** Already fixed in `openpi_changes.patch`

---

## Inference (TODO)

After training completes:

```bash
# 1. Start policy server
cd ~/openpi
uv run scripts/serve_policy.py \
  policy:checkpoint \
  --policy.config=pi05_twinnexus_finetune \
  --policy.dir=./checkpoints/pi05_twinnexus_finetune/<exp-name>/<step>

# 2. Run TwinNexus inference client (to be implemented)
python3 ~/TwinNexus-Admittance-Platform/70_vla/twinnexus_inference.py
```

---

## Bimanual Expansion (Planned)

When left arm is validated:
- Update `convert_twinnexus.py`: add `wrist_left` camera + extend state/action to (14,)
- Update `twinnexus_policy.py`: enable `right_wrist_0_rgb` slot
- Update `TwinNexusRobotConfig`: enable `wrist_left_serial`
- Collect bimanual episodes and retrain

---

## Dataset Status

| Dataset | Episodes | Task | Status |
|---------|----------|------|--------|
| `pick_place_001` | 30 | Pick screwdriver → paper box | Recorded, converted, training |
| `pick_place_001_openpi` | 30 | Same | openpi format, in HF cache |