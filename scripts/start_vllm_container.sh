#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="${WORKSPACE_ROOT:-/home/ros/dance_ws_pedestrian_tracking}"
MODEL_DIR="${MODEL_DIR:-${WORKSPACE_ROOT}/vlm_models/Qwen/Qwen3-VL-2B-Instruct}"
HF_CACHE_DIR="${HF_CACHE_DIR:-${WORKSPACE_ROOT}/vlm_models/.hf-cache}"
VLLM_IMAGE="${VLLM_IMAGE:-ghcr.io/nvidia-ai-iot/vllm:latest-jetson-orin}"
HOST_ADDR="${HOST_ADDR:-0.0.0.0}"
PORT="${PORT:-8000}"
SERVED_MODEL_NAME="${SERVED_MODEL_NAME:-socialnav}"

mkdir -p "${HF_CACHE_DIR}"

exec docker run --rm -it --runtime=nvidia --network host \
  -e HF_HOME=/data/hf \
  -e VLLM_ATTENTION_BACKEND=FLASHINFER \
  -v "${MODEL_DIR}:/model" \
  -v "${HF_CACHE_DIR}:/data/hf" \
  "${VLLM_IMAGE}" \
  vllm serve /model \
    --served-model-name "${SERVED_MODEL_NAME}" \
    --host "${HOST_ADDR}" \
    --port "${PORT}" \
    --trust_remote_code \
    --gpu-memory-utilization 0.40 \
    --kv-cache-memory-bytes 6G \
    --max-num-seqs 1 \
    --max-num-batched-tokens 4096 \
    --max-model-len 4096 \
    --mm-processor-cache-gb 0