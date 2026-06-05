#!/usr/bin/env python3
"""Minimal TorchScript export: loads checkpoint → extracts actor → saves .pt
Run on server after training completes. No Isaac Gym needed.

Usage (on server):
    conda activate abs
    cd /data/sxq/ABS/training/legged_gym/legged_gym
    python /path/to/export_rec_policy.py \
        /data/sxq/ABS/training/legged_gym/logs/go2_rec_rough/06_04_22-43-20_/model_15000.pt \
        /data/sxq/ABS/training/legged_gym/logs/go2_rec_rough/exported/policies/
"""

import sys
import os
import copy
import torch
import torch.nn as nn


def build_actor(num_obs=49, num_actions=12, hidden_dims=(512, 256, 128), activation='elu'):
    """Match the ABS ActorCritic.actor architecture exactly."""
    act_fn = {'elu': nn.ELU(), 'relu': nn.ReLU(), 'tanh': nn.Tanh()}[activation]
    layers = []
    layers.append(nn.Linear(num_obs, hidden_dims[0]))
    layers.append(act_fn)
    for i in range(len(hidden_dims)):
        if i == len(hidden_dims) - 1:
            layers.append(nn.Linear(hidden_dims[i], num_actions))
        else:
            layers.append(nn.Linear(hidden_dims[i], hidden_dims[i + 1]))
            layers.append(act_fn)
    return nn.Sequential(*layers)


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <checkpoint.pt> [output_dir]")
        print(f"  checkpoint.pt: path to model_15000.pt")
        print(f"  output_dir:    where to save exported policy (default: same dir as checkpoint)")
        sys.exit(1)

    ckpt_path = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.dirname(ckpt_path)

    print(f"Loading checkpoint: {ckpt_path}")
    ckpt = torch.load(ckpt_path, map_location='cpu', weights_only=False)

    # The checkpoint stores the full ActorCritic state dict
    # We need to reconstruct the actor part from it
    state_dict = ckpt['model_state_dict']

    # Extract actor keys (prefix "actor.")
    actor_state = {}
    for key, value in state_dict.items():
        if key.startswith('actor.'):
            actor_state[key[6:]] = value  # strip "actor." prefix

    print(f"Found {len(actor_state)} actor parameter tensors")

    # Detect input/output dims from first/last layer
    first_weight = actor_state['0.weight']
    num_obs = first_weight.shape[1]
    # Last layer weight
    last_key = sorted([k for k in actor_state if k.endswith('.weight')])[-1]
    num_actions = actor_state[last_key].shape[0]

    print(f"Detected: num_obs={num_obs}, num_actions={num_actions}")

    # Build actor with Go2 recovery architecture: [512, 256, 128]
    actor = build_actor(num_obs=num_obs, num_actions=num_actions,
                        hidden_dims=(512, 256, 128), activation='elu')
    actor.load_state_dict(actor_state)
    actor.eval()

    # Quick sanity check
    dummy = torch.randn(1, num_obs)
    with torch.no_grad():
        out = actor(dummy)
    print(f"Test forward pass: input {dummy.shape} → output {out.shape}")
    print(f"Output range: [{out.min().item():.4f}, {out.max().item():.4f}]")

    # Export to TorchScript
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, 'policy.pt')
    traced = torch.jit.script(actor)
    traced.save(output_path)

    size_kb = os.path.getsize(output_path) / 1024
    print(f"Exported: {output_path} ({size_kb:.0f} KB)")


if __name__ == '__main__':
    main()
