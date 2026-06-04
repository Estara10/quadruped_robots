import torch
import torch.nn as nn
import sys, os

model_path = sys.argv[1] if len(sys.argv) > 1 else "05_27_15-53-31_model_4000_ra.pt"
output_path = os.path.join(os.path.dirname(model_path) or ".", "ra_value_jit.pt")

print(f"Loading: {model_path}")
ra = nn.Sequential(nn.Linear(19, 64), nn.ReLU(), nn.Linear(64, 64), nn.ReLU(), nn.Linear(64, 1), nn.Tanh())
ra = torch.load(model_path, map_location="cpu")
ra.eval()

print(f"Original model input: 19, output: 1")
print("Tracing with TorchScript...")
t = torch.jit.trace(ra, torch.randn(1, 19))
t.save(output_path)
print(f"Saved: {output_path}")

# Verify
loaded = torch.jit.load(output_path)
out = loaded(torch.randn(1, 19))
print(f"Verify OK — output shape: {out.shape}, value: {out.item():.4f}")
