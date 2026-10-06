#!/usr/bin/env python3
"""Read-only: prove named RA artifact -> ra_value_jit.pt -> deployed ra_value.pt."""
import torch
from pathlib import Path

NAMED = Path("ABS_fuwuqi/ABS/training/legged_gym/logs/go2_pos_rough/exported/RA/05_27_15-53-31_model_4000_ra.pt")
JIT = Path("ABS_fuwuqi/ABS/training/legged_gym/logs/go2_pos_rough/exported/RA/ra_value_jit.pt")
DEPLOYED = Path("quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/abs/ra_value.pt")

# inspect named RA artifact
obj = torch.load(str(NAMED), map_location="cpu")
if isinstance(obj, dict):
    print("named RA top-level keys:", list(obj.keys()))
    src = obj.get("model_state_dict", obj) if "model_state_dict" in obj else obj
else:
    print("named RA is a module")
    src = {k: v for k, v in obj.state_dict().items()}
named_tensors = {k: v for k, v in src.items() if hasattr(v, "shape")}
print("named RA tensors:", {k: tuple(v.shape) for k, v in named_tensors.items()})

for label, path in (("jit", JIT), ("deployed", DEPLOYED)):
    m = torch.jit.load(str(path), map_location="cpu")
    params = {k: v for k, v in m.named_parameters()}
    buffers = {k: v for k, v in m.named_buffers()}
    print(f"\n=== {label} params: { {k: tuple(v.shape) for k, v in params.items()} }")
    print(f"    buffers: { {k: tuple(v.shape) for k, v in buffers.items()} }")
    # map named-RA tensor order to the module's params by shape+value
    all_ok = True
    for i, (nk, nv) in enumerate(sorted(named_tensors.items())):
        # match by shape against params/buffers
        for mk, mv in list(params.items()) + list(buffers.items()):
            if tuple(mv.shape) == tuple(nv.shape) and torch.equal(nv, mv):
                print(f"  named[{nk}] == {label}[{mk}] : True")
                break
        else:
            all_ok = False
            print(f"  named[{nk}] ({tuple(nv.shape)}): NO exact match in {label}")
    print(f"  => named RA -> {label}: {'ALL 6 TENSORS EQUAL' if all_ok else 'PARTIAL/NO MATCH'}")
