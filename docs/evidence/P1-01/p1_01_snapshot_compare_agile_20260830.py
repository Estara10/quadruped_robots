#!/usr/bin/env python3
"""Read-only: prove checkpoint model_4000.pt actor weights == exported TorchScript params.

Formats differ (state-dict vs TorchScript), so we compare the actual weight
tensors. Prints exact tensor-by-tensor equality.
"""
import torch
from pathlib import Path

CKPT = Path("ABS_fuwuqi/ABS/training/legged_gym/logs/go2_pos_rough/05_27_15-53-31_/model_4000.pt")
EXPORT = Path("ABS_fuwuqi/ABS/training/legged_gym/logs/go2_pos_rough/exported/policies/05_27_15-53-31_model_4000.pt")
DEPLOYED = Path("quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/abs/policy.pt")

ckpt = torch.load(str(CKPT), map_location="cpu")
print("checkpoint top-level keys:", list(ckpt.keys()))
print("checkpoint iter:", ckpt.get("iter"), "infos:", ckpt.get("infos"))
msd = ckpt["model_state_dict"]
actor_keys = sorted(k for k in msd if k.startswith("actor"))
print("checkpoint model_state_dict keys:", sorted(msd.keys()))
print("checkpoint actor keys (%d):" % len(actor_keys), actor_keys)

# exported TorchScript parameters (the module that runs in production)
for label, path in (("export", EXPORT), ("deployed", DEPLOYED)):
    m = torch.jit.load(str(path), map_location="cpu")
    params = {k: v for k, v in m.named_parameters()}
    buffers = {k: v for k, v in m.named_buffers()}
    print(f"\n=== {label} ({path.name}) named_parameters ===")
    for k, v in params.items():
        print(f"  {k}: {tuple(v.shape)}")
    print(f"  buffers: { {k: tuple(v.shape) for k, v in buffers.items()} }")

    # Map checkpoint actor.* to the exported module's sequential layers.
    # Export is a ScriptModule whose forward chains getattr(self,"0".."6").
    # actor.i -> module "i" parameters.
    all_ok = True
    for i, akey in enumerate(actor_keys):
        weight = msd[akey]
        # exported param names: "<i>.weight" / "<i>.bias"
        pname = akey.replace("actor.", "")  # e.g. "0.weight"
        if pname in params:
            eq = bool(torch.equal(weight, params[pname]))
            all_ok &= eq
            print(f"  actor->export map {akey} == {pname}: {eq} shapes {tuple(weight.shape)}")
        else:
            print(f"  NOTE: no exported param named {pname} (actor {akey})")
    print(f"  => {label} weight match: {'ALL EQUAL' if all_ok else 'MISMATCH'}")
