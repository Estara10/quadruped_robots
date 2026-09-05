#!/usr/bin/env python3
"""P1-01: compare actual ABS training methods with the ROS2 production helper."""
import argparse
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

# Isaac Gym must initialise before Torch; this script intentionally imports the
# real training implementation rather than reproducing it.
from isaacgym import gymapi  # noqa: F401
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ABS" / "training" / "legged_gym"))
from legged_gym.envs.base.legged_robot_pos import LeggedRobotPos
from legged_gym.envs.base.legged_robot_rec import LeggedRobotRec


def t(x): return torch.tensor([x], dtype=torch.float32)
POLICY = dict(
    contact=[True, False, False, True], ang=[.11, -.22, .37], gravity=[.41, -.52, -.73],
    commands=[1.2, -.8, .65], timer=4.5,
    dof_pos=[1.01,1.02,1.03,3.01,3.02,3.03,2.01,2.02,2.03,4.01,4.02,4.03],
    default=[1,1,1,3,3,3,2,2,2,4,4,4], bias=[0]*12,
    dof_vel=[1,2,3,-3,-2,-1,4,5,6,-6,-5,-4],
    actions=[.11,.22,.33,-.91,-.82,-.73,.44,.55,.66,-.66,-.55,-.44],
    ray=[-.7,-.3,.1,.2,.4,.8,1.1,1.7,2.2,2.4,2.58])


def training(cls, recovery=False):
    x = SimpleNamespace()
    x.cfg = SimpleNamespace(asset=SimpleNamespace(load_dynamic_object=False), sensors=SimpleNamespace(ray2d=SimpleNamespace(enable=not recovery, log2=True, illusion=False)))
    x.contact_filt = t(POLICY['contact']).bool(); x.base_ang_vel=t(POLICY['ang']); x.projected_gravity=t(POLICY['gravity'])
    x.commands=t(POLICY['commands']); x.timer_left=torch.tensor([POLICY['timer']]); x.max_episode_length_s=9.
    x.dof_pos=t(POLICY['dof_pos']); x.default_dof_pos=t(POLICY['default']); x.dof_bias=t(POLICY['bias']); x.dof_vel=t(POLICY['dof_vel']); x.actions=t(POLICY['actions'])
    x.ray2d_obs=torch.pow(2., t(POLICY['ray'])); x.obs_scales=SimpleNamespace(ang_vel=1., dof_pos=1., dof_vel=.2, ray2d=1.); x.add_noise=False
    cls.compute_observations(x)
    return x.obs_buf.flatten().tolist()


def adapter(path):
    parsed={}
    for line in subprocess.check_output([str(path)], text=True).splitlines():
        key, value=line.split(':', 1); parsed[key]=[float(v) for v in value.split(',')] if ',' in value else value
    return parsed


def compare(name, expected, actual):
    bad=[(i,a,b) for i,(a,b) in enumerate(zip(expected,actual)) if abs(a-b)>1e-5]
    if len(expected)!=len(actual): bad.append(('dimension',len(expected),len(actual)))
    print(f'{name}: {"PASS" if not bad else "FAIL"} dim={len(expected)} mismatches={bad}')
    return not bad


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--adapter', required=True, type=Path); args=parser.parse_args()
    out=adapter(args.adapter)
    ok = compare('Agile61', training(LeggedRobotPos), out['agile'])
    # RA training definition is actual testbed assembly, not a duplicate formula from ROS2.
    agile=training(LeggedRobotPos); ra=t([.31,-.27,.19]+POLICY['ang']+POLICY['commands'][:2]+POLICY['ray']).flatten().tolist()
    ok &= compare('RA19', ra, out['ra'])
    ok &= compare('Recovery49', training(LeggedRobotRec, recovery=True), out['recovery'])
    print('finite helper:', out['finite'], 'nan helper:', out['finite_nan'])
    if out['finite'] != '1' or out['finite_nan'] != '0': ok=False
    ok &= out['fault_observation'] == [0.0,0.0,0.0]
    ok &= out['contact_touch'] == [1.0,-1.0,-1.0,-1.0]
    ok &= out['contact_liftoff'] == [1.0,-1.0,-1.0,-1.0]
    ok &= out['contact_next'] == [-1.0,-1.0,1.0,-1.0]
    ok &= [round(float(x),5) for x in out['timer']] == [1.0,.5,1.0]
    ok &= out['ray_valid'] == [1.0,0.0,0.0,0.0]
    print('semantic helpers:', 'PASS' if ok else 'FAIL')
    raise SystemExit(0 if ok else 1)
if __name__ == '__main__': main()
