#!/usr/bin/env python3
"""P1-06 deterministic fixture: Recovery safe-twist optimizer parity.

Independent Python arithmetic oracle for Eq.21/Eq.22, citing the real source
expressions. It does NOT execute StateRL or testbed; every formula below is
transcribed from the audited source lines:

- Eq.22 reference (recovered testbed `get_pos_integral`, testbed.py:55-61):
    theta = wz*tau; x = vx*tau - 0.5*vy*wz*tau^2; y = vy*tau + 0.5*vx*wz*tau^2
- Eq.22 deployment (StateRL.cpp:623-624): pos_x = vx*tau; pos_y = vy*tau
  (yaw-coupled second-order terms omitted).
- Eq.21 objective (testbed.py:343, StateRL.cpp:627-628):
    loss = lam*max(ra + 2*eps, 0) + 0.02*((x-goal_x)^2 + (y-goal_y)^2)
- Bounds (testbed.py:67-68, StateRL.cpp:599): component-wise
  [vx,vy,wz] in [-1.5,-0.3,-3.0] .. [+1.5,+0.3,+3.0]
- Gradient clip: reference `_clip_grad` = L2-norm clip (testbed.py:70-72);
  deployment = per-element clamp to [-1,1] (StateRL.cpp:633).

Run: python3 scripts/test_p1_06_recovery_optimizer.py
"""
import math


def get_pos_integral_reference(vx, vy, wz, tau):
    """Recovered testbed testbed.py:55-61 (yaw-coupled second order)."""
    theta = wz * tau
    x = vx * tau - 0.5 * vy * wz * tau * tau
    y = vy * tau + 0.5 * vx * wz * tau * tau
    return x, y, theta


def pos_deployment(vx, vy, wz, tau):
    """StateRL.cpp:623-624 (first order only; yaw coupling omitted)."""
    return vx * tau, vy * tau  # theta not used in the deployment position penalty


def objective(ra, goal_x, goal_y, x_est, y_est, lam=10.0, eps=0.05):
    """Eq.21 loss: lam*max(ra+2*eps,0) + 0.02*((x-goal_x)^2+(y-goal_y)^2)."""
    ra_pen = lam * max(ra + 2.0 * eps, 0.0)
    pos_pen = 0.02 * ((x_est - goal_x) ** 2 + (y_est - goal_y) ** 2)
    return ra_pen, pos_pen, ra_pen + pos_pen


def clamp_bounds(vx, vy, wz, vx_m=1.5, vy_m=0.3, wz_m=3.0):
    """Component-wise bounds testbed.py:67-68 / StateRL.cpp:636-638."""
    return (max(-vx_m, min(vx, vx_m)), max(-vy_m, min(vy, vy_m)),
            max(-wz_m, min(wz, wz_m)))


def l2_grad_clip(grad, thres=1.0):
    """Reference _clip_grad (testbed.py:70-72): L2-norm clipping."""
    norm = math.sqrt(sum(g * g for g in grad))
    if norm <= thres:
        return list(grad)
    scale = thres / norm
    return [g * scale for g in grad]


def elem_grad_clip(grad, thres=1.0):
    """Deployment torch::clamp(grad,-1,1) (StateRL.cpp:633): per-element."""
    return [max(-thres, min(g, thres)) for g in grad]


def check(name, cond, detail=""):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}{'  ' + detail if detail else ''}")
    return cond


def main():
    ok = True
    tau = 0.05
    print("== Eq.22 displacement (non-zero yaw) ==")
    vx, vy, wz = 1.2, -0.4, 2.0
    xr, yr, _ = get_pos_integral_reference(vx, vy, wz, tau)
    xd, yd = pos_deployment(vx, vy, wz, tau)
    print(f"  reference: dx={xr:.6f} dy={yr:.6f}; deployment: dx={xd:.6f} dy={yd:.6f}")
    ok &= check("reference retains yaw-coupled terms (dx != vx*tau)",
                abs(xr - vx * tau) > 1e-6, f"dx_ref={xr:.6f} vs vx*tau={vx*tau:.6f}")
    ok &= check("deployment is first-order only (dx == vx*tau)", abs(xd - vx * tau) < 1e-9)
    ok &= check("reference and deployment DIFFER at non-zero yaw",
                abs(xr - xd) > 1e-6 or abs(yr - yd) > 1e-6,
                f"dx_diff={xr-xd:.6f} dy_diff={yr-yd:.6f}")
    # cross-check with the P1-03 recorded fixture numbers
    ok &= check("matches P1-03 nonzero_yaw expected (dx=0.061, dy=-0.017)",
                abs(xr - 0.061) < 1e-9 and abs(yr - (-0.017)) < 1e-9,
                f"ref=({xr:.3f},{yr:.3f})")

    print("== Eq.22 zero-yaw degenerate case ==")
    vx, vy, wz = 1.2, -0.4, 0.0
    xr, yr, _ = get_pos_integral_reference(vx, vy, wz, tau)
    xd, yd = pos_deployment(vx, vy, wz, tau)
    ok &= check("zero yaw: reference == first-order", abs(xr - xd) < 1e-9 and abs(yr - yd) < 1e-9,
                f"ref=({xr:.4f},{yr:.4f}) dep=({xd:.4f},{yd:.4f})")
    ok &= check("matches P1-03 zero_yaw expected (dx=0.06, dy=-0.02)",
                abs(xr - 0.06) < 1e-9 and abs(yr - (-0.02)) < 1e-9,
                f"ref=({xr:.3f},{yr:.3f})")

    print("== Eq.21 objective arithmetic (fixed ra/goal/twist) ==")
    ra, goal_x, goal_y, x_est, y_est = -0.03, 0.5, -0.2, 0.06, -0.017
    ra_pen, pos_pen, loss = objective(ra, goal_x, goal_y, x_est, y_est)
    print(f"  ra_pen={ra_pen:.4f} pos_pen={pos_pen:.6f} loss={loss:.6f}")
    ok &= check("ra penalty = lam*max(ra+2eps,0) = 10*max(0.07,0) = 0.7", abs(ra_pen - 0.7) < 1e-9,
                f"got {ra_pen:.4f}")
    ok &= check("pos penalty = 0.02*((x-gx)^2+(y-gy)^2)", abs(pos_pen - 0.02 * ((x_est - goal_x) ** 2 + (y_est - goal_y) ** 2)) < 1e-9)
    ok &= check("safe ra (ra=-0.2) -> ra_pen=0", abs(objective(-0.2, 0, 0, 0, 0)[0] - 0.0) < 1e-9)

    print("== Clamp boundary (component-wise bounds) ==")
    c = clamp_bounds(5.0, -5.0, 0.1)
    ok &= check("vx clipped to +1.5, vy to -0.3, wz within", c == (1.5, -0.3, 0.1), f"got {c}")
    c2 = clamp_bounds(0.0, 0.0, 0.0)
    ok &= check("interior unchanged", c2 == (0.0, 0.0, 0.0))

    print("== Gradient clip difference (reference L2 vs deployment per-element) ==")
    grad = [3.0, 4.0]  # L2 norm 5
    l2 = l2_grad_clip(grad, 1.0)
    el = elem_grad_clip(grad, 1.0)
    print(f"  L2 clip={l2}  element clip={el}")
    ok &= check("L2-norm clip scales to norm 1", abs(math.sqrt(l2[0]**2 + l2[1]**2) - 1.0) < 1e-9, f"l2={l2}")
    ok &= check("per-element clamp gives [1,1]", el == [1.0, 1.0])
    ok &= check("the two clip methods DIFFER on this gradient", l2 != el, f"l2={l2} el={el}")

    print(f"\nRESULT: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
