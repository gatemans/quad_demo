#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Educational quadrotor simulation: quintic A->B flight (takeoff, translate, land).
Physics: Newton-Euler rigid body, "+" layout, FLU body frame, Hamilton quaternions
(scalar-first, active body->world), omega in body frame. Purely educational model:
ideal instant motors, no sensors/ground effect, constant wind disturbance only in [7,10] s.

Run:  python quadrotor_demo.py [--no-animation]
Outputs (in script directory): flight_path.png, flight_signals.png,
flight_animation.gif, simulation_data.npz
Requires: numpy, matplotlib (Pillow optional, needed for the GIF).
"""
import argparse
from pathlib import Path
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# ----------------------------- parameters -----------------------------------
G, M = 9.81, 1.2
L = 0.20
J = np.diag([0.020, 0.020, 0.035])
KF, KM = 1.8e-5, 2.5e-7
T_MAX_ROTOR, T_MAX_TOTAL = 8.0, 32.0
DRAG_K = 0.12
DISTURB = np.array([0.3, -0.2, 0.0])     # N, active only t in [7,10]
DT, T_END = 0.002, 18.0
KP = np.diag([2.5, 2.5, 5.0])
KD = np.diag([3.5, 3.5, 4.0])
KR = np.diag([0.8, 0.8, 0.35])
KW = np.diag([0.25, 0.25, 0.18])
YAW_REF = 0.0
# Mixer rows: [total thrust, roll(L*kf), pitch(-L*kf), yaw(km)]  ("+" layout,
# rotors at (L,0),(0,L),(-L,0),(0,-L); reaction +yaw on rotors 1,3)
MIXER = np.array([
    [KF,      KF,      KF,      KF],
    [0.0,     L * KF,  0.0,    -L * KF],
    [-L * KF, 0.0,     L * KF,  0.0],
    [KM,     -KM,      KM,     -KM]])
ROTOR_POS = [np.array([L, 0, 0]), np.array([0, L, 0]),
             np.array([-L, 0, 0]), np.array([0, -L, 0])]
# Waypoints and segment end times (piecewise quintic, zero endpoint vel/acc)
WP_T = [0.0, 1.0, 4.0, 5.0, 12.0, 13.0, 16.0, 18.0]
WP_P = np.array([[0,0,0],[0,0,0],[0,0,2],[0,0,2],[3,2,2],[3,2,2],[3,2,0],[3,2,0]], float)

# ---------------------- quaternion helpers (Hamilton, scalar first) ----------
def quat_mul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2,
                     w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2])

def quat_from_axis_angle(axis, angle):
    a = np.asarray(axis, float); norm = np.linalg.norm(a)
    if norm < 1e-15:
        return np.array([1.0, 0.0, 0.0, 0.0])
    a = a / norm
    s = np.sin(angle / 2.0)
    return np.array([np.cos(angle / 2.0), a[0]*s, a[1]*s, a[2]*s])

def quat_normalize(q):
    return q / np.linalg.norm(q)

def quat_to_R(q):
    w, x, y, z = quat_normalize(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y - w*z),   2*(x*z + w*y)],
        [2*(x*y + w*z), 1-2*(x*x+z*z),   2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x),   1-2*(x*x+y*y)]])

def quat_integrate(q, omega_body, dt):
    """q_new = q (x) deltaq(omega*dt), then normalize."""
    dq = quat_from_axis_angle(omega_body, np.linalg.norm(omega_body)*dt)
    return quat_normalize(quat_mul(q, dq))

def R_to_euler(R):
    """ZYX yaw-pitch-roll Euler angles (rad) from rotation matrix."""
    pitch = -np.arcsin(np.clip(R[2, 0], -1, 1))
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return np.array([roll, pitch, yaw])

def vee(S):
    return np.array([S[2, 1], S[0, 2], S[1, 0]])

# ------------------------------ reference trajectory -------------------------
def quintic(u):
    s = u*u*u*(10 - 15*u + 6*u*u)
    sd = 30*u*u*(1 - u)*(1 - u)
    sdd = 60*u*(1 - 3*u + 2*u*u)
    return s, sd, sdd

def reference(t):
    """Return (pos, vel, acc, yaw) of the piecewise-quintic reference."""
    t = min(max(t, WP_T[0]), WP_T[-1])
    for i in range(len(WP_T) - 1):
        if t <= WP_T[i + 1] or i == len(WP_T) - 2:
            t0, t1 = WP_T[i], WP_T[i + 1]
            p0, p1 = WP_P[i], WP_P[i + 1]
            u = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
            s, sd, sdd = quintic(u)
            pos = p0 + s * (p1 - p0)
            vel = sd * (p1 - p0) / (t1 - t0)
            acc = sdd * (p1 - p0) / (t1 - t0) ** 2
            return pos, vel, acc, YAW_REF
    raise RuntimeError

# ------------------------------ simulation core ------------------------------
def simulate():
    n = int(round(T_END / DT)) + 1
    pos = np.zeros(n*0 or 3); pos[:] = WP_P[0]     # start at A on ground
    q = np.array([1.0, 0.0, 0.0, 0.0])
    v = np.zeros(3); omega = np.zeros(3)
    log = {k: np.zeros((n, 3)) for k in
           ("t", "pos", "vel", "acc", "q", "omega", "tau", "dist")}
    log["t"] = np.zeros(n); log["tau"] = np.zeros((n, 3)); log["q"] = np.zeros((n, 4))
    log["rotor"] = np.zeros((n, 4)); log["Tcmd"] = np.zeros(n)
    log["Tact"] = np.zeros(n); log["eR"] = np.zeros((n, 3)); log["euler"] = np.zeros((n, 3))
    disarm_t = None

    for i in range(n):
        t = i * DT
        R = quat_to_R(q)
        p_ref, v_ref, a_ref, yaw = reference(t)
        # --- outer loop: position -> force (with known-drag feedforward) ---
        ac = a_ref + KP @ (p_ref - pos) + KD @ (v_ref - v) + DRAG_K * v
        Fc = M * (ac + G * np.array([0, 0, 1.0]))
        b3d = Fc / np.linalg.norm(Fc)
        b1c = np.array([np.cos(yaw), np.sin(yaw), 0.0])
        b2d = np.cross(b3d, b1c); b2d /= np.linalg.norm(b2d)
        b1d = np.cross(b2d, b3d)
        Rd = np.column_stack([b1d, b2d, b3d])
        Tcmd = np.clip(Fc @ R[:, 2], 0.0, T_MAX_TOTAL)
        # --- inner loop: geometric attitude PD (desired rate simplified to 0) ---
        eR = 0.5 * vee(Rd.T @ R - R.T @ Rd)
        tau = -KR @ eR - KW @ omega + np.cross(omega, J @ omega)
        # --- mixer: solve then clip squared rates (simple educational allocation) ---
        rates2 = np.linalg.solve(MIXER, np.r_[Tcmd, tau])   # [w1^2..w4^2]
        sat = np.clip(rates2, 0.0, T_MAX_ROTOR / KF)
        T_rotor = KF * sat
        T_act = T_rotor.sum()
        tau_act = (MIXER @ sat)[1:]
        # --- wind disturbance window ---
        dist = DISTURB if 7.0 <= t <= 10.0 else np.zeros(3)
        # --- rigid body dynamics ---
        F_world = -M * G * np.array([0.0, 0.0, 1.0]) + T_act * R[:, 2] - DRAG_K * v + dist
        a = F_world / M
        omega_dot = np.linalg.inv(J) @ (tau_act - np.cross(omega, J @ omega))
        # --- semi-implicit integration ---
        v_new = v + a * DT
        pos_new = pos + v * DT + 0.5 * a * DT * DT         # consistent constant-acceleration step
        omega = omega + omega_dot * DT
        q = quat_integrate(q, omega, DT)
        v = v_new
        # --- ground contact: z>=0 cancels downward vz at z=0 (normal force implicit)
        if pos_new[2] < 0.0:
            pos_new[2] = 0.0
            v_new = v_new.copy()
            if v_new[2] < 0.0:
                v_new[2] = 0.0
        # --- disarm rule (small contact tolerance) ---
        if t >= 15.5 and pos_new[2] <= 0.015 and abs(v_new[2]) < 0.15 and disarm_t is None:
            if t >= 16.0:   # final descent complete -> snap to contact
                pos_new[2] = 0.0; v_new[:] = 0.0
                disarm_t = t
        if disarm_t is not None and t >= disarm_t:
            pos_new[2] = 0.0; v_new[:] = 0.0
            omega[:] = 0.0
            T_rotor[:] = 0.0; T_act = 0.0; tau_act = np.zeros(3)
            q = quat_normalize(q)
        # --- log ---
        for k, val in (("pos", pos_new), ("vel", v_new), ("acc", a), ("q", q),
                       ("omega", omega), ("tau", tau_act), ("dist", dist),
                       ("euler", R_to_euler(quat_to_R(q))), ("eR", eR)):
            log[k][i] = val
        log["t"][i] = t
        log["rotor"][i] = T_rotor
        log["Tcmd"][i] = Tcmd
        log["Tact"][i] = T_act
        pos = pos_new; v = v_new
    return log, disarm_t

# ------------------------------ plots ----------------------------------------
def make_plots(log):
    t = log["t"]
    t_ref = np.linspace(0, T_END, 1200)
    ref = np.array([reference(ti)[0] for ti in t_ref])
    # 1) 3D path
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(ref[:, 0], ref[:, 1], ref[:, 2], "r--", label="reference")
    ax.plot(log["pos"][:, 0], log["pos"][:, 1], log["pos"][:, 2], "b-", lw=1.5, label="actual")
    ax.scatter(*WP_P[0], color="g", s=60, label="A (start)")
    ax.scatter(*WP_P[4], color="m", s=60, label="B (hover)")
    ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]"); ax.set_zlabel("Z [m]")
    ax.set_zlim(0, 2.3); ax.legend()
    ax.set_title("Quadrotor A->B flight path")
    fig.tight_layout(); fig.savefig("flight_path.png", dpi=120); plt.close(fig)
    # 2) signal grid 4x2
    fig, axs = plt.subplots(4, 2, figsize=(12, 12), sharex=True)
    axs[0, 0].plot(t_ref, ref, "r--"); axs[0, 0].plot(t, log["pos"])
    axs[0, 0].set_ylabel("pos [m]"); axs[0, 0].legend(["ref x", "ref y", "ref z", "x", "y", "z"], fontsize=7)
    axs[0, 1].plot(t, log["vel"]); axs[0, 1].set_ylabel("vel [m/s]")
    err = np.linalg.norm(np.array([reference(ti)[0] for ti in t]) - log["pos"], axis=1)
    axs[0, 0].set_title("position ref vs actual / velocity")
    axs[1, 0].plot(t, np.degrees(log["euler"]))
    axs[1, 0].set_ylabel("Euler [deg]"); axs[1, 0].legend(["roll", "pitch", "yaw"], fontsize=7)
    axs[1, 1].plot(t, log["rotor"])
    axs[1, 1].set_ylabel("rotor thrust [N]"); axs[1, 1].legend([f"T{i+1}" for i in range(4)], fontsize=7)
    axs[2, 0].plot(t, log["Tact"], label="actual total T")
    axs[2, 0].plot(t, log["Tcmd"], "--", label="requested T")
    axs[2, 0].axhline(M * G, color="k", ls=":", label="m g")
    axs[2, 0].set_ylabel("total thrust [N]"); axs[2, 0].legend(fontsize=7)
    axs[2, 1].plot(t, err); axs[2, 1].set_ylabel("pos error norm [m]")
    axs[3, 0].plot(t, np.linalg.norm(log["vel"], axis=1)); axs[3, 0].set_ylabel("speed [m/s]")
    axs[3, 0].set_xlabel("t [s]")
    axs[3, 1].plot(t, log["acc"]); axs[3, 1].set_ylabel("acc [m/s^2]")
    axs[3, 1].set_xlabel("t [s]")
    fig.suptitle("Quadrotor flight signals")
    fig.tight_layout(); fig.savefig("flight_signals.png", dpi=120); plt.close(fig)

# ------------------------------ animation ------------------------------------
def make_animation(log, n_frames=120, max_px=800):
    from PIL import Image
    t = log["t"]
    t_ref = np.linspace(0, T_END, 200)
    ref = np.array([reference(ti)[0] for ti in t_ref])
    idx = np.linspace(0, len(t) - 1, n_frames).astype(int)
    frames, figs = [], []
    for k, i in enumerate(idx):
        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, projection="3d")
        ax.set_xlim(-0.5, 3.5); ax.set_ylim(-0.5, 2.5); ax.set_zlim(0, 2.3)
        # floor grid
        xx, yy = np.meshgrid(np.linspace(-0.5, 3.5, 5), np.linspace(-0.5, 2.5, 5))
        ax.plot_wireframe(xx, yy, np.zeros_like(xx), color="0.85", lw=0.5)
        ax.plot(ref[:, 0], ref[:, 1], ref[:, 2], "r--", lw=1, label="ref")
        tr = log["pos"][:i+1:20]
        ax.plot(tr[:, 0], tr[:, 1], tr[:, 2], "b-", lw=1.2, label="trace")
        p = log["pos"][i]; R = quat_to_R(log["q"][i])
        # body cross (FLU axes) and rotor markers via actual R
        ax.quiver(*p, *(R @ np.array([0.4, 0, 0])), color="g")
        ax.quiver(*p, *(R @ np.array([0, 0.4, 0])), color="orange")
        ax.quiver(*p, *(R @ np.array([0, 0, 0.4])), color="k")
        for rp in ROTOR_POS:
            w = p + R @ rp
            ax.scatter(*w, color="r", s=30)
        ax.set_title(f"t = {t[i]:.2f} s")
        ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
        ax.legend(fontsize=7)
        fig.canvas.draw()
        img = np.asarray(fig.canvas.buffer_rgba())[..., :3].copy()
        # downscale to max width
        h, wd = img.shape[:2]
        if wd > max_px:
            sc = max_px / wd
            img = np.asarray(Image.fromarray(img).resize((max_px, int(h*sc))))
        frames.append(Image.fromarray(img))
        plt.close(fig)
    dur = int(round((t[idx[1]] - t[idx[0]]) * 1000)) if n_frames > 1 else 50
    frames[0].save("flight_animation.gif", save_all=True,
                   append_images=frames[1:], duration=dur, loop=0)

# ------------------------------ validation -----------------------------------
def validate(log, disarm_t):
    q = log["q"]
    qn = np.linalg.norm(q, axis=1)
    R = np.array([quat_to_R(qi) for qi in q[:: max(1, len(q)//200)]])
    orth = max(np.max(np.abs(Ri @ Ri.T - np.eye(3))) for Ri in R)
    dets = [np.linalg.det(Ri) for Ri in R]
    t = log["t"]; mask = (t >= 1) & (t <= 16)
    pos_err = np.linalg.norm(np.array([reference(ti)[0] for ti in t[mask]]) - log["pos"][mask], axis=1)
    fin = log["pos"][-1]
    # quaternion identity example: [0,0,pi/2] for 1 s
    q0 = np.array([1.0, 0, 0, 0])
    q1 = quat_integrate(q0, np.array([0, 0, np.pi/2]), 1.0)
    return {
        "max_|q|-1": float(np.max(np.abs(qn - 1))),
        "max_orthogonality_err": float(orth),
        "det_R_min/max": (float(min(dets)), float(max(dets))),
        "quat_example_[0,0,pi/2]_1s": [float(x) for x in q1],
        "max_pos_err_t1_16": float(pos_err.max()),
        "rms_pos_err_t1_16": float(np.sqrt((pos_err**2).mean())),
        "final_pos": [float(x) for x in fin],
        "final_horiz_err": float(np.linalg.norm(fin[:2] - WP_P[-1][:2])),
        "max_tilt_deg": float(np.degrees(np.arccos(np.clip(
            [quat_to_R(qi)[2, 2] for qi in q], -1, 1)).max())),
        "rotor_min/max": (float(log["rotor"].min()), float(log["rotor"].max())),
        "per_motor_thrust_min_N": [float(x) for x in log["rotor"].min(axis=0)],
        "per_motor_thrust_max_N": [float(x) for x in log["rotor"].max(axis=0)],
        "per_motor_rpm_min": [float(x) for x in (np.sqrt(log["rotor"].min(axis=0) / KF) * 60/(2*np.pi))],
        "per_motor_rpm_max": [float(x) for x in (np.sqrt(log["rotor"].max(axis=0) / KF) * 60/(2*np.pi))],
        "total_T_min/max": (float(log["Tact"].min()), float(log["Tact"].max())),
        "rpm_conversion": "omega_rad_s * 60/(2*pi)",
        "max_rpm": float(np.sqrt((log["rotor"] / KF).max()) * 60.0 / (2.0 * np.pi)),
        "saturation_frac": float((log["rotor"] >= T_MAX_ROTOR - 1e-9).mean()),
        "disarm_time_s": disarm_t,
        "nonfinite": bool(not np.isfinite(np.concatenate([log["pos"].ravel(), log["vel"].ravel(), q.ravel()])).all()),
        "params": {"m": M, "g": G, "l": L, "J": J.tolist(), "kf": KF, "km": KM,
                   "dt": DT, "Kp": KP.tolist(), "Kd": KD.tolist(),
                   "KR": KR.tolist(), "KW": KW.tolist(),
                   "disturbance_N": DISTURB.tolist(), "drag_k": DRAG_K}}

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-animation", action="store_true")
    args = ap.parse_args()
    import time
    os.chdir(Path(__file__).resolve().parent)
    t0 = time.time()
    log, disarm_t = simulate()
    print(f"sim done in {time.time()-t0:.2f}s, disarm at t={disarm_t}")
    make_plots(log)
    if not args.no_animation:
        make_animation(log)
    np.savez("simulation_data.npz", **{k: v for k, v in log.items()},
             disarm_t=disarm_t if disarm_t is not None else -1)
    metrics = validate(log, disarm_t)
    import json
    print(json.dumps(metrics, indent=2))
