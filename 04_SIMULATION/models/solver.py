"""准静态求解与瞬态积分。

瞬态
----
用速度 Verlet（kick-drift-kick）固定步长积分。无阻尼时它是辛格式，能量误差
随步长二阶收敛，用来验证步长是否足够。阻尼按元件线粘性显式处理，阻尼较弱时
对精度影响可忽略。

步长选择：先按最高关注频率估 sqrt(k_eff/m_min)，取 dt 为其周期的 1/50 以下，
再用 dt 与 dt/2 两次积分比较输出峰值，相对变化小于 2% 认为收敛。

准静态
------
给定基座位移，解节点力平衡 R(x)=0。用牛顿法，雅可比由元件切线刚度拼装。
snap 发生时稳定平衡消失（雅可比出现非正特征值或牛顿不收敛），此时改用大阻尼
动态松弛让系统跳到新的稳定平衡，记录位移跳变。扫描基座位移即得到力—位移曲线、
稳定分支与 snap 阈值。
"""

from __future__ import annotations

import numpy as np


def integrate(chain, t_end, dt, base_pos_func, base_vel_func, x0=None, v0=None,
              record_every=1, ext_force_func=None):
    """速度 Verlet 积分。

    ext_force_func(t) 返回每个节点的外部力数组（可选），用于力控作动。
    返回字典：t, x, v, pe, ke。
    """
    n = chain.n
    x = np.zeros(n) if x0 is None else np.array(x0, dtype=float)
    v = np.zeros(n) if v0 is None else np.array(v0, dtype=float)
    steps = int(round(t_end / dt))
    n_rec = steps // record_every + 1
    t_hist = np.empty(n_rec)
    x_hist = np.empty((n_rec, n))
    v_hist = np.empty((n_rec, n))
    pe_hist = np.empty(n_rec)
    ke_hist = np.empty(n_rec)
    slot = 0

    def record(t):
        nonlocal slot
        t_hist[slot] = t
        x_hist[slot] = x
        v_hist[slot] = v
        ke = 0.5 * float(np.sum(chain.masses * v * v))
        pe_hist[slot] = chain.energy(x, v, base_pos_func(t), base_vel_func(t)) - ke
        ke_hist[slot] = ke
        slot += 1

    t = 0.0
    ef = None if ext_force_func is None else ext_force_func(t)
    a = chain.accelerations(x, v, base_pos_func(t), base_vel_func(t), ef)
    record(t)
    for step in range(1, steps + 1):
        v_half = v + 0.5 * dt * a
        x = x + dt * v_half
        t = step * dt
        ef = None if ext_force_func is None else ext_force_func(t)
        a = chain.accelerations(x, v_half, base_pos_func(t), base_vel_func(t), ef)
        v = v_half + 0.5 * dt * a
        if step % record_every == 0:
            record(t)
    return {
        "t": t_hist,
        "x": x_hist,
        "v": v_hist,
        "pe": pe_hist,
        "ke": ke_hist,
        "dt": dt,
    }


def convergence_check(make_chain, t_end, dt, base_pos_func, base_vel_func,
                      metric, tol=0.02):
    """用 dt 与 dt/2 两次积分比较某个标量 metric，判断步长收敛。"""
    m1 = metric(integrate(make_chain(), t_end, dt, base_pos_func, base_vel_func))
    m2 = metric(integrate(make_chain(), t_end, dt / 2.0, base_pos_func, base_vel_func))
    denom = abs(m2) if abs(m2) > 1e-30 else 1.0
    return abs(m1 - m2) / denom <= tol, m1, m2


def solve_static(chain, base_pos, x_guess, max_iter=60, tol=1e-10):
    """牛顿法解 R(x)=0。

    返回 (x, converged, hess_min)。R 对 x 的雅可比 jac 等于势能 Hessian 的负，
    所以稳定平衡对应 jac 全部特征值为负，hess_min = -max(eigvalsh(jac)) > 0。
    """
    x = np.array(x_guess, dtype=float)
    hess_min = np.nan
    for _ in range(max_iter):
        r, jac = chain.residual_jacobian(x, base_pos)
        try:
            eigs = np.linalg.eigvalsh(jac)
            hess_min = float(-eigs[-1])
        except np.linalg.LinAlgError:
            hess_min = np.nan
        try:
            step = np.linalg.solve(jac + 1e-9 * np.eye(chain.n), -r)
        except np.linalg.LinAlgError:
            return x, False, hess_min
        x = x + step
        if np.linalg.norm(step) < tol:
            return x, True, hess_min
    r, _ = chain.residual_jacobian(x, base_pos)
    return x, np.linalg.norm(r) < 1e-6, hess_min


def relax_dynamic(chain, base_pos, x_start, steps=4000, dt=1e-6, damping_scale=50.0,
                  ext_force=None):
    """大阻尼动态松弛，用来在 snap 后找到新的稳定平衡。"""
    x = np.array(x_start, dtype=float)
    v = np.zeros(chain.n)
    saved = list(chain.damping)
    chain.damping = [c * damping_scale + 1e-3 for c in saved]
    try:
        for _ in range(steps):
            a = chain.accelerations(x, v, base_pos, 0.0, ext_force)
            # 半隐式欧拉，强阻尼下稳定
            v = v + dt * a
            v *= 0.9
            x = x + dt * v
    finally:
        chain.damping = saved
    return x


def solve_static_ext(chain, ext_force, x_guess, max_iter=80, tol=1e-10):
    """力控静态平衡：G(x) = R(x) + F_ext = 0。

    ext_force 是施加在各节点上的外部力数组。返回 (x, converged, hess_min)，
    hess_min 为势能 Hessian 最小特征值，正值表示稳定平衡。
    """
    x = np.array(x_guess, dtype=float)
    fext = np.zeros(chain.n) if ext_force is None else np.asarray(ext_force, float)
    hess_min = np.nan
    for _ in range(max_iter):
        r, jac = chain.residual_jacobian(x, 0.0)
        g = r + fext
        try:
            eigs = np.linalg.eigvalsh(jac)
            hess_min = float(-eigs[-1])
        except np.linalg.LinAlgError:
            hess_min = np.nan
        try:
            step = np.linalg.solve(jac + 1e-9 * np.eye(chain.n), -g)
        except np.linalg.LinAlgError:
            return x, False, hess_min
        x = x + step
        if np.linalg.norm(step) < tol:
            return x, True, hess_min
    r, _ = chain.residual_jacobian(x, 0.0)
    g = r + fext
    return x, np.linalg.norm(g) < 1e-6, hess_min


def trace_static_force(chain, forces, x0=None, relax_steps=3000):
    """扫描外部力，返回力控准静态路径。

    记录 input_force（外部力）、节点位移、是否跳变、Hessian 最小特征值。
    跳变发生在稳定平衡消失处，跳变前一步的外力即 snap 阈值。
    """
    records = []
    x = np.zeros(chain.n) if x0 is None else np.array(x0, dtype=float)
    for F in forces:
        fext = np.zeros(chain.n)
        fext[0] = F
        x_new, ok, hess = solve_static_ext(chain, fext, x)
        jumped = False
        if (not ok) or (np.isfinite(hess) and hess <= 0.0):
            x_new = relax_dynamic(chain, 0.0, x, steps=relax_steps, ext_force=fext)
            _, ok2, hess2 = solve_static_ext(chain, fext, x_new)
            jumped = True
            if np.isfinite(hess2):
                hess = hess2
        records.append(
            {
                "force": float(F),
                "x": x_new.copy(),
                "jumped": jumped,
                "hess_min": float(hess) if np.isfinite(hess) else float("nan"),
            }
        )
        x = x_new
    return records


def trace_static(chain, base_positions, x0=None, relax_steps=3000):
    """扫描基座位移，返回准静态路径记录。

    每条记录包含 base_pos、节点位移、输入力、是否发生跳变、Hessian 最小特征值。
    出现跳变时把跳变后的状态记入该步，便于后续读取 snap 阈值（跳变前一步的力）。
    """
    records = []
    x = np.zeros(chain.n) if x0 is None else np.array(x0, dtype=float)
    for Xb in base_positions:
        x_new, ok, hess = solve_static(chain, Xb, x)
        jumped = False
        if (not ok) or (np.isfinite(hess) and hess <= 0.0):
            # 稳定平衡消失，动态松弛到新平衡
            x_new = relax_dynamic(chain, Xb, x, steps=relax_steps)
            _, ok2, hess2 = solve_static(chain, Xb, x_new)
            jumped = True
            if np.isfinite(hess2):
                hess = hess2
        f_in = chain.input_force(x_new, Xb)
        records.append(
            {
                "base_pos": float(Xb),
                "x": x_new.copy(),
                "force": float(f_in),
                "jumped": jumped,
                "hess_min": float(hess) if np.isfinite(hess) else float("nan"),
            }
        )
        x = x_new
    return records
