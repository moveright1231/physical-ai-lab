"""ch08 PID 실습 - 교안 원안 vs 개선안 비교."""
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")          # WSLg GUI 불안정 -> 파일로 저장
import matplotlib.pyplot as plt


def sim(Kp, Ki, Kd, target=45.0, T=10.0, dt=0.05,
        load=0.0, decay=None, damp_tau=0.475, d_kick=True):
    """load=0, decay=0.9 이면 교안 원안과 완전히 동일."""
    if decay is None:
        decay = math.exp(-dt / damp_tau)
    pos = vel = integral = 0.0
    prev_error = 0.0 if d_kick else target
    hist = []
    for _ in range(int(T / dt)):
        error = target - pos
        integral += error * dt
        derivative = (error - prev_error) / dt
        u = Kp * error + Ki * integral + Kd * derivative
        vel += (u - load) * dt
        vel *= decay
        pos += vel * dt
        prev_error = error
        hist.append(pos)
    return hist


CASES = {"P only (1,0,0)": (1.0, 0.0, 0.0),
         "P big  (3,0,0)": (3.0, 0.0, 0.0),
         "PD   (3,0,0.5)": (3.0, 0.0, 0.5),
         "PID (3,.5,.5)":  (3.0, 0.5, 0.5)}

print("=" * 62)
print("[1] 교안 원안 그대로 (load=0, vel*=0.9), 10초")
print(f"{'case':18s}{'final':>10s}{'ss_err':>10s}{'max':>10s}{'over%':>9s}")
for n, (kp, ki, kd) in CASES.items():
    h = sim(kp, ki, kd, T=10, decay=0.9)
    print(f"{n:18s}{h[-1]:10.5f}{45-h[-1]:10.5f}{max(h):10.4f}"
          f"{(max(h)-45)/45*100:8.2f}%")

print()
print("[2] 같은 코드를 100초/1000초로 늘리면?  <- 핵심")
print(f"{'case':18s}{'10s':>12s}{'100s':>12s}{'1000s':>12s}")
for n, (kp, ki, kd) in CASES.items():
    r = [sim(kp, ki, kd, T=t, decay=0.9)[-1] for t in (10, 100, 1000)]
    print(f"{n:18s}{r[0]:12.6f}{r[1]:12.6f}{r[2]:12.6f}")
print("  -> P만으로도 정확히 45.000000. 없앨 정상상태 오차가 애초에 없음.")
print("     ch07 이론('P만으론 못 미친다')을 이 실습이 반증함.")

print()
print("[3] 개선안: 중력 부하 load=8.0 추가 -> 이론대로 동작")
print(f"{'Kp':>6s}{'final':>12s}{'잔여오차':>12s}{'이론 load/Kp':>16s}")
for kp in (1.0, 3.0, 6.0):
    h = sim(kp, 0.0, 0.0, T=60, load=8.0)
    print(f"{kp:6.1f}{h[-1]:12.5f}{45-h[-1]:12.5f}{8.0/kp:16.5f}")

print()
print("[4] I항이 그 잔여오차를 없애는가 (Kp=3, Kd=0.5, load=8)")
for ki in (0.0, 0.3, 0.6, 1.0):
    h = sim(3.0, ki, 0.5, T=60, load=8.0)
    print(f"  Ki={ki:<5.1f} final={h[-1]:9.5f}  잔여={45-h[-1]:+9.5f}"
          f"  max={max(h):8.3f}")

print()
print("[5] dt 의존성: 원안(vel*=0.9) vs 개선안(exp(-dt/tau))")
print(f"{'dt':>8s}{'원안 10s후':>14s}{'개선안 10s후':>16s}")
for d in (0.05, 0.025, 0.01):
    a = sim(3.0, 0.0, 0.5, T=10, dt=d, decay=0.9)[-1]
    b = sim(3.0, 0.0, 0.5, T=10, dt=d, load=8.0)[-1]
    print(f"{d:8.3f}{a:14.5f}{b:16.5f}")
print("  -> 원안은 dt를 바꾸면 물리가 통째로 달라짐(마찰이 아니라 스텝당 감쇠).")

print()
print("[6] D항 킥 (Kp=3, Kd=0.5, load=8)")
for kick, lab in ((True, "prev_error=0 (교안)"), (False, "prev_error=target")):
    h = sim(3.0, 0.0, 0.5, T=10, load=8.0, d_kick=kick)
    print(f"  {lab:22s} 1스텝후 pos={h[0]:8.4f}  max={max(h):8.3f}")
print(f"  1스텝 D항 기여 = {0.5*45/0.05:.0f}  vs  P항 기여 = {3*45:.0f}")

# ---- 그림 2장 ----
fig, ax = plt.subplots(1, 2, figsize=(13, 5))
t = np.arange(int(10/0.05)) * 0.05
for n, (kp, ki, kd) in CASES.items():
    ax[0].plot(t, sim(kp, ki, kd, T=10, decay=0.9), label=n)
    ax[1].plot(t, sim(kp, ki, kd, T=10, load=8.0), label=n)
for a, title in zip(ax, ["AS-IS (load=0)", "FIXED (load=8.0)"]):
    a.axhline(45, ls="--", c="gray"); a.set_title(title)
    a.set_xlabel("time (s)"); a.set_ylabel("angle (deg)")
    a.legend(fontsize=8); a.grid(alpha=.3)
plt.tight_layout(); plt.savefig("pid_lab.png", dpi=110)
print("\nsaved pid_lab.png")
