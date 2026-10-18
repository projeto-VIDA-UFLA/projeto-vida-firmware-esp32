import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10})

KP, KI, DB, FLOOR, IMAX, DT = 0.007, 0.1, 1.2, 0.15, 10.0, 0.005
DB_ENTER = 0.4

def static_u(e, integral=0.0):
    if abs(e) <= DB:
        return 0.0
    u = max(-1, min(1, KP * e + KI * integral))
    if u != 0 and abs(u) < FLOOR:
        u = np.copysign(FLOOR, u)
    return u

fig, (a1, a2) = plt.subplots(1, 2, figsize=(14, 5.6))
fig.suptitle("3. Comportamento do controlador de esterçamento (ganhos e zona morta atuais)", fontsize=15, fontweight="bold")

e = np.linspace(-30, 30, 2401)
a1.plot(e, KP * e, "--", color="#999999", label="só P (Kp·e), sem piso nem zona morta")
a1.plot(e, [static_u(x) for x in e], color="#2a6fb0", lw=2.2, label="comando real com integral = 0")
a1.axvspan(-DB, DB, color="#fbe5d3", alpha=0.8, label="zona morta: sai em ±1,2° (u = 0)")
a1.axvspan(-DB_ENTER, DB_ENTER, color="#f5c6a5", alpha=0.8, label="entra em ±0,4° (histerese)")
a1.axhline(FLOOR, color="#d35400", lw=0.8, ls=":"); a1.axhline(-FLOOR, color="#d35400", lw=0.8, ls=":")
a1.annotate("piso ±0,15 domina até |e| ≈ 21°\n(P só passa do piso com e > 0,15/0,007)", xy=(10, 0.15), xytext=(-27, 0.55),
            arrowprops=dict(arrowstyle="->", color="#d35400"), color="#d35400")
a1.annotate("salto de 0 para 0,15 na saída;\nsó volta a 0 abaixo de 0,4°\n(histerese)", xy=(1.2, 0.15), xytext=(3, -0.55),
            arrowprops=dict(arrowstyle="->", color="#c0392b"), color="#c0392b")
a1.set_xlabel("erro e = θref − θmedido (°)"); a1.set_ylabel("comando normalizado u")
a1.set_title("Característica estática (sem integral)"); a1.set_ylim(-1.05, 1.05); a1.grid(alpha=0.3)
a1.legend(loc="lower right", fontsize=8.5)

t = np.arange(0, 4.0, DT)
for err, c in [(3, "#2e8b57"), (10, "#2a6fb0"), (15, "#c0392b")]:
    I, us = 0.0, []
    for _ in t:
        cand_I = max(-IMAX, min(IMAX, I + err * DT))
        cand_u = KP * err + KI * cand_I
        if not (cand_u > 1 and cand_I > I):
            I = cand_I
        us.append(static_u(err, I))
    a2.plot(t, us, color=c, lw=2, label=f"erro de {err}° mantido")
a2.axhline(1.0, color="#999999", lw=0.8, ls=":")
a2.set_xlabel("tempo com o veículo DESABILITADO, motor parado (s)")
a2.set_ylabel("comando aplicado no instante da habilitação")
a2.set_title("Windup com o atuador desconectado — comportamento ANTERIOR")
a2.text(1.25, 0.04, "Antes: a integral acumulava com o motor parado\ne o motor partia com até 100% ao habilitar.\nCorrigido (47b6108): sem permissão a integral\nfica em zero e o comando inicial é só P + piso.", fontsize=9,
        bbox=dict(boxstyle="round", fc="#f8dcd8", ec="#c0392b"))
a2.set_ylim(0, 1.08); a2.grid(alpha=0.3); a2.legend(loc="center right", fontsize=9)

fig.tight_layout(rect=[0, 0, 1, 0.94])
fig.savefig("3_grafico.pdf"); fig.savefig("3.png", dpi=55)
