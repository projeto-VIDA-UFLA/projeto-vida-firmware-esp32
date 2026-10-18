"""Gera 4_fragilidades.dot (tabela de fragilidades do controle de esterçamento).

Uso: python3 4_fragilidades.py && dot -Tpdf 4_fragilidades.dot -o 4_fragilidades.pdf
"""
import textwrap

OK, PART, OPEN = "#d9f2e3", "#fff6d6", "#f8dcd8"

# (risco original, cor, título, o que acontecia, sugestão, situação, cor da situação)
rows = [
    ("ALTA", "#f8dcd8", "Windup com o atuador desconectado",
     "O controlador roda mesmo sem permissão de movimento; com o motor parado, a integral acumulava até saturar e o esterçamento partia com até 100% ao habilitar.",
     "Zerar a integral sem permissão.",
     "CORRIGIDO (47b6108): update() recebe a permissão; sem ela a integral fica em zero.", OK),
    ("ALTA", "#f8dcd8", "LVDT desconectado ou em curto não detectado",
     "valid só conferia 0 ≤ ADC ≤ 4095. ADC = 0 lê −28,9° e parece leitura real; o motor iria ao batente sem detecção.",
     "Validar faixa do ADC e sinal travado.",
     "MITIGADO (c826ce2): mediana de 3, ADC travado ±3 por 500 ms e fora de [40, 4055] por 10 ms invalidam. Entrada flutuante ainda não é detectada (plausibilidade, TODO).", PART),
    ("MÉDIA", "#fbe5d3", "Zona morta sem histerese + piso de 0,15",
     "Na borda de ±1,5° o comando saltava de 0 para 0,15. Visto no teste de 12/10: PWM alternando 0/38 a cada 100–300 ms.",
     "Histerese na zona morta.",
     "CORRIGIDO (6cd69e8): entra com |e| ≤ 0,4°, sai com |e| > 1,2°.", OK),
    ("MÉDIA", "#fbe5d3", "Zona morta grande para o Stanley",
     "Erro estacionário de até ±1,5° (curvatura ~0,023 1/m).",
     "Reduzir junto com a histerese.",
     "REDUZIDO: erro máximo 1,2°, típico ~0,4°. Validar em teste.", PART),
    ("MÉDIA", "#fbe5d3", "Integral zerada dentro da zona morta",
     "Ao entrar na zona morta o esforço de sustentação é descartado; se o mecanismo voltar sozinho, entra e sai ciclicamente.",
     "Manter a integral se o redutor for reversível.",
     "EM OBSERVAÇÃO: redutor com atrito, volta pouco. Observar em teste antes de mudar.", PART),
    ("MÉDIA", "#fbe5d3", "Fim de curso com limites não validados",
     "Limites de ±30° nunca atingidos; o lado direito chega ao fim físico em ~28,9°.",
     "Limites abaixo do fim físico.",
     "CORRIGIDO (ce4614a, f37fc87): trabalho ±20°, segurança ±25°, fundo de escala do LVDT ~28,5°.", OK),
    ("BAIXA", "#fff6d6", "Integral pode sozinha comandar 100%",
     "Limite de ±10 °·s × Ki 0,1 = ±1,0 (a escala inteira).",
     "Limitar a parcela integral (ex.: ±0,5).",
     "ABERTO (efeito reduzido com a correção do windup).", OPEN),
    ("BAIXA", "#fff6d6", "Kp baixo: malha dominada por I + piso",
     "P só passa do piso de 0,15 com erro acima de ~21°; na prática é relé ±0,15 + integral.",
     "Subir Kp e rever o piso em ensaio.",
     "ABERTO (sintonia).", OPEN),
    ("BAIXA", "#fff6d6", "Referência sem limite de taxa",
     "angular_velocity_deg_s é recebido e ignorado; degraus viram velocidade máxima do motor.",
     "Rate limiter usando esse campo.",
     "ABERTO.", OPEN),
    ("LATENTE", "#eeeeee", "Derivada sobre o erro, sem filtro",
     "Hoje Kd = 0. Se ativada, degraus de referência geram pico e ruído amplificado.",
     "Derivar a medição filtrada.",
     "ABERTO (inativo com Kd = 0).", OPEN),
]


def esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def wrap(s, w):
    return '<BR ALIGN="LEFT"/>'.join(esc(l) for l in textwrap.wrap(s, w)) + '<BR ALIGN="LEFT"/>'


head = ["#", "Risco", "Fragilidade", "O que acontecia", "Sugestão", "Situação"]
hdr = "<TR>" + "".join(f'<TD BGCOLOR="#2a6fb0"><FONT COLOR="white"><B>{h}</B></FONT></TD>' for h in head) + "</TR>"
body = ""
for i, (risk, color, title, what, sug, status, scolor) in enumerate(rows, 1):
    body += (f'<TR><TD BGCOLOR="{color}">{i}</TD><TD BGCOLOR="{color}"><B>{risk}</B></TD>'
             f'<TD ALIGN="LEFT" BALIGN="LEFT"><B>{wrap(title, 24)}</B></TD>'
             f'<TD ALIGN="LEFT" BALIGN="LEFT">{wrap(what, 52)}</TD>'
             f'<TD ALIGN="LEFT" BALIGN="LEFT">{wrap(sug, 26)}</TD>'
             f'<TD ALIGN="LEFT" BALIGN="LEFT" BGCOLOR="{scolor}">{wrap(status, 44)}</TD></TR>')
strong = ("Pontos fortes: entradas NaN/velhas resetam e invalidam a saída · dt fora de (0, 100 ms] reseta · "
          "anti-windup condicional na saturação · integral zerada sem permissão · fim de curso bloqueia só o sentido "
          "que piora e zera a integral · saída monitorada pelo SafetyManager (≤ 20 ms) · inversão do motor com tempo morto.")
dot = f'''digraph G {{
 graph [fontname="DejaVu Sans", fontsize=20, labelloc=t, pad=0.4,
  label=<<B>4. Fragilidades do controle de esterçamento e situação atual</B><BR/><FONT POINT-SIZE="12">branch pendente-teste/ajustes-pre-merge · verde = corrigido · amarelo = parcial/observar · vermelho = aberto</FONT><BR/> >];
 node [shape=plaintext, fontname="DejaVu Sans", fontsize=10];
 t [label=<<TABLE BORDER="1" CELLBORDER="1" CELLSPACING="0" CELLPADDING="5" COLOR="#bbbbbb">{hdr}{body}
 <TR><TD COLSPAN="6" ALIGN="LEFT" BGCOLOR="#d9f2e3">{wrap(strong, 190)}</TD></TR></TABLE>>];
}}'''
open("4_fragilidades.dot", "w").write(dot)
