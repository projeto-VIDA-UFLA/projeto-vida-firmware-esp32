# EncoderReader

> `include/sensors/EncoderReader.h` · `src/sensors/EncoderReader.cpp`

## Visão geral

Leitor de **encoder incremental em quadratura** com decodificação x4. Cada objeto representa um encoder físico. O VIDA tem três:

| Objeto | Roda | Pinos A/B | Ticks por volta |
|---|---|---|---|
| `rear_encoder` | eixo traseiro (motriz, sem diferencial) | 19 / 18 | `ENC_REAR_TICKS_PER_REV` = 1440 |
| `right_encoder` | dianteira direita | 35 / 34 | `ENC_FRONT_TICKS_PER_REV` = 730 |
| `left_encoder` | dianteira esquerda | 33 / 32 | `ENC_FRONT_TICKS_PER_REV` = 730 |

A ISR só conta ticks e grava o instante da última borda. Velocidade (pelo **método M/T**) e aceleração são calculadas em `update()`, chamado periodicamente.

## Onde é usada

| Quem | Método | Frequência |
|---|---|---|
| ISRs do `main.cpp` (canais A e B, `CHANGE`) | `handleInterrupt()` | a cada borda |
| `TaskSensors` | `update()` | 500 Hz |
| [VehicleMotion](VehicleMotion.md), [SafetyManager](../safety/SafetyManager.md), [VidaRosNode](../comunication/VidaRosNode.md) | `getMeasurementSnapshot()` | 200 Hz / 50 Hz |

## Interface pública

| Método | Descrição |
|---|---|
| `EncoderReader(pin_a, pin_b, tick_per_rev, name)` | Guarda a configuração; não acessa o hardware |
| `begin(isr = nullptr)` | Configura os pinos, lê o estado inicial e, com `isr`, liga a interrupção dos dois canais |
| `update()` | Calcula velocidade e aceleração desde o último `update()` |
| `getMeasurementSnapshot()` | Cópia coerente da última medição (ver abaixo) |
| `handleInterrupt()` | Decodifica a transição A/B e atualiza a contagem (chamada pela ISR) |
| `getCount()` | Contagem total de ticks |
| `getAndResetDelta()` | Ticks desde a última chamada, zerando o acumulador (sem uso hoje) |
| `reset()` | Zera contagem, velocidades e acelerações |
| `getVelocity()`, `getRawVelocity()` | Velocidade filtrada / bruta (m/s) |
| `getAcceleration()`, `getRawAcceleration()` | Aceleração filtrada / bruta (m/s²) |
| `getName()` | Nome do encoder (`"rear"`, `"front_right"`, `"front_left"`) |

### Snapshot

| Campo | Unidade | Descrição |
|---|---|---|
| `timestamp_us` | µs | Instante do `update()` que gerou a medição |
| `count` | ticks | Contagem acumulada nesse instante |
| `raw_velocity_m_s` | m/s | Velocidade M/T, atualizada a cada janela |
| `filtered_velocity_m_s` | m/s | Velocidade filtrada (`DRIVE_VELOCITY_FILTER_ALPHA`) |
| `raw_acceleration_m_s2` | m/s² | Derivada da velocidade filtrada |
| `filtered_acceleration_m_s2` | m/s² | Aceleração filtrada (`DRIVE_ACCEL_FILTER_ALPHA`) |

Todos os campos pertencem ao mesmo `update()`.

## Funcionamento

### Decodificação x4 (ISR)

O estado é codificado em 2 bits (`A<<1 | B`). O índice `estado_anterior<<2 | estado_atual` consulta uma tabela de 16 posições que devolve `+1` (avanço), `−1` (retorno) ou `0` (sem movimento ou transição inválida). Cada borda de A e de B gera uma interrupção, então um ciclo completo dos canais vale 4 ticks.

### Velocidade pelo método M/T (`update()` → `estimateVelocity()`)

Contar ticks num intervalo fixo de 2 ms gera uma quantização de ~0,4 m/s (traseiro) e ~0,8 m/s (dianteiros) por tick. O método M/T elimina isso medindo o **tempo exato entre bordas**:

```
janela: abre numa borda e fecha numa borda posterior, com
        duração ≥ ENCODER_VELOCITY_WINDOW_MS (20 ms) e nº de ticks múltiplo de 4
v_bruta = ticks_da_janela / ticks_por_volta × π × D / (t_borda_final − t_borda_inicial)
```

- Fechar só com **ciclos completos de quadratura** (múltiplo de 4) evita o erro das quatro bordas de um ciclo não serem igualmente espaçadas.
- Entre o fechamento de duas janelas, a última velocidade é mantida.
- **Desaceleração:** sem borda nova, a velocidade é limitada a "2 ticks desde a última borda" e cai suavemente.
- **Parado:** sem bordas por `ENCODER_ZERO_VELOCITY_TIMEOUT_MS` (200 ms) a velocidade é zero, e a próxima janela abre na próxima borda.

Depois do M/T:

```
v_filtrada = α_v · v_bruta + (1 − α_v) · v_filtrada_anterior        (α_v = 0,5)
a_bruta = (v_filtrada − v_filtrada_anterior) / dt
a_filtrada = α_a · a_bruta + (1 − α_a) · a_filtrada_anterior        (α_a = 0,01)
```

Em simulação (bordas desiguais em ±10% e jitter de 5 µs na ISR), o atraso da velocidade filtrada caiu de 39 ms (estimador antigo: Δticks em 2 ms + α = 0,05) para 25–28 ms, sem ruído de quantização nem viés.

O primeiro `update()` após `begin()`/`reset()` só cria a referência e não gera medição.

## Parâmetros (`config.h`)

| Parâmetro | Valor | Uso |
|---|---|---|
| `ENC_REAR_TICKS_PER_REV` | 1440 | Ticks x4 por volta da roda traseira |
| `ENC_FRONT_TICKS_PER_REV` | 730 | Ticks x4 por volta das rodas dianteiras |
| `DRIVE_WHEEL_DIAMETER_M` | 0,366 m | Diâmetro usado para os três encoders |
| `ENCODER_VELOCITY_WINDOW_MS` | 20 ms | Duração mínima da janela M/T |
| `ENCODER_ZERO_VELOCITY_TIMEOUT_MS` | 200 ms | Sem bordas por este tempo: velocidade zero |
| `DRIVE_VELOCITY_FILTER_ALPHA` | 0,5 | Filtro após o M/T |
| `DRIVE_ACCEL_FILTER_ALPHA` | 0,01 | Filtro da aceleração (τ ≈ 200 ms) |

## Concorrência

`count_`, `delta_` e a medição são protegidos por um `portMUX` compartilhado entre a ISR e as tasks. `getMeasurementSnapshot()` copia todos os campos dentro da mesma seção crítica. Os getters individuais (`getVelocity()` etc.) leem um `float` sem proteção.

## Pontos de atenção

- **Velocidades muito baixas:** abaixo de ~0,1 m/s há poucos ticks em 20 ms e a janela se estende até completar 4 ticks (no dianteiro, a 0,05 m/s, ~125 ms).
- **Escala:** no teste de 12/10/2026 os encoders mediram ~7% mais distância que o GPS. Conferir o diâmetro real de cada roda e se 730 é de fato a contagem x4 dos dianteiros (não é múltiplo de 4).
- `handleInterrupt()` não está em IRAM. Só faria diferença com escrita em flash durante a operação, o que o firmware não faz.
- Os pinos 34 e 35 (encoder direito) são só de entrada e não têm pull-up interno: dependem de pull-up no hardware.
