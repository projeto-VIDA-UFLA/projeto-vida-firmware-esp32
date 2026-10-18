# EncoderReader

> `include/sensors/EncoderReader.h` · `src/sensors/EncoderReader.cpp`

## Visão geral

Leitor de **encoder incremental em quadratura** com decodificação x4. Cada objeto representa um encoder físico. O VIDA tem três:

| Objeto | Roda | Pinos A/B | Ticks por volta |
|---|---|---|---|
| `rear_encoder` | eixo traseiro (motriz, sem diferencial) | 19 / 18 | `ENC_REAR_TICKS_PER_REV` = 1440 |
| `right_encoder` | dianteira direita | 35 / 34 | `ENC_FRONT_TICKS_PER_REV` = 730 |
| `left_encoder` | dianteira esquerda | 33 / 32 | `ENC_FRONT_TICKS_PER_REV` = 730 |

A ISR só conta ticks. Velocidade e aceleração são calculadas em `update()`, chamado periodicamente.

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
| `raw_velocity_m_s` | m/s | Velocidade sem filtro |
| `filtered_velocity_m_s` | m/s | Velocidade filtrada (`DRIVE_VELOCITY_FILTER_ALPHA`) |
| `raw_acceleration_m_s2` | m/s² | Derivada da velocidade filtrada |
| `filtered_acceleration_m_s2` | m/s² | Aceleração filtrada (`DRIVE_ACCEL_FILTER_ALPHA`) |

Todos os campos pertencem ao mesmo `update()`.

## Funcionamento

### Decodificação x4 (ISR)

O estado é codificado em 2 bits (`A<<1 | B`). O índice `estado_anterior<<2 | estado_atual` consulta uma tabela de 16 posições que devolve `+1` (avanço), `−1` (retorno) ou `0` (sem movimento ou transição inválida). Cada borda de A e de B gera uma interrupção, então um ciclo completo dos canais vale 4 ticks.

### Velocidade e aceleração (`update()`)

```
Δticks = count − count_anterior
distância = Δticks / ticks_por_volta × π × DRIVE_WHEEL_DIAMETER_M
v_bruta = distância / dt
v_filtrada = α_v · v_bruta + (1 − α_v) · v_filtrada_anterior        (α_v = 0,05)
a_bruta = (v_filtrada − v_filtrada_anterior) / dt
a_filtrada = α_a · a_bruta + (1 − α_a) · a_filtrada_anterior        (α_a = 0,01)
```

O primeiro `update()` após `begin()`/`reset()` só cria a referência e não gera medição.

## Parâmetros (`config.h`)

| Parâmetro | Valor | Uso |
|---|---|---|
| `ENC_REAR_TICKS_PER_REV` | 1440 | Ticks x4 por volta da roda traseira |
| `ENC_FRONT_TICKS_PER_REV` | 730 | Ticks x4 por volta das rodas dianteiras |
| `DRIVE_WHEEL_DIAMETER_M` | 0,366 m | Diâmetro usado para os três encoders |
| `DRIVE_VELOCITY_FILTER_ALPHA` | 0,05 | Filtro da velocidade (τ ≈ 39 ms a 500 Hz) |
| `DRIVE_ACCEL_FILTER_ALPHA` | 0,01 | Filtro da aceleração (τ ≈ 200 ms) |

## Concorrência

`count_`, `delta_` e a medição são protegidos por um `portMUX` compartilhado entre a ISR e as tasks. `getMeasurementSnapshot()` copia todos os campos dentro da mesma seção crítica. Os getters individuais (`getVelocity()` etc.) leem um `float` sem proteção.

## Pontos de atenção

- **Quantização da velocidade:** com Δt = 2 ms, um tick vale ~0,4 m/s no traseiro e ~0,8 m/s nos dianteiros. A velocidade bruta é dominada pela quantização e o filtro compensa com atraso. Para o controle longitudinal em malha fechada, a estimativa precisa ser feita por janela maior ou por período entre pulsos (a ISR já grava `last_timestamp_`, ainda sem uso).
- **Escala:** no teste de 12/10/2026 os encoders mediram ~7% mais distância que o GPS. Conferir o diâmetro real de cada roda e se 730 é de fato a contagem x4 dos dianteiros (não é múltiplo de 4).
- `handleInterrupt()` não está em IRAM. Só faria diferença com escrita em flash durante a operação, o que o firmware não faz.
- Os pinos 34 e 35 (encoder direito) são só de entrada e não têm pull-up interno: dependem de pull-up no hardware.
