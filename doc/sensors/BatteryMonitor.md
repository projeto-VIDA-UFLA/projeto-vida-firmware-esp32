# BatteryMonitor

> `include/sensors/BatteryMonitor.h` · `src/sensors/BatteryMonitor.cpp`

## Visão geral

Mede a **tensão e a corrente do banco de baterias de tração** com um INA228 no I2C (GPIO 21/22, endereço 0x40) e estima o estado de carga (SoC) pela tensão. É um monitoramento para estudo do consumo: **não age na segurança** (o [SafetyManager](../safety/SafetyManager.md) não usa o estado).

Origem: a classe da branch `Alexandre` (01/10, só tensão e SoC), trazida para o firmware atual em 31/10 com a corrente, uma task própria e a reconexão do sensor.

## Onde é usada

| Quem | Método | Frequência |
|---|---|---|
| `TaskBattery` (núcleo 0, prioridade 1) | `update()` | 10 Hz (`BATTERY_MEASUREMENT_PERIOD_MS`) |
| [VidaRosNode](../comunication/VidaRosNode.md) → `/vida/sensors/battery` | `getSnapshot()` | a cada leitura nova (10 Hz) |

A leitura fica **fora da `TaskSensors`**: cada transação I2C bloqueia a task até terminar (até `BATTERY_I2C_TIMEOUT_MS`, 5 ms; o padrão do Arduino é 50 ms). Na `TaskSensors` (período de 2 ms) um sensor com defeito daria flag 64 e, passando de 50 ms, dispararia o [TaskSupervisor](../safety/TaskSupervisor.md) (flag 8192, retida até reiniciar). No núcleo 0, abaixo da `TaskRosCommunication`, um I2C lento só atrasa a própria leitura.

## Interface pública

| Método | Descrição |
|---|---|
| `BatteryMonitor(endereço, sda, scl, wire)` | Guarda endereço, pinos e barramento |
| `begin()` | Inicia o I2C e procura o INA228; falha não impede o boot |
| `update()` | Uma leitura de VBUS e VSHUNT (procura o sensor de novo se ele não respondeu) |
| `getSnapshot()` | Cópia coerente da última leitura |
| `vbusRawToVolts()`, `vshuntRawToVolts()`, `socFromVoltage()` (estáticos) | Conversões |

### Snapshot

| Campo | Descrição |
|---|---|
| `sample_timestamp_us` | Instante da leitura, válida ou não (0 = nenhuma ainda) |
| `voltage_v` | Tensão do banco, sem filtro |
| `current_a` | Corrente, `> 0` = descarga; NaN sem shunt configurado |
| `soc_percent` | SoC pela tensão filtrada |
| `state` | `Invalid` (0), `Normal` (1), `Emergency` (2, SoC ≤ 20%) |
| `valid` | Leitura utilizável |

## Funcionamento

1. **Detecção** (`probe()`): lê os registradores de identificação (fabricante `0x5449`, dispositivo `0x228x`) e grava o `ADC_CONFIG` = `0xBB6A`: conversão contínua só de VSHUNT e VBUS, 1052 µs por conversão e **média de 16** (~34 ms por leitura, filtra o chaveamento da controladora da scooter). A faixa do shunt fica no padrão, ±163,84 mV.
2. **Leitura:** VBUS (bits 23:4, 195,3125 µV/LSB) e VSHUNT (bits 23:4 em complemento de 2, 312,5 nV/LSB).
   - Tensão acima de 85 V (faixa do INA228) → leitura inválida.
   - Corrente = `BATTERY_CURRENT_SIGN × VSHUNT / BATTERY_SHUNT_RESISTANCE_OHM`; com o shunt em 0, NaN.
3. **SoC:** tensão → filtro passa-baixa (`BATTERY_VOLTAGE_FILTER_ALPHA` = 0,1, τ ≈ 1 s) → linear entre 35,8 V (0%) e 40 V (100%), limitado a [0, 100].
4. **Falha de comunicação:** leitura inválida (`valid = false`, NaN, estado `Invalid`, com o instante da tentativa) e, no ciclo seguinte, o sensor é procurado e configurado de novo. Assim o INA228 pode ser ligado com o firmware rodando, e um mau contato se recupera sozinho.

Nada é escrito na `Serial` (é o transporte do micro-ROS): a falha aparece no tópico, com `valid = false` a 10 Hz.

## Parâmetros (`config.h` e `pinout.h`)

| Parâmetro | Valor | Uso |
|---|---|---|
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | 21 / 22 | Barramento I2C |
| `INA228_I2C_ADDRESS` | 0x40 | A0 e A1 em GND |
| `BATTERY_MEASUREMENT_PERIOD_MS` | 100 ms | Período da `TaskBattery` |
| `BATTERY_I2C_FREQUENCY_HZ` | 400 kHz | Clock do I2C |
| `BATTERY_I2C_TIMEOUT_MS` | 5 ms | Timeout de cada transação |
| `BATTERY_SHUNT_RESISTANCE_OHM` | **0 (a preencher)** | Shunt da corrente; 0 = corrente NaN |
| `BATTERY_CURRENT_SIGN` | +1 | Sinal para a descarga sair positiva |
| `BATTERY_VOLTAGE_VALID_MAX_V` | 85 V | Acima disto a leitura é inválida |
| `BATTERY_SOC_FULL_V` / `BATTERY_SOC_EMPTY_V` | 40,0 / 35,8 V | Faixa linear do SoC |
| `BATTERY_SOC_EMERGENCY_PERCENT` | 20% | Estado `Emergency` |
| `BATTERY_VOLTAGE_FILTER_ALPHA` | 0,10 | Filtro da tensão usada no SoC |

## Concorrência

O estado da leitura (filtro, sensor encontrado) só é usado pela `TaskBattery`. O snapshot é escrito e lido sob `portMUX`.

## Pontos de atenção

- **Shunt a configurar:** a corrente só é publicada depois de preencher `BATTERY_SHUNT_RESISTANCE_OHM` com o shunt instalado. A corrente máxima medida é 163,84 mV / R (por exemplo, 218 A com um shunt de 75 mV/100 A, 0,75 mΩ). Conferir o sinal com o veículo tracionando e ajustar `BATTERY_CURRENT_SIGN`.
- **SoC pela tensão só vale em repouso:** sob carga a tensão cai e o SoC estimado cai junto (e volta ao soltar o acelerador). O estado `Emergency` pode piscar numa arrancada; por isso ele não age na segurança.
- **Faixa de 35,8–40 V** é a estimativa inicial do Alexandre; conferir com a curva de descarga do banco.
- **Sem teste no hardware** nesta versão do firmware: as conversões e a reconexão foram verificadas no PC com um INA228 simulado (31/10).
