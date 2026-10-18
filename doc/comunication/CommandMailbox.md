# CommandMailbox

> `include/comunication/CommandMailbox.h` · `src/comunication/CommandMailbox.cpp`

## Visão geral

"Caixa de correio" entre a comunicação ROS e as tasks de controle e segurança. Guarda a **última mensagem recebida** de cada tipo, junto com o **instante local de chegada**.

- **Quem escreve:** os callbacks do [VidaRosNode](VidaRosNode.md), na `TaskRosCommunication` (núcleo 0).
- **Quem lê:** a `TaskControl` (controladores) e a `TaskSafety` ([SafetyManager](../safety/SafetyManager.md)), no núcleo 1.

## Interface pública

| Método | Chamado por |
|---|---|
| `updateSteeringReference(angle_deg, angular_velocity_deg_s, source_timestamp_ns)` | callback de `/vida/control/steering/reference` |
| `updateDriveReference(velocity_m_s, acceleration_m_s2, brake_command, source_timestamp_ns)` | callback de `/vida/control/drive/reference` |
| `updateLogicalEnable(enabled, source_timestamp_ns)` | callback de `/vida/safety/logical_enable` |
| `getSteeringReferenceSnapshot()` | `TaskControl`, `TaskSafety` |
| `getDriveReferenceSnapshot()` | `TaskControl`, `TaskSafety` |
| `getLogicalEnableSnapshot()` | `TaskSafety` |

## Snapshots

Campos comuns a todos:

| Campo | Descrição |
|---|---|
| `source_timestamp_ns` | Timestamp da mensagem (relógio ROS). Guardado só para diagnóstico |
| `received_at_us` | Instante de chegada no ESP32 (`esp_timer`). **É o que os watchdogs usam** |
| `received` | `false` até chegar a primeira mensagem |

| Snapshot | Campos específicos |
|---|---|
| `SteeringReferenceSnapshot` | `angle_deg` (REP-103), `angular_velocity_deg_s` (ainda sem uso) |
| `DriveReferenceSnapshot` | `velocity_m_s` (m/s), `acceleration_m_s2` (aceleração máxima da rampa; 0 = padrão), `brake_command` (> 0,05 = freio manual) |
| `LogicalEnableSnapshot` | `enabled` |

## Por que a idade usa o instante de chegada

O relógio do ESP32 só é sincronizado com o do Odroid depois da conexão, e a sincronização pode oscilar. Medir a idade pelo relógio local (`agora − received_at_us`) não depende dessa sincronização e detecta exatamente o que importa: há quanto tempo o ESP32 não recebe aquela referência.

## Concorrência

Cada tipo de mensagem tem o seu `portMUX`. Escrita e leitura copiam a estrutura inteira dentro da seção crítica, então quem lê nunca vê uma mensagem pela metade.
