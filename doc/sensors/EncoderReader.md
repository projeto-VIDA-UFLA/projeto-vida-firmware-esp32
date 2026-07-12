# Documentação da Classe `EncoderReader`

## 1. Visão geral

A classe `EncoderReader` é responsável por encapsular a leitura e o processamento de um encoder incremental em quadratura.

Ela foi pensada para uso em firmware embarcado no ESP32 com Arduino Framework, principalmente em aplicações onde é necessário medir deslocamento, velocidade e aceleração a partir de encoders rotativos.

Cada objeto da classe representa um encoder físico conectado a dois pinos digitais, normalmente chamados de canal A e canal B.

A classe realiza as seguintes funções principais:

- leitura dos canais A e B do encoder;
- identificação do sentido de rotação;
- contagem acumulada de ticks;
- cálculo de delta de ticks;
- cálculo de velocidade linear em metros por segundo;
- cálculo de aceleração linear em metros por segundo ao quadrado;
- armazenamento de valores brutos e filtrados;
- proteção de variáveis compartilhadas entre interrupções e tasks.

---

## 2. Contexto de uso

A classe é adequada para sistemas em que a leitura do encoder precisa ser feita em tempo real, geralmente usando interrupções.

No ESP32, o encoder pode gerar pulsos rapidamente. Por isso, a contagem dos ticks deve ser feita dentro de uma rotina de interrupção, enquanto os cálculos mais pesados, como velocidade e aceleração, devem ser feitos em uma função chamada periodicamente por uma task.

A separação recomendada é:

- `handleInterrupt()`: chamada pela interrupção, deve ser rápida e simples;
- `update()`: chamada por uma task, calcula velocidade e aceleração;
- getters: usados por outras partes do firmware para consultar os dados calculados.

---

## 3. Declaração da classe

```cpp
class EncoderReader
{
public:
    EncoderReader(uint8_t pin_a, uint8_t pin_b, float tick_per_rev, const char* name);

    void begin();
    void update();
    long getCount();
    void getSnapshot(int64_t &count, int64_t &timestamp_us);
    long getAndResetDelta();
    void reset();

    float getVelocityMps() const;
    float getAccelerationMps2() const;
    float getRawVelocityMps() const;
    float getRawAccelerationMps2() const;

    void handleInterrupt();

    const char* getName() const;

private:
    uint8_t pin_a_;
    uint8_t pin_b_;
    float tick_per_rev_;
    const char* name_;

    volatile int64_t count_;
    volatile long delta_;
    volatile int64_t last_timestamp_us_;
    volatile uint8_t last_state_;

    int64_t previous_count_;
    int64_t previous_update_us_;
    float velocity_m_s_;
    float acceleration_m_s2_;
    float raw_velocity_m_s_;
    float raw_acceleration_m_s2_;
    float previous_velocity_m_s_;
    bool has_previous_update_;

    portMUX_TYPE mux_;

    uint8_t readState() const;
    float ticksToDistanceM(int64_t ticks) const;
    float calculateVelocityMps(int64_t delta_ticks, float dt_s) const;
    float calculateAccelerationMps2(float current_velocity_m_s, float previous_velocity_m_s, float dt_s) const;
    float applyLowPassFilter(float raw_value, float filtered_value, float alpha) const;
};
```

---

## 4. Construtor

```cpp
EncoderReader(uint8_t pin_a, uint8_t pin_b, float tick_per_rev, const char* name);
```

Cria uma instância da classe para um encoder específico.

### Parâmetros

| Parâmetro | Tipo | Descrição |
|---|---|---|
| `pin_a` | `uint8_t` | Pino conectado ao canal A do encoder. |
| `pin_b` | `uint8_t` | Pino conectado ao canal B do encoder. |
| `tick_per_rev` | `float` | Quantidade de ticks por revolução do encoder. |
| `name` | `const char*` | Nome usado para identificar o encoder em logs ou debug. |

### Observação

O construtor apenas armazena as configurações iniciais. A configuração dos pinos e estados internos deve ser feita em `begin()`.

---

## 5. Métodos públicos

### 5.1 `begin()`

```cpp
void begin();
```

Inicializa o encoder.

Essa função deve ser chamada antes do uso da classe, normalmente dentro do `setup()` ou durante a etapa de inicialização dos periféricos do firmware.

Responsabilidades esperadas:

- configurar os pinos dos canais A e B;
- ler o estado inicial do encoder;
- inicializar contadores internos;
- preparar variáveis usadas nos cálculos de velocidade e aceleração.

---

### 5.2 `update()`

```cpp
void update();
```

Atualiza os cálculos de velocidade e aceleração.

Essa função deve ser chamada periodicamente por uma task. Ela compara a contagem atual com a contagem anterior, calcula o intervalo de tempo decorrido e estima a velocidade linear.

A aceleração é calculada a partir da diferença entre a velocidade atual e a velocidade anterior.

Essa função não deve ser chamada dentro da interrupção.

---

### 5.3 `getCount()`

```cpp
long getCount();
```

Retorna a contagem total acumulada de ticks.

Essa contagem representa o deslocamento acumulado do encoder desde a inicialização ou desde a última chamada de `reset()`.

Como a variável de contagem pode ser alterada por uma interrupção, a implementação deve fazer a leitura de forma protegida.

---

### 5.4 `getSnapshot()`

```cpp
void getSnapshot(int64_t &count, int64_t &timestamp_us);
```

Retorna uma cópia segura da contagem e do timestamp da última transição.

Esse método é útil quando é necessário obter dois valores relacionados de forma consistente.

Sem essa leitura protegida, poderia acontecer de a contagem ser lida antes de uma interrupção e o timestamp depois, gerando um par de dados incoerente.

---

### 5.5 `getAndResetDelta()`

```cpp
long getAndResetDelta();
```

Retorna o delta de ticks acumulado desde a última chamada e zera esse acumulador.

Esse método é útil quando alguma parte do firmware precisa trabalhar com deslocamento incremental em vez de contagem total.

Exemplos de uso:

- medir ticks por intervalo;
- publicar deslocamento incremental;
- alimentar uma malha de controle;
- estimar movimento entre ciclos de execução.

---

### 5.6 `reset()`

```cpp
void reset();
```

Zera os contadores e estados internos do encoder.

Pode ser usado em rotinas de inicialização, calibração ou troca de estado operacional do firmware.

Responsabilidades esperadas:

- zerar `count_`;
- zerar `delta_`;
- reiniciar variáveis de cálculo;
- limpar histórico de velocidade e aceleração.

---

### 5.7 `getVelocityMps()`

```cpp
float getVelocityMps() const;
```

Retorna a velocidade linear filtrada em metros por segundo.

Esse valor é indicado para uso em malhas de controle, pois tende a ser mais estável do que a velocidade bruta.

---

### 5.8 `getAccelerationMps2()`

```cpp
float getAccelerationMps2() const;
```

Retorna a aceleração linear filtrada em metros por segundo ao quadrado.

Como aceleração é uma grandeza derivada da velocidade, ela pode apresentar bastante ruído. Por isso, o valor filtrado é geralmente mais útil para controle ou monitoramento.

---

### 5.9 `getRawVelocityMps()`

```cpp
float getRawVelocityMps() const;
```

Retorna a velocidade bruta, antes da aplicação do filtro passa-baixa.

Esse valor é útil para debug, validação dos cálculos e ajuste de parâmetros do filtro.

---

### 5.10 `getRawAccelerationMps2()`

```cpp
float getRawAccelerationMps2() const;
```

Retorna a aceleração bruta, antes da aplicação do filtro.

Esse valor pode ser ruidoso, mas é útil para análise e comparação com a aceleração filtrada.

---

### 5.11 `handleInterrupt()`

```cpp
void handleInterrupt();
```

Trata uma interrupção gerada pelo encoder.

Essa função deve ser chamada pela ISR associada aos canais do encoder.

Responsabilidades esperadas:

- ler o estado atual dos canais A e B;
- comparar o estado atual com o estado anterior;
- identificar o sentido de rotação;
- incrementar ou decrementar a contagem;
- atualizar o delta;
- salvar o timestamp da transição.

Como roda em contexto de interrupção, essa função deve ser mantida rápida. Não é recomendado usar `Serial.print`, cálculos pesados, alocação dinâmica ou funções bloqueantes dentro dela.

---

### 5.12 `getName()`

```cpp
const char* getName() const;
```

Retorna o nome identificador do encoder.

Esse nome pode ser usado em logs, mensagens de debug, tópicos de comunicação ou diferenciação entre múltiplos encoders.

---

## 6. Atributos privados

### 6.1 Configuração física

| Atributo | Tipo | Descrição |
|---|---|---|
| `pin_a_` | `uint8_t` | Pino do canal A. |
| `pin_b_` | `uint8_t` | Pino do canal B. |
| `tick_per_rev_` | `float` | Ticks por revolução. |
| `name_` | `const char*` | Nome lógico do encoder. |

---

### 6.2 Dados atualizados pela interrupção

| Atributo | Tipo | Descrição |
|---|---|---|
| `count_` | `volatile int64_t` | Contagem total acumulada de ticks. |
| `delta_` | `volatile long` | Delta acumulado desde a última leitura incremental. |
| `last_timestamp_us_` | `volatile int64_t` | Timestamp da última transição detectada. |
| `last_state_` | `volatile uint8_t` | Último estado lido dos canais A e B. |

Essas variáveis são `volatile` porque podem ser alteradas fora do fluxo normal do programa, dentro de uma interrupção.

---

### 6.3 Dados usados no cálculo periódico

| Atributo | Tipo | Descrição |
|---|---|---|
| `previous_count_` | `int64_t` | Contagem anterior usada no cálculo de velocidade. |
| `previous_update_us_` | `int64_t` | Timestamp da última chamada de `update()`. |
| `velocity_m_s_` | `float` | Velocidade filtrada em m/s. |
| `acceleration_m_s2_` | `float` | Aceleração filtrada em m/s². |
| `raw_velocity_m_s_` | `float` | Velocidade bruta em m/s. |
| `raw_acceleration_m_s2_` | `float` | Aceleração bruta em m/s². |
| `previous_velocity_m_s_` | `float` | Velocidade anterior usada no cálculo da aceleração. |
| `has_previous_update_` | `bool` | Indica se já existe uma atualização anterior válida. |

---

### 6.4 Proteção de concorrência

```cpp
portMUX_TYPE mux_;
```

Usado para proteger regiões críticas acessadas tanto por interrupções quanto por tasks.

No ESP32, interrupções e tasks podem acessar as mesmas variáveis ao mesmo tempo. O uso de `portMUX_TYPE` evita condições de corrida e leituras inconsistentes.

---

## 7. Métodos privados auxiliares

### 7.1 `readState()`

```cpp
uint8_t readState() const;
```

Lê os canais A e B e retorna um estado codificado em bits.

Um formato comum é:

- bit 0: canal A;
- bit 1: canal B.

Esse estado é usado para determinar a transição do encoder e o sentido de rotação.

---

### 7.2 `ticksToDistanceM()`

```cpp
float ticksToDistanceM(int64_t ticks) const;
```

Converte uma quantidade de ticks em distância linear, em metros.

A implementação precisa considerar a resolução do encoder e a geometria do sistema, como diâmetro ou perímetro da roda.

---

### 7.3 `calculateVelocityMps()`

```cpp
float calculateVelocityMps(int64_t delta_ticks, float dt_s) const;
```

Calcula a velocidade linear a partir da variação de ticks e do intervalo de tempo.

Fórmula geral:

```text
velocidade = distância / tempo
```

---

### 7.4 `calculateAccelerationMps2()`

```cpp
float calculateAccelerationMps2(
    float current_velocity_m_s,
    float previous_velocity_m_s,
    float dt_s
) const;
```

Calcula a aceleração linear a partir da variação de velocidade no tempo.

Fórmula geral:

```text
aceleração = (velocidade_atual - velocidade_anterior) / tempo
```

---

### 7.5 `applyLowPassFilter()`

```cpp
float applyLowPassFilter(
    float raw_value,
    float filtered_value,
    float alpha
) const;
```

Aplica um filtro passa-baixa simples sobre um valor bruto.

Fórmula típica:

```text
valor_filtrado = alpha * valor_bruto + (1 - alpha) * valor_filtrado_anterior
```

Quanto maior o `alpha`, mais rápido o filtro responde ao valor novo.

Quanto menor o `alpha`, mais suave fica a resposta, porém com maior atraso.

---

## 8. Fluxo recomendado de uso

### Inicialização

```cpp
EncoderReader encoder_left(18, 19, 1024.0f, "encoder_left");

void setup()
{
    encoder_left.begin();
}
```

### Interrupção

```cpp
void IRAM_ATTR encoderLeftISR()
{
    encoder_left.handleInterrupt();
}
```

### Atualização periódica

```cpp
void taskSensors(void *parameter)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true)
    {
        encoder_left.update();

        float velocity = encoder_left.getVelocityMps();
        float acceleration = encoder_left.getAccelerationMps2();

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
}
```

---

## 9. Boas práticas

### Na interrupção

Evite:

- `Serial.print()`;
- cálculos pesados;
- alocação dinâmica;
- delays;
- chamadas bloqueantes.

A interrupção deve apenas atualizar os dados mínimos necessários.

### Na task

A task pode executar os cálculos mais pesados, como:

- velocidade;
- aceleração;
- filtros;
- publicação de dados;
- debug via serial.

### Em leituras compartilhadas

Sempre que a leitura envolver variáveis modificadas por interrupção, use proteção com região crítica.

---

## 10. Papel da classe no firmware

Dentro do firmware do ESP32, a classe `EncoderReader` funciona como uma camada de abstração para os encoders.

Ela evita que o restante do código precise lidar diretamente com:

- leitura dos pinos;
- decodificação em quadratura;
- contagem de ticks;
- proteção contra concorrência;
- cálculo de velocidade;
- cálculo de aceleração;
- filtragem dos sinais.

Dessa forma, outras partes do firmware podem simplesmente consultar valores já processados, como:

```cpp
float v = encoder.getVelocityMps();
float a = encoder.getAccelerationMps2();
long ticks = encoder.getCount();
```

Isso deixa o código principal mais limpo, modular e fácil de manter.
