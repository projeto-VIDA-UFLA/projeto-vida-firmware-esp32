# Banco de baterias de tracao, medido pelo INA228 (I2C) a cada 100 ms.
uint8 STATE_INVALID=0
uint8 STATE_NORMAL=1
uint8 STATE_EMERGENCY=2         # SoC <= 20%; so informativo, nao bloqueia o veiculo

builtin_interfaces/Time timestamp  # instante da leitura
float32 voltage_v               # tensao do banco (VBUS), sem filtro
float32 current_a               # corrente no shunt (> 0 = descarga); NaN sem shunt configurado
float32 soc_percent             # pela tensao filtrada, linear entre 35,8 V (0%) e 40 V (100%); so vale em repouso
uint8 state
bool valid                      # false: INA228 sem resposta ou leitura fora da faixa
