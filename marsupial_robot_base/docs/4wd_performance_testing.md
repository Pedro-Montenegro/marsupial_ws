# Ensaios de desempenho 4WD

O setup registra comando, odometria das rodas, posições individuais FL/RL/FR/RR,
odometria externa opcional, IMU, estado dos eixos e eventos do perfil.

## Segurança

1. Use uma área livre e plana no primeiro ensaio.
2. Mantenha um operador próximo ao corte de alimentação/E-stop.
3. Confirme os quatro pneus, cabos USB e alimentação.
4. Execute inicialmente com as velocidades conservadoras do perfil padrão.
5. O movimento automático só é habilitado com `armed:=true`.

## Ensaio automático

Inicie previamente os sensores que serão usados como referência. Depois:

```bash
cd ~/marsupial_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch marsupial_robot_base performance_test_4wd.launch.py \
  armed:=true \
  run_name:=terra_compactada_01
```

O perfil faz reta lenta, reta média, arcos e pivôs, sempre intercalados com
paradas. O launch termina automaticamente e coloca os eixos em `IDLE`.

Para usar outro tópico de odometria independente:

```bash
ros2 launch marsupial_robot_base performance_test_4wd.launch.py \
  armed:=true \
  run_name:=grama_01 \
  reference_odom_topic:=/minha_referencia/odom
```

Não use `/odometry/filtered` como ground truth se ele incorporar `/wheel/odom`.
Isso esconderia justamente o slip que o ensaio pretende medir.

## Ensaio manual/teleop

Terminal 1:

```bash
ros2 launch marsupial_robot_base bringup_base_4wd.launch.py
```

Terminal 2:

```bash
ros2 launch marsupial_robot_base performance_record_4wd.launch.py \
  run_name:=teleop_terra_01
```

Terminal 3:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p speed:=0.20 -p turn:=0.30
```

Encerre primeiro o logger com `Ctrl+C` para garantir o flush dos CSVs.

## Arquivos gerados

Cada ensaio cria:

```text
~/marsupial_test_results/YYYYMMDD_HHMMSS_nome/
├── metadata.json
├── samples.csv
└── events.csv
```

Após análise também são criados:

```text
├── summary.json
├── report.md
├── timeseries.png
└── trajectory.png
```

## Análise

```bash
ros2 run marsupial_robot_base performance_analyze \
  ~/marsupial_test_results/YYYYMMDD_HHMMSS_nome
```

Se não houver odometria externa, meça com trena o deslocamento final e, quando
aplicável, o giro real:

```bash
ros2 run marsupial_robot_base performance_analyze \
  ~/marsupial_test_results/YYYYMMDD_HHMMSS_nome \
  --actual-distance-m 5.00 \
  --actual-yaw-deg 360
```

## Métricas

- latência de 20% do degrau de comando até cada roda;
- RMSE de velocidade linear e angular;
- divergência FL–RL e FR–RR;
- período, P95, maior lacuna e falhas >100 ms da telemetria USB;
- caminho previsto pelas rodas e caminho de referência;
- erro de posição e yaw finais;
- slip longitudinal e angular quando há referência independente;
- comparação wheel yaw-rate × IMU;
- erro contra distância e yaw medidos manualmente.

Slip absoluto exige referência independente: visual odometry, SLAM que não use
wheel odom, mocap, RTK ou medição física. A IMU sozinha não mede deslocamento
linear confiável durante um ensaio longo.
