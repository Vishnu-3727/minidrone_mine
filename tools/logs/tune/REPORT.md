# Auto-tune report

Generated 25-Nov-2026 06:11:25

Best training score: 967.8 (points 62)

## Final parameters
- `A_UP = 0.6`
- `A_BRAKE = 1.5`
- `LAG_T = 0.3`
- `END_MARGIN = 0.1`
- `LOOK_R = 0.187`
- `YAW_ACC = 2.0944`

## Every evaluation

| round | parameter | value | score | kept |
|---|---|---|---|---|
| 0 | base | NaN | 872.8 | yes |
| 1 | A_UP | 0.36 | 872.1 |  |
| 1 | A_UP | 0.9 | 873.4 |  |
| 1 | A_BRAKE | 1.05 | 872.8 |  |
| 1 | A_BRAKE | 1.95 | 872.7 |  |
| 1 | LAG_T | 0.21 | 872.7 |  |
| 1 | LAG_T | 0.39 | 872.7 |  |
| 1 | END_MARGIN | 0.07 | 872.9 |  |
| 1 | END_MARGIN | 0.14 | 872.4 |  |
| 1 | LOOK_R | 0.187 | 967.8 | yes |
| 1 | LOOK_R | 0.2094 | 872.6 |  |
| 1 | YAW_ACC | 1.257 | 967.0 |  |
| 1 | YAW_ACC | 3.142 | 968.6 |  |

## Full validation (logs/final)
100 / 103 flights pass (all sections + landed within 10 cm)
landing error: mean 3.25 cm, max 175.63 cm
max path error: mean 9.9 cm, worst 22.3 cm
touchdown time: mean 37.8 s

Failed flights (story logs in logs/final/<track>.txt):
- cross_s1_3: sections 0/5, landed 0, SEARCH used (line lost); did not land before the time limit
- cross_s1_4: sections 4/6, landed 0, SEARCH used (line lost); did not land before the time limit; path error 22 cm
- cross_s1_6: sections 3/6, landed 1, path error 20 cm
