# 순찰(PATROL) ↔ 추격(TRACKING) 제어 로직 정리

이 문서는 `hackathon_bringup.launch.py` 파이프라인에서 로봇이 순찰할 때와 새를
추격할 때 실제로 어떤 제어 로직을 쓰는지, 왜 그렇게 만들었는지를 정리한다.
관련 파일: `patrol_node.py`, `bird_pursuit_node.py`, `bird_detector_node.py`,
`lidar_fusion_node.py`.

---

## 1. 전체 아키텍처

```
lidar_fusion_node → /fused_pointcloud2 (odom 프레임)
        ↓
bird_detector_node (DBSCAN + 칼만필터) → /bird_detected, /bird_target(+velocity)
        │                                        │
        ├→ patrol_node (PATROL↔TRACKING 상태기계) └→ bird_pursuit_node (PN/Pure Pursuit)
        │        └→ /tracking_active ──────────────────────┘
        └→ AMCL(map_server만, Nav2 순찰 스택은 안 씀) — patrol_node의 위치 센서 역할
```

**상태**
- `PATROL`: `patrol_node`가 직접 `/cmd_vel` 발행, 웨이포인트를 순서대로 순찰
- `TRACKING`: `patrol_node`는 정지, `bird_pursuit_node`가 `/cmd_vel` 소유

`/tracking_active` 플래그로 두 노드가 `/cmd_vel`을 절대 동시에 쓰지 않도록
상호 배타적으로 동작한다.

### 왜 Nav2(TEB/코스트맵/BT)를 순찰에서 뺐는가

맵이 정적이고 웨이포인트를 미리 다 아는 상황이라 Nav2의 핵심 강점(동적 장애물
실시간 회피)이 필요 없었다. 오히려 TEB의 `RotationShimController`,
`velocity_smoother`, 코스트맵 등 여러 레이어가 서로 값을 다시 clamp하면서
튜닝이 계속 꼬였다(속도를 올리면 다른 레이어가 도로 낮추는 식). 그래서 `patrol_node`
자체가 `bird_pursuit_node`와 동일한 패턴(방위각 직접 계산 + 거리비례 전진 +
LaserScan 정지/감속 게이트)으로 직접 조향하도록 재작성했다. **AMCL(map_server+amcl)만
남기고 나머지 Nav2 스택은 launch에서 제거**했다 — `patrol_node`가 웨이포인트까지
방위각/거리를 계산하려면 map 프레임 위치가 필요하기 때문에 위치 센서 용도로만 쓴다.

---

## 2. PATROL 모드 — `patrol_node.py`

### 2.1 웨이포인트

지그재그(lawnmower) 패턴, `(x, y)`만 있고 목표 yaw는 없음:

```python
x_tracks = [-6.67, -4.0, -1.33, 1.33, 4.0, 6.67]
y_low, y_high = -6.67, 6.67
```
X 트랙을 따라가며 각 트랙은 Y축으로 왕복. 원래는 목표 yaw까지 지정했었는데,
Nav2/TEB가 최종 yaw를 맞추려고 후진을 택하는 문제가 있어서 (x, y)만 남기고
뺐다 — 다음 웨이포인트로 향하는 방위각이 자연스럽게 헤딩을 정해준다.

### 2.2 코너를 바깥으로 도는 조준점 (`_turn_aim_point`)

원래 웨이포인트를 그대로 찍으면 코너에서 각지게 꺾여서 지그재그처럼 보였다.
그래서 실제 조준점은 **웨이포인트 자체가 아니라, 진입방향→진출방향 회전의
바깥쪽으로 살짝 밀어낸 점**이다:

1. 이전→현재(`in_dir`), 현재→다음(`out_dir`) 단위벡터 계산
2. 두 방향의 평균(`avg_dir`)에 수직인 두 방향 중, 외적 부호(`cross_z`)로 회전방향(좌/우)을 판단해 **바깥쪽**을 고름
3. 턴이 급할수록(거의 U턴, `turn_sharpness→1`) 많이 밀어내고, 완만하면 거의 안 밀어냄:
   ```python
   turn_sharpness = (1 - dot(in_dir, out_dir)) / 2   # 0=직진, 1=U턴
   offset = corner_offset * turn_sharpness            # corner_offset = 1.5m
   ```
4. `aim = curr_wp + perp_outward * offset`

이 조준점을 향해 가면 코너를 바깥에서 감아 돌아 다음 구간으로 자연스럽게
이어진다. 도착 판정(`waypoint_tolerance = 1.5m`)도 이 조준점 기준이라, 코너에
못 미쳐서부터 이미 다음 웨이포인트를 보기 시작한다(Pure Pursuit의
look-ahead와 동일한 효과).

### 2.3 조향 — PD 제어

```python
bearing = atan2(local_y, local_x)   # 로봇 기준 조준점 방위각

desired_angular = angular_gain * bearing - angular_damping * robot_angular_vel
# angular_gain = 1.5, angular_damping = 0.8 (실측 각속도는 /odom에서)
desired_angular = clamp(desired_angular, ±max_angular_speed)  # max_angular_speed = 1.2
```

**P만 쓰면 왜 안 되는가**: 방위각 오차만 보고 조향하면(P only), 게인이 조금만
높아도 목표를 넘어서고 반대로 꺾이는 지속 진동(limit cycle)이 생긴다 — 실제로
직선 구간에서 헤딩이 -43°~-165°처럼 100도 넘게 계속 좌우로 흔들리는 게
확인됐었다. `/odom`의 실측 각속도를 감쇠항(D)으로 빼서, 이미 빠르게 돌고 있을수록
제동이 걸려 오버슈트를 막는다.

**I(적분)항은 안 씀**: 목표(방위각/거리) 자체가 로봇이 움직이며 계속 바뀌고
웨이포인트도 계속 넘어가서, "고정된 오차를 오래 누적해 없앤다"는 I항의 취지와
안 맞는다. 오히려 최대속도에 막혀 오차를 못 줄이는 동안 I항이 쌓였다가 나중에
와인드업(overshoot)을 일으키는 부작용만 크다.

### 2.4 전진 — 이중 스케일링

```python
align_scale = cos(bearing / 2)                                    # ①
rotation_calm_scale = max(0, 1 - |robot_angular_vel| / max_angular_speed)  # ②
desired_linear = min(linear_gain * dist, max_linear_speed) * align_scale * rotation_calm_scale
# linear_gain = 0.5, max_linear_speed = 1.0
```

- **① `cos(bearing/2)`**: 정렬 여부를 "정지/전진" 이분법으로 게이트하면, 코너에서
  방위각이 크게 바뀌어야 할 때 "완전정지 → 최고속도 제자리회전 → 급출발"로
  뚝뚝 끊겨서 물결치듯 보였다. `cos(bearing)`(90도에서 0)보다 완만하게 떨어지는
  `cos(bearing/2)`(180도에서만 0)로 바꿔서, 많이 틀어져도 어느 정도는 전진을
  유지한 채 넓게 커브를 그리도록 했다.
- **② `rotation_calm_scale`**: ①은 "목표 방위각"만 보는데, 제자리 회전 직후
  관성으로 실제로는 아직 빠르게 돌고 있는 상태에서 그 값만 믿고 바로 전진을
  넣으면 실제 헤딩이 목표를 지나쳐 엉뚱한 방향으로 직진했다. 실측 각속도가
  클수록 전진을 추가로 눌러서, 회전이 실제로 잦아든 뒤에야 직진하게 한다.

### 2.5 장애물 회피 + 스무딩

`bird_pursuit_node`와 동일한 LaserScan 전방(`obstacle_half_angle=0.40rad`) 최근접
거리 기반 정지(`<0.5m`)/감속(`<1.2m`) 게이트. 이후 EMA 스무딩
(`smooth_alpha=0.8` — AMCL 위치가 깨끗해서 가볍게만) 적용 후 `/cmd_vel` 발행.

---

## 3. TRACKING 모드 — `bird_pursuit_node.py`

`steering_mode` 파라미터로 두 조향법칙 중 선택 (현재 기본값 **`pn`**):

### 3.1 PN (비례항법, Proportional Navigation)

```python
los_rate = (rx*vy - ry*vx) / range2      # 시선각 변화율 (R×V 외적 / 거리²)
desired_angular = pn_gain * los_rate + pn_bearing_bias_gain * bearing
# pn_gain = 3.5, pn_bearing_bias_gain = 0.6
```

**왜 bias 항을 섞었는가**: 순수 PN은 LOS_rate가 우연히 0이면(새가 로봇 기준
거의 일직선으로 멀어지거나, 로봇이 정지해서 상대속도가 새 속도뿐인 경우)
조향값도 0이 되어 스스로 돌 이유를 못 찾는다. 방위각이 거의 반대(180도)라
아래 전진 스케일까지 0으로 눌리는 상황과 겹치면 **완전 정지 상태(교착)**에
빠진다 — 실제로 "로봇이 아무것도 안 하고 제자리에만 있는" 증상으로 나타났다.
방위각 자체에 비례하는 최소 성분(bias)을 살짝 섞어 이 교착을 깼다.

### 3.2 Pure Pursuit

```python
desired_angular = pp_angular_gain * bearing   # pp_angular_gain = 2.0
```
표적의 현재 방위각을 직접 겨냥하는 단순 P제어. PN과 달리 속도 추정치가 전혀
필요 없어서, 탐지가 느리고 끊기는 환경에서 더 안정적으로 동작한다(칼만필터
속도 추정 수렴을 기다릴 필요가 없음). 해커톤 정량평가에서 PN vs Pure Pursuit
비교가 필요해서 둘 다 남겨뒀고, `steering_mode` launch arg로 바로 전환 가능.

### 3.3 전진 — 방위각 기반 스케일링

```python
align_scale = max(0, cos(bearing / 2))
if dist_2d > chase_target_dist:  # chase_target_dist = 2.0 (최소 접근거리)
    desired_linear = min(linear_gain * (dist_2d - chase_target_dist), max_linear_speed) * align_scale
    # linear_gain = 0.5, max_linear_speed = 1.8
else:
    desired_linear = 0.0
```

**이분법 게이트를 없앤 이유**: 예전엔 "방위각이 `chase_align_threshold` 이내일
때만 전진"하는 이분법 게이트를 PN에도 적용했더니, PN은 원래 방위각을 0으로
몰아가지 않고(요격 코스 유지가 목적) 일정한 값으로 유지하는 게 정상이라
전진 자체가 영구히 막혀버렸다. 그래서 PN에 한해 게이트를 아예 없앴는데, 이번엔
로봇이 지금 향한 방향이 새와 무관해도(심하면 정반대여도) 거리 조건만 맞으면
전속력 돌진하는 사고가 났다(추적 시작 순간, LOS_rate가 잠깐 작을 때). 최종적으로
`cos(bearing/2)` 완만한 스케일링으로 통일 — PN이 막히는 문제도 없고(180도가
아닌 이상 어느 정도는 전진), 정반대 방향 돌진 사고도 없다(180도 근처에서 0에
수렴).

`max_linear_speed=1.8`인 이유: 새 기본 비행속도(1.2m/s)보다 로봇이 느리면
추적이 완벽해도 물리적으로 못 따라잡는다(기존 0.5m/s는 새보다 느렸음).

### 3.4 표적 신선도 체크

```python
target_fresh = (target_pos is not None) and (now - target_stamp) < target_timeout  # 0.8s
```
`/bird_target`이 0.8초 이상 안 들어오면 즉시 정지. (x, y)만 쓰고 새의 고도(z)는
조향/속도 계산 어디에도 안 씀 — 로봇은 지상차량이라 항상 새의 지상 투영점만 본다.

### 3.5 장애물 회피 + 스무딩

`patrol_node`와 동일한 LaserScan 게이트 + EMA 스무딩(`smooth_alpha=0.4` — 탐지
자체가 노이즈 있어서 patrol보다 무겁게).

---

## 4. 상태 전환 (`patrol_node._bird_cb`)

- **PATROL → TRACKING**: `/bird_detected=True` 즉시 전환. `patrol_node`가 자기
  cmd_vel을 0으로 한 번 발행하고 정지, `/tracking_active=True` 발행.
- **TRACKING → PATROL**: `/bird_detected=False`가 **연속 20프레임(`_BIRD_LOST_THRESHOLD`,
  약 2초)** 이상 지속돼야 복귀 (hysteresis) — 새가 잠깐 안 보였다가 바로 다시
  보이는 상황에서 매번 왔다갔다 전환되는 걸 방지.

---

## 5. 검출/추적 계층 (`bird_detector_node.py`) 요약

TRACKING 제어가 받는 `/bird_target`, `/bird_target_velocity`의 품질을 좌우하는
상류 로직. 자세한 필터링 순서는 별도 문서(`bird_detection_filtering_pipeline.md`) 참고.

- **DBSCAN**: `eps=0.6m`, `min_samples=3`, 높이필터(`z>2.1m`)+자기포인트 제거 후 클러스터링
- **트랙 매칭 거리**: `track_match_dist=8.0m` (원래 3.0m — 라이더 실측 갱신 주기가
  느려서 그 사이 새가 3m 넘게 움직이는 경우가 흔해 3.0m로는 매 갱신마다 트랙
  ID가 리셋됐음)
- **잠금 타깃 재연결**: 이번 라운드에 잠금 타깃이 `track_match_dist`를 넘겨
  매칭 실패해도, 씬에 새가 한 마리뿐이라는 전제로 남은 미매칭 후보 중 가장
  가까운 걸 **새 ID 발급 없이 같은 트랙으로 이어붙임** — 칼만필터 속도 추정치가
  안 끊기게 함
- **stale 프레임 스킵**: `/fused_pointcloud2`의 `header.stamp`가 직전과 같으면
  (=원본 라이더가 아직 새 데이터를 안 냈으면) DBSCAN/`kf_update`를 건너뛰고
  예측(coast)만 함 — 반복된 stale 관측이 칼만필터 속도 추정치를 자꾸 0으로
  끌어내리는 걸 방지
- **회전 중 동결**: `rotation_freeze_threshold=0.20rad/s` 넘으면 새 타깃 잠금 보류
  (odom 노이즈로 오탐 위험)

---

## 6. 현재 기본 파라미터 값 (launch arg로 노출된 것)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `steering_mode` | `pn` | `pn` 또는 `pure_pursuit` |
| `pn_gain` | 3.5 | PN 게인 |
| `pn_bearing_bias_gain` | 0.6 | PN에 섞는 방위각 비례 최소 성분 |
| `pp_angular_gain` | 2.0 | Pure Pursuit 게인 |
| `flight_speed` | 1.2 | 새 비행 속도(m/s) |

`patrol_node`, `bird_pursuit_node`의 나머지 게인/임계값들은 코드 내
클래스 속성(launch arg로 노출 안 됨) — 위 문서의 각 절에 표기된 값이 현재값.
