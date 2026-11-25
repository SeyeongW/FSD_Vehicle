# 단일 및 다중 LiDAR 기반 조류 탐지·추적 시스템 성능 비교 및 다중 센서 융합 효과 분석

**Performance Comparison of Single and Multi-LiDAR-Based Bird Detection and Tracking Systems and Analysis of Multi-Sensor Fusion Effects**

공학사 졸업연구 · 2026년 11월 · 이병하, 서영욱 · 지도교수 고기성
청주대학교 국방·항공학부 항공기계공학과

> 표기 규칙
> - 📷 **[그림 삽입]**: 들어갈 그림, 📊 **[표 삽입]**: 들어갈 표
> - 🛠 표시는 새로 만들거나 고쳐야 하는 그림
> - [2]에서 가져온 그림·표는 캡션에 "[2]에서 인용"을 표시함

---

## 1. 서 론

항공기 조류 충돌은 큰 인명 사고로 이어질 수 있는 안전 위험 요인이다. 국회 국토교통위원회 국정감사 자료에 따르면 최근 약 5년간 국내 민간 공항에서 발생한 조류 충돌은 총 623건이며, 연도별 현황은 Fig. 1과 같다. 기존의 인력 투입 퇴치 방식은 탐지 범위와 대응 속도에 한계가 있어, 활주로 주변 조류를 자동으로 탐지·추적하는 체계가 필요하다.

> 📷 **[그림 삽입] Fig. 1. Annual Bird Strike Cases at Domestic Airports**
> 기존 초안 Fig. 1을 그대로 사용 (캡션 오타 "Brid" → "Bird" 수정)

LiDAR는 반사점의 거리와 방향으로 3차원 Point Cloud를 생성하여 물체의 공간 위치를 정밀하게 측정할 수 있다[1]. 선행 연구에서는 3D LiDAR와 카메라를 연동한 이동형 플랫폼을 구성하고, 가상 공항 환경에서 조류 탐지·추적 가능성을 확인하였다[2].

그러나 단일 LiDAR 시스템은 세 가지 한계를 가진다. 첫째, 탐지 파이프라인이 하나의 센서에 의존하므로 센서 고장이나 통신 중단 시 탐지 정보가 완전히 상실된다. 둘째, 물체에 맺히는 포인트 수는 거리의 제곱에 반비례하여 감소하므로, 조류처럼 작은 표적은 거리가 멀어지면 탐지에 필요한 포인트를 확보하지 못한다. 셋째, 센서의 수직 시야각 한계로 센서 상부에 탐지 불가 영역이 존재한다.

다중 LiDAR를 서로 다른 위치에 배치하면 이러한 한계를 보완할 수 있다. 다만 선행 연구[2]의 융합 방식은 모든 센서의 데이터가 시간 동기화되어야 출력되는 구조로, 센서 하나만 중단되어도 융합 전체가 정지하는 문제가 있었다.

본 연구에서는 일부 센서가 중단되어도 나머지 센서로 융합을 지속하는 고장 허용 융합 구조를 적용하고, 가상 환경에서 단일 및 다중 LiDAR 시스템을 센서 고장, 거리에 따른 포인트 감소, 상부 탐지 불가 영역의 세 측면에서 정량 비교·분석한다.

---

## 2. 본 론

### 2.1 단일 LiDAR 시스템 구성 및 흐름

단일 LiDAR 기반 조류 탐지·추적 시스템은 Livox Mid-360 센서를 탑재한 이동형 플랫폼을 중심으로 구성하였다. 이동형 플랫폼은 주변 환경의 Point Cloud를 수집하여 조류의 위치를 탐지·추적하며, 벽면에 고정된 짐벌 카메라와 YOLO 기반 객체 인식으로 LiDAR가 탐지한 표적을 시각적으로 재확인한다. 전체 데이터 처리 과정은 Fig. 2와 같다.

> 📷 **[그림 삽입] Fig. 2. Point Cloud Detection and Tracking Pipeline for Single-LiDAR System**
> 기존 초안 Fig. 2 사용 (또는 [2]의 Fig. 3 "Detection-to-tracking processing pipeline" 인용)

먼저 높이 기준 필터로 벽 높이(2.0 m) 이하의 포인트를 제거하여 지면·벽 반사를 배제하고, 로봇 위치 중심의 동적 경계 상자로 플랫폼 자체의 반사점을 제거한다(Self-Point Removal).

전처리된 포인트에는 밀도 기반 군집 알고리즘인 DBSCAN(Density-Based Spatial Clustering of Applications with Noise)을 적용한다[3]. DBSCAN은 한 포인트의 반경 eps 이내에 min_samples개 이상의 포인트가 존재할 때 이를 군집의 핵심점으로 판단한다. 따라서 표적에 맺힌 포인트가 min_samples보다 적으면 군집 자체가 형성되지 않으며, 이 조건이 2.5절에서 분석하는 탐지 성능의 기준이 된다. DBSCAN은 군집의 형태·크기를 반영하지 못하므로, 생성된 군집의 크기와 중심 높이로 표적 후보를 판별하는 후처리 필터를 추가하였다. 사용한 파라미터는 Table 1과 같다.

> 📊 **[표 삽입] Table 1. DBSCAN Clustering and Target-Candidate Filter Parameters** ([2]에서 인용)
>
> | Category | Parameter | Value |
> |---|---|---|
> | DBSCAN | eps / min_samples | 0.6 m / 3 |
> | Target-candidate filter | Max. bounding box size (x, y, z) | 2.0 m / 2.0 m / 1.5 m |
> | Target-candidate filter | Min. centroid height | 2.0 m |
> | Track matching | Track-to-detection matching distance | 8.0 m |
> | Track retention | Locked-target missed-detection limit | 40 frames |
>
> ※ [2]의 Table 2 오타("min_samles", "Target-candidat e") 수정

판별된 표적 후보는 odom 좌표계로 변환한 뒤 등속 운동 모델 기반 칼만 필터로 추적한다[4]. 상태 벡터는 위치와 속도 x = [p_x, p_y, p_z, v_x, v_y, v_z]ᵀ로 정의하며, 상태 방정식과 측정 방정식은 식 (1), (2)와 같다.

> x_{k+1} = A·x_k + w_k,  A = [ I₃  Δt·I₃ ; 0  I₃ ]  (1)
>
> z_k = H·x_k + v_k,  H = [ I₃  0 ]  (2)

여기서 w_k와 v_k는 각각 공정 잡음과 측정 잡음이다. 일시적으로 측정이 없는 프레임에서는 예측 단계만 수행하여 추적을 유지한다. Fig. 3은 이 과정을 거쳐 추적된 조류를 RViz2에서 시각화한 결과이다.

> 📷 **[그림 삽입] Fig. 3. Tracked Bird Visualized with a Bounding Box in RViz2** ([2]의 Fig. 4 인용)

추정된 조류의 위치는 이동형 플랫폼의 로컬 좌표계로 변환되어 주행 제어에 사용되며, 추격 조향에는 Pure Pursuit[5]와 비례 제어를 적용하였다. 동시에 짐벌 카메라가 LiDAR의 추정 좌표를 조준하고 YOLO로 표적을 재확인한다(Fig. 4).

> 📷 **[그림 삽입] Fig. 4. YOLO-Based Bird Detection with Gimbal Camera** ([2]의 Fig. 6 인용)
> ※ 분량을 줄여야 하면 가장 먼저 뺄 수 있는 그림

### 2.2 단일 LiDAR 시스템의 한계 및 기능 위험 분석

단일 LiDAR 시스템은 하나의 센서가 획득한 정보만으로 주변 공간 전체를 탐지해야 하므로 다음과 같은 한계를 가진다.

첫째, 센서 고장이나 통신 중단이 발생하면 탐지 파이프라인의 입력 자체가 사라진다. 이때 표적 위치 정보가 상실되고, 추적 실패와 제어 기능 저하로 이어진다.

둘째, 물체에 맺히는 포인트 수 N은 물체의 단면적 A에 비례하고 거리 d의 제곱에 반비례한다(N ∝ A/d²). 조류는 단면적이 작아 거리가 멀어지면 포인트 수가 DBSCAN의 min_samples 아래로 떨어져 군집이 형성되지 않는다.

셋째, Mid-360의 수직 시야각은 약 −7°~52°로, 센서 상부에 원뿔형 탐지 불가 영역이 존재한다. 고도 h의 조류는 센서로부터 수평 거리 약 0.7h 이내에 들어오면 관측되지 않는다. 이 한계는 상부 시야각이 넓은 LiDAR로 완화할 수 있으나, 동일한 포인트 수를 더 넓은 시야에 분산하므로 각 밀도가 감소하여 둘째 한계가 심화되는 trade-off가 있다.

이러한 한계를 체계적으로 분석하기 위해 주요 기능별 기능 위험 분석을 수행하였으며, 결과는 Table 2와 같다.

> 📊 **[표 삽입] Table 2. Analysis of System Limitations and Improvements**
>
> | Main Function | Potential Failure Condition | System Effect | Design Requirement |
> |---|---|---|---|
> | 조류 탐지 (Bird Detection) | 탐지 실패 (Loss of Detection) | 표적 위치 정보 상실 (Loss of Target Position Information) | 복수 센서 관측 (Multi-Sensor Detection) |
> | 조류 추적 (Bird Tracking) | 추적 실패 (Loss of Tracking) | 추적 중단 및 제어 성능 저하 (Tracking Interruption and Control Performance Degradation) | 다중 관측 기반 추적 유지 (Tracking Continuity Through Multi-Sensor Observations) |
>
> ※ 기존 초안 Table 1에서 변경: ① 3번째 행(오탐지 → LiDAR·카메라 교차 검증)은 본 연구에서 평가하지 않아 삭제 ② 오타 "(Tracking of Detection)" → "(Loss of Tracking)", "Peformance" → "Performance"

이 분석을 바탕으로, 특정 센서가 표적을 관측하지 못하더라도 다른 관측 지점에서 표적 정보를 확보할 수 있도록 고정형 LiDAR와 이동형 LiDAR를 함께 배치하는 다중 LiDAR 시스템을 구성하였다.

### 2.3 다중 LiDAR 시스템 구성 및 흐름

다중 LiDAR 시스템은 활주로 양 끝에 서로 마주 보도록 배치한 고정형 LiDAR 2대와 이동형 플랫폼에 탑재한 LiDAR 1대, 그리고 벽면 고정 짐벌 카메라로 구성하였다. 각 센서의 탐지 영역은 Fig. 5와 같다.

> 📷 **[그림 삽입] Fig. 5. Sensor Detection Coverage of the Multi-LiDAR System**
> 기존 초안 Fig. 3 사용

서로 다른 위치에서 수집한 데이터를 통합하기 위해 odom 좌표계를 공통 기준으로 설정하였다. 조류의 탐지·추적에는 이동형 플랫폼 기준의 상대 위치만 필요하므로, 전역 좌표계는 사전에 정의된 웨이포인트를 따르는 순찰 경로계획에만 사용하였다. 두 고정형 LiDAR의 위치는 odom 프레임 기준 정적 변환으로 등록하였다.

세 LiDAR는 서로 다른 주기로 비동기 발행된다. 선행 연구[2]에서는 ApproximateTimeSynchronizer로 세 센서의 타임스탬프를 정합하였으나, 이 방식은 모든 센서의 메시지가 도착해야 출력이 생성되므로 센서 하나만 중단되어도 융합 출력 전체가 정지한다. 즉 센서를 늘렸음에도 고장 지점이 오히려 세 곳으로 늘어나는 구조이다.

이를 해결하기 위해 본 연구에서는 고장 허용 융합 구조를 적용하였다. 각 센서의 최신 Point Cloud를 개별 버퍼에 저장하고, 10 Hz 주기로 버퍼를 병합한다. 이때 가장 최근 데이터보다 0.3 s(약 2~3 스캔 주기) 이상 오래된 센서는 고장으로 판단하여 병합에서 제외한다. 따라서 일부 센서가 중단되어도 남은 센서만으로 융합이 지속되며, 센서가 복구되면 자동으로 다시 포함된다. 적용한 다중 LiDAR 시스템의 처리 과정은 Fig. 6과 같다.

> 📷 **[그림 삽입] Fig. 6. Point Cloud Detection and Tracking Pipeline for Multi-LiDAR System**
> 파일: `~/ros2_ws/ugv_ws/logs/thesis/results/fig_multi_lidar_pipeline.png` (기존 Fig. 4 스타일에 LiDAR 3개 입력과 고장 허용 융합 블록 추가)

### 2.4 시뮬레이션 환경 및 성능 평가

실험은 Gazebo에 구축한 20 m × 20 m 규모의 가상 활주로 환경에서 수행하였으며, 환경은 Fig. 7과 같다. 고정형 LiDAR 2대는 활주로 양 끝 (±9.33 m, 0 m, 높이 1.8 m)에 서로 마주 보도록 배치하였고, 이동형 LiDAR는 UGV 상부에 탑재하였다.

> 📷 **[그림 삽입] Fig. 7. Simulated Runway Environment in Gazebo** ([2]의 Fig. 5 인용)
> 🛠 가능하면 고정형 LiDAR 2대의 위치를 화살표나 원으로 표시

조류는 0.5 m 크기의 정육면체로 모델링하였으며, 고도 2.5~5.0 m 범위에서 임의의 목표점을 향해 비행한다. 실제 조류의 회피 기동을 모사하기 위해 최대 선회율을 120 °/s로 제한하였으며, 시뮬레이션 시간 기준 실측 평균 비행 속도는 약 1.5 m/s이다. 단일 및 다중 시스템이 동일한 비행 궤적에서 비교되도록 비행 경로의 난수 시드를 고정하였다. 단일 시스템은 이동형 LiDAR 1대만 융합 입력으로 사용하고, 그 외 탐지·추적·제어 과정은 다중 시스템과 동일하게 구성하였다. 주요 실험 조건은 Table 3과 같다.

> 📊 **[표 삽입] Table 3. Simulation and Experiment Conditions**
>
> | Item | Value |
> |---|---|
> | Map size | 20 m × 20 m (wall height 2.0 m) |
> | LiDAR | Livox Mid-360 (simulated) × 3 |
> | Points per scan / Update rate | 12,000 points / 10 Hz |
> | Vertical FOV (simulation) | −7.2° ~ 55.2° |
> | Fixed LiDAR position | (±9.33, 0, 1.8) m, facing each other |
> | Bird model | 0.5 m cube, altitude 2.5 ~ 5.0 m |
> | Bird flight | Random waypoints, max. turn rate 120 °/s, mean speed ≈ 1.5 m/s |
> | Trial duration | 120 s (fault injected at 60 s) |
> | Test cases | Single: Mid-360 fault / Multi: Mid-360, Fixed LiDAR 1, Fixed LiDAR 2 fault |
> | Repetitions | 3 flight paths (random seeds) per case, 12 trials in total |

센서 고장의 영향을 분석하기 위해, 조류가 등장한 후 60 s 시점에 특정 LiDAR의 메시지를 차단하여 센서 고장 및 통신 중단을 모사하였다. 각 시행에서 조류 등장 후 5~60 s 구간을 정상 구간, 60~120 s 구간을 고장 구간으로 구분하였다. 처음 5 s는 칼만 필터의 초기 수렴 구간이므로 제외하였다. 정상 구간은 고장 시행의 고장 전 데이터를 사용하므로, 다중 시스템의 정상 구간은 9회, 단일 시스템은 3회 시행으로 구성된다.

성능은 Table 4의 지표로 평가하였으며, 모든 지표는 조류의 실제 위치(Ground Truth)를 기준으로 10 Hz로 산출하였다. 각 지표는 시행별로 계산한 뒤 평균 ± 표준편차로 나타내었다.

> 📊 **[표 삽입] Table 4. Evaluation Metrics**
>
> | Metric | Definition |
> |---|---|
> | Detection Rate (%) | 융합 Point Cloud에서 조류 실제 위치 반경 0.5 m 이내 포인트가 3개(min_samples) 이상인 프레임의 비율 |
> | Tracking Continuity Rate (%) | 0.5 s 이내에 갱신된 추정 위치가 실제 위치로부터 1.0 m 이내인 프레임의 비율 |
> | Position Estimation Error (m) | 추정 위치와 실제 위치 간 3차원 거리의 평균 |
> | Points on Bird | 프레임당 조류 반경 0.5 m 이내 융합 포인트 수의 평균 |
> | Max. Tracking Interruption (s) | 추적 실패가 연속된 최장 시간 |
> | Processing Time (ms) | 융합 노드의 1회 처리 시간 평균 |

### 2.5 단일 및 다중 LiDAR 성능 비교 및 분석

**(1) 정상 상태 비교**

센서 고장이 없는 정상 구간에서의 성능 비교 결과는 Table 5와 같다.

> 📊 **[표 삽입] Table 5. Performance Comparison Between Single and Multi-LiDAR Systems (Normal Condition)**
>
> | Performance Metric | Single LiDAR | Multi LiDAR | Difference |
> |---|---|---|---|
> | Detection Rate (%) | 48.2 ± 4.2 | 97.4 ± 1.2 | +49.2 %p |
> | Tracking Continuity Rate (%) | 84.6 ± 1.0 | 99.6 ± 0.8 | +15.0 %p |
> | Position Estimation Error (m) | 0.81 ± 0.05 | 0.42 ± 0.03 | −0.39 m (−48 %) |
> | Points on Bird | 11.6 ± 2.6 | 24.5 ± 3.3 | ×2.1 |
> | Max. Tracking Interruption (s) | 2.47 ± 0.55 | 0.23 ± 0.46 | −2.24 s |
> | Average Processing Time (ms) | 15.2 ± 1.3 | 30.5 ± 2.5 | +15.3 ms (×2.0) |
>
> ※ 기존 초안 Table 2 양식을 유지하고 지표 2개(Points on Bird, Max. Tracking Interruption)를 추가

다중 시스템은 조류에 맺히는 포인트 수가 단일 시스템의 약 2.1배로 증가하였다. 그 결과 탐지 성공률은 48.2 %에서 97.4 %로 향상되었고, 위치 추정 오차는 약 48 % 감소하였다. 단일 시스템은 탐지 성공률이 48.2 %에 그쳤으나, 칼만 필터의 예측으로 추적 유지율은 84.6 %를 유지하였다. 다만 최대 2.5 s의 추적 중단이 발생하였다.

처리 시간은 다중 시스템이 약 2배 증가하였으나, 30.5 ms는 센서 스캔 주기(100 ms)의 약 1/3 수준으로 실시간 처리에 지장이 없었다. 다만 처리 지연은 추격 제어 루프의 위상 여유를 줄이는 요인이 되므로, 센서 수를 늘릴 때 고려해야 할 trade-off이다.

**(2) 센서 고장 시 비교**

고장 구간에서의 성능은 Table 6과 같으며, 시간에 따른 탐지 성공률의 변화는 Fig. 8과 같다.

> 📊 **[표 삽입] Table 6. Performance Under Sensor Fault (60 ~ 120 s)**
>
> | System / Faulted Sensor | Detection Rate (%) | Tracking Continuity Rate (%) | Max. Tracking Interruption (s) |
> |---|---|---|---|
> | Single / Mid-360 | 0.4 ± 0.3 | 0.8 ± 0.2 | 61.7 ± 1.5 (not recovered) |
> | Multi / Mid-360 (onboard) | 94.9 ± 0.7 | 99.1 ± 1.1 | 0.50 ± 0.62 |
> | Multi / Fixed LiDAR 1 | 80.1 ± 6.7 | 98.9 ± 0.9 | 0.30 ± 0.20 |
> | Multi / Fixed LiDAR 2 | 82.0 ± 3.9 | 99.2 ± 0.8 | 0.23 ± 0.23 |

> 📷 **[그림 삽입] Fig. 8. Detection Rate Before and After Sensor Fault**
> 파일: `~/ros2_ws/ugv_ws/logs/thesis/results/fig_fault_timeline.png`
> 🛠 지금 그림은 seed 3개를 하나로 합친 시계열이므로 그대로 써도 됨. 범례를 한글로 바꿀지 결정

단일 시스템은 고장 발생 약 2 s 후 탐지 성공률이 0 %로 떨어졌으며, 실험 종료까지 회복되지 않았다. 특히 탐지 파이프라인에 입력이 전혀 들어오지 않아 "표적 상실" 판정조차 발생하지 않았고, 이동형 플랫폼은 추적 상태를 유지한 채 정지하였다. 시스템이 스스로 고장을 인지하지 못하는 무음 고장(silent failure)으로, 단일 센서 의존 구조의 위험성을 보여준다.

반면 다중 시스템은 어느 센서가 고장 나더라도 추적 유지율 98.9 % 이상을 유지하였다. 고정형 LiDAR 고장 시에는 탐지 성공률이 80~82 %로 감소하여, 이동형 LiDAR 고장 시(94.9 %)보다 영향이 컸다. 이는 이동형 LiDAR가 추격 중 조류 하부에 위치하는 시간이 길어 상부 탐지 불가 영역에 자주 놓이므로, 실제 탐지에는 고정형 LiDAR의 기여가 더 크기 때문이다. 탐지 성공률이 감소한 구간에서도 칼만 필터의 예측과 남은 센서의 관측으로 추적 중단은 최대 0.5 s 이내에 그쳤다.

**(3) 거리에 따른 포인트 수 감소**

각 센서가 조류에 맺은 포인트 수를 센서와 조류 간 거리에 대해 나타내면 Fig. 9와 같다.

> 📷 **[그림 삽입] Fig. 9. Number of Points on Bird Versus Sensor-to-Bird Distance**
> 파일: `~/ros2_ws/ugv_ws/logs/thesis/results/fig_hits_vs_distance.png`
> 1 m 구간 평균(마커)과 N = k/d² 피팅(곡선). 온보드 Mid-360은 앙각 55° 이상 프레임 제외

측정된 포인트 수는 N = k/d² 관계를 따랐다. 최소제곱 피팅 결과, 포인트 수가 min_samples(3개) 아래로 떨어지는 한계 거리는 고정형 LiDAR가 약 11.8~12.6 m였다. 이동형 LiDAR는 상부 탐지 불가 영역의 프레임을 제외했을 때 약 11.0 m로, 고정형 LiDAR와 유사하였다. 즉 0.5 m 크기의 조류는 단일 Mid-360에서 약 12 m 이내에서만 안정적으로 탐지할 수 있으며, 탐지 범위를 넓히려면 센서의 관측 지점을 분산해 표적과의 거리를 줄이는 다중 배치가 효과적이다.

**(4) 상부 탐지 불가 영역**

이동형 LiDAR를 기준으로 조류의 앙각에 따른 탐지 결과는 Table 7과 같다.

> 📊 **[표 삽입] Table 7. Onboard LiDAR Detection by Bird Elevation Angle (Normal Condition)**
>
> | Elevation Angle | Frame Ratio (%) | Mean Distance (m) | Points on Bird | Detection Rate (%) |
> |---|---|---|---|---|
> | 0° ~ 40° | 33.9 | 6.4 | 10.6 | 60.2 |
> | 40° ~ 55° | 43.4 | 5.0 | 17.2 | 60.9 |
> | 55° ~ 90° | 22.7 | 4.3 | 0.2 | 1.0 |
>
> ※ 분량을 줄여야 하면 이 표를 빼고 본문 한 문장으로 대체 가능

앙각이 55°를 넘는 구간은 평균 거리가 4.3 m로 가장 가까웠음에도 탐지 성공률이 1.0 %에 그쳤다. 이 구간은 전체 정상 구간 프레임의 22.7 %를 차지하였다. 이는 추격 중인 이동형 플랫폼이 조류 하부로 접근하는 경우가 많기 때문이다. 이 한계는 센서 하드웨어의 시야각에서 비롯되며, 다중 시스템에서는 측면에서 관측하는 고정형 LiDAR가 이를 보완하였다.

---

## 3. 결 론

본 연구에서는 Gazebo 가상 활주로 환경에서 단일 및 다중 LiDAR 기반 조류 탐지·추적 시스템을 센서 고장, 거리에 따른 포인트 감소, 상부 탐지 불가 영역의 세 측면에서 비교·분석하였다. 주요 결과는 다음과 같다.

(1) 정상 상태에서 다중 시스템은 조류에 맺히는 포인트 수가 약 2.1배 증가하여, 탐지 성공률이 48.2 %에서 97.4 %로, 추적 유지율이 84.6 %에서 99.6 %로 향상되었고, 위치 추정 오차는 0.81 m에서 0.42 m로 감소하였다.

(2) 센서 고장 시 단일 시스템은 탐지 성공률이 0 %로 떨어지고 회복되지 않았으며, 시스템이 고장을 인지하지 못하는 무음 고장이 발생하였다. 다중 시스템은 고장 허용 융합 구조를 통해 어느 센서가 고장 나더라도 추적 유지율 98.9 % 이상을 유지하였다.

(3) 조류에 맺히는 포인트 수는 거리의 제곱에 반비례하였으며, 0.5 m 크기의 조류는 단일 Mid-360으로 약 12 m 이내에서만 안정적으로 탐지되었다. 또한 앙각 55° 이상의 상부 영역에서는 탐지 성공률이 1 %에 그쳤으며, 다중 시스템은 관측 지점을 분산하여 두 한계를 함께 보완하였다.

(4) 다중 시스템의 융합 처리 시간은 단일 시스템의 약 2배인 30.5 ms였으나, 센서 주기 이내로 실시간 처리가 가능하였다.

다만 본 연구는 시뮬레이션 환경에서 수행되었으며, 단일 조류와 정육면체 표적 모델을 사용하였다. 또한 맵 규모의 제약으로 센서와 조류 간 최대 거리가 약 20 m에 그쳐, 원거리 탐지 성능은 거리–포인트 수 관계를 통해 간접적으로 분석하였다. 장애물에 의한 가림, 오탐지, 카메라 교차 검증은 평가하지 않았다. 향후 실제 공항 규모의 환경과 복수 조류 시나리오에서의 검증, 실물 센서를 이용한 실험, 카메라 교차 검증을 통한 오탐지 저감을 수행할 예정이다.

---

## 참 고 문 헌

[1] Seoane, P., et al., "Assessment of LiDAR-based sensing technologies in bird–drone collision scenarios," Drones, Vol. 9, No. 1, p. 13, 2025.
　※ 초안에는 2024로 적혀 있음. Drones 9권 1호는 2025년 1월호로 보이니 원문에서 연도 확인 필요

[2] Seo, Y., Lee, B., Wang, S., Cho, T., and Ko, G., "Multi-LiDAR-Based Distributed Collaborative Response System for Bird Strike Prevention," Journal of the KNST, Vol. 9, No. 3, pp. 798-803, 2026. doi:10.31818/JKNST.2026.9.9.3.798

[3] Ester, M., Kriegel, H.-P., Sander, J., and Xu, X., "A Density-Based Algorithm for Discovering Clusters in Large Spatial Databases with Noise," Proceedings of the 2nd International Conference on Knowledge Discovery and Data Mining (KDD-96), pp. 226-231, 1996.

[4] Kalman, R. E., "A New Approach to Linear Filtering and Prediction Problems," Journal of Basic Engineering, Vol. 82, No. 1, pp. 35-45, 1960.

[5] Coulter, R. C., "Implementation of the Pure Pursuit Path Tracking Algorithm," Technical Report CMU-RI-TR-92-01, Carnegie Mellon University, 1992.

※ 추가 권장: 조류 충돌 통계(Fig. 1)의 출처인 국정감사 자료를 정확한 서지로 추가

---

## [작성 메모: 제출 전 삭제]

**그림·표 목록 (총 그림 9개, 표 7개)**

| 번호 | 내용 | 출처 | 상태 |
|---|---|---|---|
| Fig. 1 | 국내 공항 조류 충돌 현황 | 기존 초안 | 있음 |
| Fig. 2 | 단일 LiDAR 처리 파이프라인 | 기존 초안 | 있음 |
| Fig. 3 | RViz2 바운딩 박스 | [2] Fig. 4 | 있음 |
| Fig. 4 | YOLO 인식 화면 | [2] Fig. 6 | 있음 (생략 가능) |
| Fig. 5 | 다중 LiDAR 탐지 영역 | 기존 초안 Fig. 3 | 있음 |
| Fig. 6 | 다중 LiDAR 처리 파이프라인 | 기존 초안 Fig. 4 수정 | 완료 (`fig_multi_lidar_pipeline.png`) |
| Fig. 7 | Gazebo 활주로 환경 | [2] Fig. 5 | 있음 (🛠 LiDAR 위치 표시 권장) |
| Fig. 8 | 고장 전후 탐지 성공률 | 실험 결과 | 있음 |
| Fig. 9 | 거리별 포인트 수 + k/d² 피팅 | 실험 결과 | 완료 (`fig_hits_vs_distance.png`) |
| Table 1 | DBSCAN 파라미터 | [2] Table 2 | 오타 수정 |
| Table 2 | 기능 위험 분석 | 기존 초안 Table 1 | 3행 삭제, 오타 수정 |
| Table 3 | 실험 조건 | 신규 | 작성됨 |
| Table 4 | 평가 지표 정의 | 신규 | 작성됨 |
| Table 5 | 정상 상태 비교 | 기존 초안 Table 2 | 수치 채움 |
| Table 6 | 고장 시 비교 | 신규 | 작성됨 |
| Table 7 | 앙각별 탐지 | 신규 | 작성됨 (생략 가능) |

**수치 출처:** `~/ros2_ws/ugv_ws/logs/thesis/results/summary.md` (3 seed × 4 케이스 = 12회). 실측 조류 속도(평균 1.5 m/s)만 별도 계산 값이며, summary.md에는 없음.
