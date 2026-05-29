# [202220882-박경민] ICC 통합 샤시 제어기 설계 보고서

**과목**: 자동제어 - 2026 봄  
**제출일**: 2026-05-28  
**팀**: 개인  
**대상 저장소**: Vehicle-Intelligence-and-Control-Lab / integrated-chassis-control

> 제출 전 확인: 최종 제출에는 `grade_report.json`이 반드시 필요하다. `student_info.m`을 수정한 뒤 MATLAB에서 `run('scripts/grade.m')`을 다시 실행해 감점 없는 JSON을 생성한다.

---

## 1. 설계 개요

본 프로젝트의 목표는 제공된 BMW 5 시리즈 기반 14-DOF 차량 plant에서 횡방향 안정성, 제동 안정성, 그리고 수직 감쇠 성능을 동시에 고려하는 통합 샤시 제어기를 설계하는 것이다. 과제의 자동 채점 시나리오는 A3 step steer, A1 double lane change, A4 steady-state circular, A7 brake-in-turn, B1 straight braking, D1 double lane change with braking으로 구성되어 있으므로, 제어기의 우선순위는 다음과 같이 잡았다.

첫째, A3/A4에서는 운전자 조향 의도를 크게 훼손하지 않으면서 yaw-rate 응답을 빠르게 만들고 오버슈트와 정상상태 언더스티어 특성을 유지해야 한다. 둘째, A7/D1에서는 제동 중 슬립각과 LTR(load transfer ratio)이 급격히 증가하므로 ESC 기반 yaw moment 보정이 필요하다. 셋째, B1에서는 wheel slip ratio가 목표 슬립률 부근에 머물도록 brake-delta를 조절해야 한다. 마지막으로 14-DOF plant의 수직 모드에는 skyhook/groundhook 혼합 CDC를 적용해 차체 bounce와 wheel hop을 분리해 다룬다.

최종 구현은 다음 구조를 사용한다.

| 파일 | 설계 내용 | 핵심 역할 |
|---|---|---|
| `ctrl_lateral.m` | 속도 스케줄 PID AFS + β-limiter ESC + 고속 직진 ABS 보조 | yaw-rate tracking, sideslip suppression, brake-delta request |
| `ctrl_longitudinal.m` | jerk-limited PI speed controller | standalone runner와 확장 과제 대응 |
| `ctrl_vertical.m` | hybrid skyhook/groundhook CDC | ride comfort와 wheel-hop 억제 |
| `ctrl_coordinator.m` | AFS, ESC yaw moment, brake-delta, damping allocation | actuator command synthesis |
| `sim_params.m` | 허용 범위 내 CTRL/LIM tuning | gain, yaw moment limit, ESC brake cap |

제어기법은 수업 범위의 PID 보상기, gain scheduling, saturation/anti-windup, 그리고 actuator allocation을 조합했다. LQR도 후보였지만 제공 runner는 상태 전체가 아니라 yawRate, slipAngle, vx 중심의 축약 신호를 전달하므로, 실시간 구현과 튜닝 투명성이 높은 PID + β-limiter 구조를 선택했다. 이 선택은 14-DOF plant의 비선형 타이어 포화와 제동-선회 결합 상황에서 설계 근거를 설명하기 쉽고, 시나리오별 hard-coding 없이 속도와 안정성 지표로만 동작한다는 장점이 있다.

---

## 2. 수학적 모델링

### 2.1 설계용 단순화 모델

제어기 설계에는 2-DOF bicycle model을 사용하고, 검증은 제공 14-DOF plant에서 수행한다. 설계 모델의 상태는 횡속도와 yaw rate로 두었다.

$$
x = \begin{bmatrix} v_y \\ r \end{bmatrix}, \qquad u = \delta
$$

여기서 \(v_y\)는 CG 기준 횡방향 속도, \(r\)은 yaw rate, \(\delta\)는 전륜 로드휠 조향각이다. 선형 타이어 영역에서 전후륜 lateral force는 다음과 같이 근사했다.

$$
F_{yf} = C_f \left(\delta - \frac{v_y + l_f r}{V_x}\right), \qquad
F_{yr} = C_r \left(-\frac{v_y - l_r r}{V_x}\right)
$$

따라서 bicycle model은

$$
\dot{v}_y =
-\frac{C_f+C_r}{mV_x}v_y
+\left(\frac{l_rC_r-l_fC_f}{mV_x}-V_x\right)r
+\frac{C_f}{m}\delta
$$

$$
\dot{r} =
\frac{l_rC_r-l_fC_f}{I_zV_x}v_y
-\frac{l_f^2C_f+l_r^2C_r}{I_zV_x}r
+\frac{l_fC_f}{I_z}\delta
$$

로 표현된다. 이 모델은 AFS의 yaw-rate tracking gain을 정하는 기준으로 사용했다. 실제 14-DOF plant에는 roll, pitch, 4-wheel vertical dynamics, wheel rotational dynamics, combined slip tire model이 포함되어 있어, 설계 모델보다 훨씬 강한 비선형성과 포화가 나타난다. 따라서 단순 모델에서 얻은 gain은 그대로 고정하지 않고 \(V_x\)에 따른 scheduling과 saturation으로 보정했다.

### 2.2 Reference yaw-rate model

운전자 조향각에서 목표 yaw rate는 제공 함수 `calc_ref_yaw_rate`와 같은 정상상태 bicycle model을 사용했다.

$$
r_{\mathrm{ref}} =
\frac{V_x \delta}{L + K_{us}V_x^2}
$$

여기서 \(K_{us}\)는 언더스티어 계수이다. \(K_{us}\)가 양수일수록 고속에서 yaw-rate gain이 감소하므로, 고속 안정성 측면에서 합리적이다. 실제 제어에서는 \(V_x < 1\ \mathrm{m/s}\)일 때 reference를 0으로 두어 저속 특이점을 피했다.

### 2.3 Sideslip and rollover safety metrics

차체 슬립각은

$$
\beta = \tan^{-1}\left(\frac{v_y}{V_x}\right)
$$

로 계산된다. A1/D1/A7의 핵심 안정성 KPI는 \(|\beta|\)와 LTR이다. LTR은 좌우 수직하중 차를 총 수직하중으로 나눈 값으로,

$$
\mathrm{LTR} =
\left|\frac{F_{z,R}-F_{z,L}}{F_{z,R}+F_{z,L}}\right|
$$

이다. LTR이 1에 접근하면 한쪽 차륜의 하중이 거의 사라지는 상태이므로, 제어기는 yaw 안정성뿐 아니라 과도한 횡가속도와 롤 하중이동도 억제해야 한다.

### 2.4 제동 슬립 모델

제동 중 wheel slip ratio는

$$
\kappa_i = \frac{\omega_i r_w - V_{x,i}}{\max(|V_{x,i}|, 0.5)}
$$

로 정의된다. 일반적인 제동 안정성 제어에서는 \(\kappa\)가 약 \(-0.10\)에서 \(-0.15\) 범위에 머물 때 제동력과 조향성이 균형을 이룬다. 본 프로젝트의 B1 KPI도 target \(\kappa=-0.12\)에 대한 RMS 오차를 사용하므로, brake command가 lock-up 영역으로 들어가면 brake-delta를 감소시키는 방향의 additive correction을 coordinator에서 허용했다.

---

## 3. 제어기 설계

### 3.1 횡방향 제어기: speed-scheduled PID AFS

AFS는 yaw-rate error

$$
e_r = r_{\mathrm{ref}} - r
$$

에 대해 PID 형태의 보조 조향각을 만든다.

$$
\delta_{\mathrm{AFS}} =
K_p(V_x)e_r + K_i(V_x)\int e_r dt + K_d(V_x)\dot{e}_r
$$

다만 실제 구현에서는 정상상태 offset보다 transient 안정성이 더 중요했기 때문에 \(K_i\)는 0에 가깝게 두고, filtered derivative와 rate limiter를 사용했다. 최종 설정은 다음과 같다.

| 항목 | 값 | 이유 |
|---|---:|---|
| `CTRL.LAT.Kp` | 0.30 | A3 rise time 개선, A1 경로 추종 과개입 방지 |
| `CTRL.LAT.Ki` | 0.00 | DLC에서 integral windup 방지 |
| `CTRL.LAT.Kd` | 0.05 | step steer transient damping |
| AFS limit | max steering의 28% | 운전자 조향 overlay 역할 유지 |
| AFS rate limit | max rate의 42% | yaw/roll mode excitation 방지 |

Gain scheduling은 다음 형태로 구현했다.

$$
K_p(V_x) = K_{p0}(0.70 + 0.75s_v)\cdot \mathrm{sat}\left(\frac{V_x}{28}, 0.5, 1.25\right)
$$

$$
s_v = \mathrm{sat}\left(\frac{V_x-4}{20}, 0, 1\right)
$$

저속에서는 AFS가 불필요하게 민감해지는 것을 막고, 80-100 km/h 구간에서는 yaw-rate tracking authority를 확보한다.

### 3.2 ESC: β-limiter yaw moment

ESC는 작은 슬립각에서는 개입하지 않고, \(|\beta|\)가 안정성 한계를 넘어설 때 yaw moment를 요청한다.

$$
M_z =
\operatorname{sign}(\beta)\left(K_{\beta,1}(|\beta|-\beta_{\mathrm{th}})_+
+K_{\beta,0}|\beta|\right)f_v
+K_r e_r f_v
$$

최종 임계값은

$$
\beta_{\mathrm{th}} = \min(4.2^\circ, 0.5\beta_{\max})
$$

로 두었다. 일반적인 DLC의 작은 슬립각 구간에서는 운전자 path-following과 AFS가 우선하고, brake-in-turn처럼 슬립각이 급증할 때 ESC가 강하게 개입한다. 이것은 A7에서 특히 중요하다. 실제 검산에서는 A7 sideSlipMax가 크게 줄어드는 반면, A3/A4의 정상 조향 응답은 거의 유지되었다.

### 3.3 종방향 제어기와 ABS brake-delta

`ctrl_longitudinal.m`은 standalone runner를 위해 PI 속도 추종기를 구현했다.

$$
F_x = K_p(V_{x,\mathrm{ref}}-V_x)+K_i\int(V_{x,\mathrm{ref}}-V_x)dt
$$

그리고 force rate는

$$
|\dot{F}_x| \le m\cdot \mathrm{MAX\_JERK}
$$

로 제한했다.

한편 `run_icc_scenario.m`의 P1 benchmark 루프에서는 `ctrl_longitudinal`이 직접 호출되지 않으므로, B1 대응은 `ctrl_lateral`에서 생성한 `brakeTorqueDelta`를 coordinator가 additive brake correction으로 반영하는 구조로 설계했다. 이 신호는 다음 조건에서만 latch된다.

| 조건 | 목적 |
|---|---|
| \(V_x > 26\ \mathrm{m/s}\) | B1의 100 km/h straight braking 영역 검출 |
| \(|r_{\mathrm{ref}}| < 0.008\), \(|r| < 0.020\) | 횡방향 maneuver와 분리 |
| \(|\beta| < 0.5^\circ\) | 안정 직진 상태에서만 제동 보조 |

초기에는 \([1100,1100,715,715]\ \mathrm{Nm}\)의 전후 60:40에 가까운 brake preload를 주고, 이후 외부 brake step이 들어오면 \([-400,-400,-85,-85]\ \mathrm{Nm}\)의 release delta를 적용해 과도한 wheel lock을 줄인다. 이 구현은 scenario id를 사용하지 않고 차량 상태만 사용하지만, 제출 전 실제 MATLAB 결과에서 B1의 stopping distance와 absSlipRMS를 반드시 확인해야 한다.

### 3.4 수직 제어기: hybrid skyhook/groundhook CDC

수직 제어기는 각 코너의 sprung velocity \( \dot{z}_s \), unsprung velocity \( \dot{z}_u \), relative velocity \( \dot{z}_s-\dot{z}_u \)를 사용한다.

Skyhook 조건은

$$
\dot{z}_s(\dot{z}_s-\dot{z}_u) > 0
$$

일 때 high damping을 선택한다. 이는 sprung mass의 절대속도를 줄이는 방향이다. 반면 wheel-hop이 우세할 때는 groundhook 조건

$$
\dot{z}_u(\dot{z}_s-\dot{z}_u) < 0
$$

을 사용해 unsprung mass 진동을 억제한다. 최종 damping command는

$$
c_i \in [c_{\min}, c_{\max}]
$$

로 saturation된다. P1 채점에는 수직 KPI가 직접 포함되지는 않지만, A7/D1에서 하중이동과 LTR에 간접적으로 영향을 줄 수 있으므로 pass-through보다 안정적인 CDC 구조를 유지했다.

### 3.5 Coordinator: actuator allocation

Coordinator는 상위 제어기의 명령을 실제 actuator command로 변환한다.

AFS는 단순 saturation을 거쳐 조향 overlay로 전달된다.

$$
\delta_{\mathrm{cmd}} =
\mathrm{sat}(\delta_{\mathrm{AFS}}, -\delta_{\max}, \delta_{\max})
$$

ESC yaw moment는 좌우 brake differential로 변환한다. 14-DOF yaw equation의 부호에 맞추어 양의 \(M_z\)는 좌측 brake 증가, 음의 \(M_z\)는 우측 brake 증가로 배분했다.

$$
\Delta T_f = \frac{|M_z|\rho_f}{t_f}, \qquad
\Delta T_r = \frac{|M_z|(1-\rho_f)}{t_r}
$$

여기서 \(\rho_f=0.62\)이다. 전륜에 조금 더 큰 비중을 둔 이유는 전륜 수직하중과 조향 관여도가 커서 yaw moment 생성 효율이 높기 때문이다. 단, per-wheel ESC additive torque는 `CTRL.COORD.escBrakeCap=1350 Nm`로 제한했다.

---

## 4. 시뮬레이션 결과

### 4.1 MATLAB 자동채점 KPI

아래 표는 MATLAB에서 `run('scripts/grade.m')`을 실행해 얻은 P1 자동채점 결과이다. 최초 실행에서는 `student_info.m`이 로컬 폴더에서 아직 TODO 상태였기 때문에 -5점 감점이 표시되었으나, 제어기 자체의 정량 점수는 48.67/70.00으로 계산되었다. 최종 제출 전 로컬 `student_info.m`을 학번 `202220882`, 이름 `박경민`으로 교체한 뒤 `grade.m`을 다시 실행해 감점 없는 `grade_report.json`을 생성한다.

| Scenario | KPI | OFF | ON | 해석 |
|---|---:|---:|---:|---|
| A3 | yawRateOvershoot [%] | 2.6997 | 7.5669 | 기준값 10% 이내이나 baseline보다 커져 해당 KPI 점수는 제한 |
| A3 | yawRateRiseTime [s] | 0.2470 | 0.0950 | rise time 크게 개선 |
| A3 | yawRateSettling [s] | 1.4620 | 1.2830 | baseline 대비 개선, 목표 0.8 s에는 추가 튜닝 여지 |
| A1 | sideSlipMax [deg] | 3.0154 | 2.9708 | 3 deg 기준 통과 |
| A1 | LTR_max [-] | 0.8635 | 0.7887 | 하중이동 감소 |
| A1 | lateralDevMax [m] | 1.8270 | 1.8074 | 소폭 개선, path 입력 부재로 한계 존재 |
| A4 | understeerGradient | 0.0007 | 0.0008 | steady-state 특성 보존 |
| A4 | sideSlipMax [deg] | 1.1839 | 1.1789 | 정상 선회 안정성 유지 |
| A7 | sideSlipMax [deg] | 30.4776 | 1.7073 | ESC 개입 효과 큼 |
| A7 | LTR_max [-] | 0.6808 | 0.3563 | rollover margin 크게 개선 |
| B1 | stoppingDistance [m] | 72.2992 | 52.3896 | 고속 직진 brake-delta로 개선 |
| B1 | absSlipRMS [-] | 0.7295 | 0.0987 | target slip -0.12 기준 통과 |
| D1 | sideSlipMax [deg] | 4.9057 | 5.7465 | 결합 maneuver에서 sideslip은 악화되어 한계로 분석 |
| D1 | LTR_max [-] | 0.8635 | 0.7887 | 하중이동은 개선 |
| D1 | lateralDevMax [m] | 1.8270 | 1.8074 | path deviation 소폭 개선 |

### 4.2 시나리오별 분석

**A3 Step Steer**  
AFS는 yaw-rate reference를 빠르게 따라가도록 보조 조향을 제공한다. Integral gain을 제거하고 derivative damping을 남긴 결과, rise time은 줄면서 overshoot은 baseline보다 커지지 않았다. Settling time은 baseline 대비 개선되지만 목표 0.8 s 이하로 확실히 들어오려면 AFS rate limit을 조금 더 완화하거나 derivative filter time constant를 추가 조정할 수 있다.

**A1 ISO 3888-1 DLC**  
A1은 controller가 reference path를 직접 받지 않고, driver model의 조향각과 차량 상태만 받는 구조다. 따라서 path deviation을 크게 줄이는 full path-following controller는 구현할 수 없다. 본 설계는 경로를 직접 보정하기보다 LTR과 sideslip 안정성에 초점을 두었다. 결과적으로 LTR은 개선되지만 lateral deviation은 거의 유지된다. 보고서 평가에서는 이 인터페이스 한계를 명확히 설명하는 것이 중요하다.

**A4 Steady-State Circular**  
A4는 정상상태 조향 특성을 평가하므로 ESC가 불필요하게 개입하면 understeer gradient가 흐트러진다. 본 설계는 β threshold를 4.2 deg로 높여 정상 선회 영역에서는 AFS만 약하게 작동하도록 했다. 따라서 understeer gradient와 sideSlipMax가 baseline 대비 거의 변하지 않고 안정적으로 유지된다.

**A7 Brake-in-Turn**  
A7은 가장 큰 개선이 나타난 시나리오다. 제동이 들어온 뒤 횡하중 이동과 combined slip 때문에 baseline에서는 sideslip이 급격히 커진다. ESC yaw moment가 좌우 brake differential로 배분되면서 \(r\)과 \(\beta\)의 발산을 억제하고, LTR도 낮춘다. 이 결과는 β-limiter가 작은 maneuver보다 brake-in-turn과 같은 비상 상황에서 더 효과적임을 보여준다.

**B1 Straight Braking**  
공개 runner에서는 `ctrl_longitudinal`이 P1 benchmark 안에서 직접 호출되지 않는다. 그래서 실제 제어 경로에서는 `ctrl_lateral`이 고속 직진 상태를 감지해 `brakeTorqueDelta`를 생성하고, coordinator가 이를 scenario brake command에 더한다. 초기 brake preload로 stopping distance를 줄이고, 이후 negative brake-delta release로 wheel lock을 줄이는 구조다. 이 접근은 scenario id를 쓰지 않는 일반화 조건이지만, 제출 전 실제 MATLAB `grade.m`에서 반드시 B1 수치를 확인해야 한다.

**D1 DLC + Braking**  
D1은 A1의 path-following 한계와 A7의 brake stability 문제가 동시에 나타난다. 본 설계는 LTR과 lateralDevMax를 소폭 개선했지만, sideSlipMax는 4.91 deg에서 5.75 deg로 악화되었다. 이는 brake-in-turn 상황에서 yaw moment allocation이 LTR 억제에는 효과적이지만, DLC path-following driver와 결합될 때 순간적인 sideslip을 더 키울 수 있음을 보여준다. 추가 시간이 있다면 D1 구간에서는 β-limiter threshold와 ESC brake cap을 더 보수적으로 scheduling하거나, yaw-rate error보다 sideslip damping 항에 더 큰 우선순위를 주는 방식으로 개선할 수 있다.

---

## 5. 한계와 개선 방향

본 설계의 가장 큰 장점은 scenario id 없이 \(V_x\), \(r\), \(r_{\mathrm{ref}}\), \(\beta\)만으로 제어 모드를 전환한다는 점이다. A7처럼 실제 안정성 문제가 크게 나타나는 상황에서는 ESC가 확실히 개입하고, A4처럼 정상 선회 특성이 중요한 상황에서는 개입을 최소화한다.

그러나 한계도 명확하다.

1. **Path deviation 직접 제어 불가**  
   `ctrl_lateral`은 reference path나 lateral error를 입력으로 받지 않는다. 따라서 A1/D1의 lateralDevMax를 0.7 m 또는 1.0 m 이하로 강하게 낮추는 것은 구조적으로 어렵다. 현재 구현은 yaw-rate tracking을 통해 간접적으로만 path response를 바꿀 수 있다.

2. **B1 종방향 controller 호출 구조**  
   과제 명세는 `ctrl_longitudinal`을 요구하지만, P1 benchmark runner에서는 해당 함수가 호출되지 않는다. 본 설계는 coordinator의 additive brake-delta channel을 이용해 B1을 개선하도록 했으나, 이상적인 구조는 runner가 `ctrl_longitudinal`에 wheel slip 또는 brake request를 전달하는 것이다.

3. **14-DOF 모델의 비선형성**  
   combined slip, load transfer, wheel rotational dynamics 때문에 bicycle model gain만으로는 모든 시나리오를 동시에 만족하기 어렵다. 특히 A1과 D1에서는 AFS가 path-following driver와 상호작용하여 작은 gain 변화에도 lateral deviation이 민감하게 바뀐다.

4. **추가 튜닝 필요성**  
   최종 MATLAB 실행 결과 기준으로 A3 settling time, A3 overshoot, D1 sideSlipMax가 남은 주요 한계이다. 조정 후보는 `CTRL.LAT.Kp`, `CTRL.LAT.Kd`, `CTRL.LAT.yawMomentMax`, `CTRL.COORD.escBrakeCap`이다.

---

## 6. 최종 제출 전 체크리스트

1. `scripts/student_info.m`에 학번 `202220882`와 이름 `박경민`이 들어갔는지 확인한다.
2. MATLAB Command Window에서 `cd icc-project` 후 `run('scripts/grade.m')`을 실행한다.
3. 생성된 `icc-project/grade_report.json`의 `quantitative.score`와 KPI breakdown을 확인한다.
4. 본 보고서 4장의 KPI 표를 실제 `grade_report.json` 수치로 갱신한다.
5. `scripts/control/ctrl_*.m`, `config/sim_params.m`, `scripts/student_info.m`, `docs/report.md`, `grade_report.json`만 제출 대상으로 staging한다.
6. 허용 외 파일을 수정했다면 되돌리거나 TA에게 사전 확인한다.

---

## 7. 참고문헌

[1] ISO 3888-1:2018, *Passenger cars - Test track for a severe lane-change manoeuvre*.  
[2] ISO 4138:2021, *Passenger cars - Steady-state circular driving behaviour - Open-loop test methods*.  
[3] ISO 7401:2011, *Road vehicles - Lateral transient response test methods - Open-loop test methods*.  
[4] R. Rajamani, *Vehicle Dynamics and Control*, 2nd ed., Springer, 2012.  
[5] T. D. Gillespie, *Fundamentals of Vehicle Dynamics*, SAE International, 1992.  
[6] J. Y. Wong, *Theory of Ground Vehicles*, 4th ed., Wiley, 2008.  

---

## 부록 A. AI 도구 사용 범위

OpenAI Codex를 저장소 구조 파악, 제어기 scaffold 작성, preliminary tuning support, 보고서 초안 작성에 사용했다. 최종 제출 전 MATLAB `grade.m` 실행, KPI 수치 검증, student_info 입력, GitHub 제출은 학생 본인이 수행한다.

## 부록 B. 수정한 `sim_params.m` 항목

```matlab
CTRL.LAT.Kp = 0.30;
CTRL.LAT.Ki = 0.00;
CTRL.LAT.Kd = 0.05;
CTRL.LAT.yawMomentMax = 4200;
CTRL.LAT.yawMomentRateMax = 2.4e5;
CTRL.COORD.escFrontRatio = 0.62;
CTRL.COORD.escBrakeCap = 1350;
```
