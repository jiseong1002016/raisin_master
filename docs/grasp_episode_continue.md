# Grasp episode 후속 준비 — 2026-10-24

이후의 유한한 NONREAL 종료·오류·재파지 검사와 수정은
[후속 검증 기록](grasp_episode_nonreal_closure.md)에 있다. 아래는 앞선 27-tick 실행의 기록이다.

C의 최종 EE 변경을 합친 railab 격리 설치에서 **실제 NONREAL node + local licensed
AnyGrasp SSH + A1/B1 에피소드**를 통과했다. 실제 로봇의 ready·관측 이동이나 grasp는
수행하지 않았다. 이 기록은 [이전 준비 문서](grasp_episode_preparation.md)의
2026-10-23 실행 결과와 [저장 RGBD SSH 검사](anygrasp_ssh_validation.json)를 잇는다.
정확한 source·hash·검증 범위는 [검증 JSON](grasp_episode_continue_validation.json)에 있다.

## 테이블과 자세

기본값은 grasp_iteration의 `configs/deployment_preparation.yaml`이 소유한다.

- 전체 상판: **X 0.70 m × Y 0.44 m**, 상면 **Z 0.78 m**.
  높이 주석은 `height of railab sliding table`이다.
- 제안한 source-world 중심: **`[0, 0.60] m`**. 상판 범위는
  X `[-0.35, 0.35]`, Y `[0.38, 0.82]` m다.
- 기본 물체의 기하 중심 XY: **`[0, 0.55] m`**. 캔 중심 Z는 약 `0.830901 m`다.
  mesh 원점 좌표와 구분한다. 이 XY는 도달·시야 검사용 배치 제안이며 실제
  robot base와 테이블의 정합을 측정한 값이 아니다.
- native bucket의 기존 벽 두께 0.012 m를 한 번 빼 내부를 `0.676 × 0.416 m`로
  만든다. native floor와 exported support의 전체 크기는 `0.70 × 0.44 m`다.
  학습 기본값은 바꾸지 않았다.

새 치수로 source reset/readback을 다시 검사하고 ready → front 관측 경로를
remote native library로 재검사했다. 7.55초, 10 ms 간격 **756 sample**, 최대 속도
0.299676 rad/s다. 이 검사는 설정된 scene에 대한 L arm/EE 검사다. self collision,
다른 팔, 보이지 않는 장애물, 연속 충돌 무발생이나 현재 실기 자세 → ready 경로를
검증하지 않는다. **두 자세 모두 실제 로봇에서 시험한 적이 없다.**

## 통합된 동작

1. C의 EE identification·bridge·maintenance·parameter 변경과 L-EE **30°** 시험
   delta를 보존했다. 다른 profile의 기존 0.2 rad 설정은 유지했다.
2. NONREAL은 rayrai, REAL은 D405의 aligned RGBD를 취득한 뒤 같은 robot self-filter,
   AnyGrasp, WORLD 변환, IK·경로 검사로 연결한다. railab은
   [SDK client 폴더](anygrasp_ssh.md)로 local의 기존 SDK를 호출한다.
3. 관측·AnyGrasp G0 생성·검증된 pregrasp 이동을 마치면 controller가 자세와
   heartbeat를 유지한다. **A1이 episode를 시작**하며 준비 전 A1은 버린다.
   A1 뒤 새 geometry·센서로 다시 검사하고 G0와 원래 추론 시각은 보존한다.
4. B1은 즉시 episode 종료를 요청한다. 결과를 기록하고 측정 자세 유지 + EE STOP으로
   끝낸다. 같은 controller load에서 자동 재시작하지 않는다.
5. REAL의 관측 자격 검사를 두 native 진입점에서 공유한다. launcher가 실제 쓰는
   `start.observation_qualification`도 검사하므로 NONREAL plan이 REAL에서 실행되는
   우회가 막혔다. 임시 node input contract는 복구하지 않았다.

동적 AnyGrasp 준비의 기본 실행 경로는 **single L+EE 7-DOF**다. launcher는 dual
profile도 제공하지만 dual은 별도 검증한 legacy 관측 왕복 plan이 필요하다.
L arm·EE·D405·tactile 모델은 고정된 최신 L/EE URDF revision을 사용한다.
74개 물체 선택은 제공하며 각 물체의 실기 경로·인식을 인증한 것은 아니다.

`grasp_iteration C`의 `x870-grasp_iteration-C` pane에서 완료 표시와 idle prompt를
확인했다. C는 정식 EE 30° 빌드·설치를 마쳤으며 실제 30° 구동은 아직 하지 않았다고
보고했다. 확인 capture는 `evidence/C_session_prepublish.{txt,json}`에 보관한다.

## 실제로 통과한 검증

- 통합 workspace build/install, CTest **13/13**, maintenance hold **20 case**,
  joint/ready trajectory 및 설치 parameter load 검사.
- 실제 smoke의 `LD_LIBRARY_PATH` 조건에서 **83 ELF**의 의존성 누락과 외부
  framework 경로가 없음을 확인했다. 환경변수 없는 개별 DSO 배포는 검증하지 않았다.
- 새 bundle **97 file** seal. bundle source는 `c4b8df8e`이며 이후 project 변경은
  문서뿐이다. 봉인된 bundle은 갱신하지 않았다.
- railab `node_ssh_005`: `single_L_ee_nonreal` 실제 node, ready/front 도착 안정화,
  **A1 전 2.029초 대기 → A1 1회 → 실제 제어 27 tick → B1 1회 →
  `operator_B1` 결과 → 0.5초 정지 유지**. 전체 44.19초.
- 현재 rayrai 관측의 self-filter 후 **8,057점**을 production SSH transport로
  local SDK에 보냈다. 왕복 **0.310초**, 사용 시 결과 age **0.401초**,
  선택 후보 rank 0, pregrasp 경로 **777 sample / 7.76초**를 통과했다.
- A1 뒤 geometry 관측 age는 **0.0549초**다. 준비 이동 전 최초 G0의 age
  18.16초와 별도 기록한다. SDK를 다시 호출해 최초 G0를 바꾸지 않았다.

이는 lifecycle·transport 통합 검사다. B1으로 일찍 종료했으므로 lift/hold 성공이나
실패 후 재제안 성능을 검증하지 않았다(`pose_model_calls: 0`,
`independent_grasp_success: null`). 선택한 Controller/Failure는 bootstrap 미인증
모델이며 registry의 Failure shadow-only·trigger 금지, Actor/Selector offline/research
메타데이터는 유지했다. 다만 episode의 `grasp_pose_generator.enabled=true`가
재제안 실행을 명시적으로 허용한다. 27 tick에서 미호출이었다는 결과로 실행이
차단됐다고 판단할 수 없다. relaxed timing으로 수행했으므로 실기 deadline 보장이 아니다.

## 설치·근거·재현

local 작업 root는 `/home/jiseong/raisim_ws/isolation/episode_continue_20261024`,
railab 작업 root는 `/home/raipal/workspace/jiseong/isolation/episode_continue_20261024`다.
railab의 **새 `master/install`**과 `inference_bundle_002`를 사용했다. 정식 Git 소스의
승격은 운영 `/home/raipal/raisin_master/install` 교체를 뜻하지 않는다.

build는 `nice -n 10`, `-j2`로 수행했다. 호환되지 않는 구형 clang-tidy는 끈 상태다.
task prefix에 필요한 SOEM/SVT-AV1/implot DSO를 이전 격리 설치에서 복사했고,
offscreen을 제공하지 않는 시스템 SDL 대신 공식 SDL 2.30.12를 task vendor 경로에서
빌드해 task prefix에만 넣었다. source revision·DSO hash·의존성 감사는 검증 JSON에
남겼다. 운영 설치·시스템 라이브러리는 수정하지 않았다.
이 보존 범위는 이번 에피소드 통합 작업이며 C의 별도 정식 EE 설치는 유지했다.

두 기기의 작업 root 아래 `evidence/`에 다음을 보관한다.

- `node_ssh_005/smoke_result.json`, `episodes/*/{inputs_and_results.jsonl,result.json}`.
- `source_start.local.json` 및 hash를 확인해 경로만 옮긴 `source_start.table.json`.
  원래 source native binary의 local 경로는 provenance로 보존한다.
- `front_observation_start.table.json`, `verification/device_free/summary.json`.
- `harness/launch_argv_005.json`과 harness source, build/install·SDL·의존성·cleanup 기록.

재검사는 **railab에서 새 run directory**를 지정한다. 아래는 node·SDK를 시작하지
않고 경로와 bundle seal만 검사하는 preflight다. 실제 격리 NONREAL 재실행은 CPU/RAM
여유를 확인한 뒤 `subprocess.run` 앞에 `argv.append('--execute')`를 추가한다.

```bash
python3 - <<'PY'
import json, subprocess
from pathlib import Path
base = Path('/home/raipal/workspace/jiseong/isolation/episode_continue_20261024')
argv = json.loads((base / 'evidence/harness/launch_argv_005.json').read_text())
if '--execute' in argv:
    raise RuntimeError('preflight argv must not contain --execute')
run = base / 'evidence/node_ssh_recheck_001'
if run.exists():
    raise FileExistsError('choose a new run directory: ' + str(run))
argv[argv.index('--run-dir') + 1] = str(run)
subprocess.run(argv, check=True)
PY
```

harness는 private net/PID/IPC/shm와 읽기 전용 root를 사용하고 표준 pseudo device 및
렌더용 NVIDIA device만 노출한다. 로봇·USB·D405·EtherCAT·입력 장치는 노출하지 않는다.
SSH는 고정 local 목적지로 가는 Unix relay와 기존 제한 키/host pin을 사용한다.
`node_ssh_005/episode.json`에는 이미 정리한 임시 job/relay 경로가 있으므로 그 파일을
직접 재실행하지 않는다. harness 또는 production launcher로 새 설정을 생성한다.

`--execute` 실행에서는 harness가 A1·B1을 각각 한 번 자동 게시한다. 사람이 joy를
누를 필요가 없다. 내부 driver 제한은 200초, 바깥 namespace 제한은 220초다.
성공·실패 후 소유한 process group과 namespace를 종료하고 job/relay를 정리한다.
기존 run은 덮어쓰지 않으므로 재검사마다 `node_ssh_recheck_001` 이름을 바꾼다.

## 실기에서 남은 것

실제 base/world 정합, 테이블·camera 시야·주변 장애물, 현재 자세 → ready 및 관측
경로, EE homing·tactile pad/baseline과 독립 성공 판정을 확인해야 한다. 최근 camera
설정은 mount/URDF 합성과 일치하지만 별도 물리 실측을 수행하지 않았다.
실기 첫 실행은 사용자와 장치 담당자가 이 항목들의 실측·점검 기록 및 별도 REAL
경로 자격을 검토하고 공용 node/device 사용 해제를 확인한 뒤 진행한다.

REAL geometry는 선택한 mesh/prior 주변의 6 DoF 관측성을 요구한다. 기본 캔은
yaw가 모호해 현재 REAL 준비에서 거부된다. qualification flag 변경으로 통과시키지
않으며, 물체마다 관측성·경로를 검증해야 한다.

내려놓기 → 손가락 열기 → ready 복귀 → 물체 재배치 → 다음 episode는
[기존 후속 계획](grasp_episode_preparation.md#에피소드-이후-동작의-구현-위치)을 따른다.
현재 terminal hold 이후 자동 수행은 추가하지 않았다.
