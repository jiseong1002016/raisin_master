# NONREAL grasp episode 검증 — 2026-10-24

관측 → AnyGrasp → pregrasp → A1 시작 → 실행 → 종료의 NONREAL 소프트웨어 경로를
검사하고, JSON 입력 처리와 재파지 접촉 판정에서 발견한 두 버그를 수정했다.
REAL 로봇 노드와 구동 장치는 사용하지 않았다. 실행 결과와 검증 범위는
[검증 JSON](grasp_episode_nonreal_closure_validation.json)에 기록한다.

자연 실행은 **1,124 active tick 후 `ik_failure_limit`로 종료**했다. B0 close
870 tick, lift 254 tick을 관측했으며 hold 완료와 자연 발생 실패 경보·재제안은
관측하지 못했다. 이것은 종료 경로의 검증이며 grasp 성공 판정은 아니다.
학습 모델·threshold·안전 오차 한계는 변경하지 않았다.

## 수정

- `raipal_grasp/runtime.py`: JSON object가 아닌 array/null/string을 명시적으로
  거부한다. 수정 전에는 세 입력이 `AttributeError`로 정상 오류 처리를 벗어났다.
  실제 Unix socket의 13개 잘못된 입력·단절 회귀 검사와 기존 A1/B1 초기화를 통과했다.
- `GraspBV0Controller.hpp`: 재파지 open/retreat/reentry의 단계 전환에 **유효하고
  신선한 tactile 무접촉 상태**를 요구한다. 기존 pad 접촉 개수는 torque 의존
  모드를 껐을 때 0에 머물렀다. 기존 센서 입력을 사용하며 해당 모드를 켜지 않는다.
  변경 전 실패, 변경 후 단독 C++ 검사 및 실제 B0 DSO 재파지 검사를 보존했다.
- synthetic smoke의 초기화 순서·센서 시각·terminal hold 기대값을 실제 startup에
  맞췄다. 테스트를 위해 production 우회나 임시 input contract를 추가하지 않았다.

최종 source는 controller `605fcca`, grasp_iteration `b0bbf3e`이며 두 저장소의
`feat/nonreal-episode-closure-20261024`에 있다. 재검사 argv는 이 작업 root의
`controller/raisin_raipal_grasp_controller/python`과 별도 빌드 B0를 사용한다.

접촉 gate는 단계 전환을 막는다. 진행 중인 retreat/new-pregrasp 위치 명령을
별도로 멈추는 기능이나 손가락의 실측 완전 개방 판정은 추가하지 않았다.
기존 B0의 commanded-open 조건과 위치·회전 도착 조건은 유지한다.

## 실제 NONREAL 노드

railab의 격리 namespace에서 다음을 실행했다. 모든 종료 후 A1/B1을 다시 보내
2초 동안 같은 load가 재시작하지 않는 것을 확인했다.
`t01`–`t06`은 수정한 Python runtime과 기존 B0를, `t07`은 수정한 Python과
새 B0를 사용했다. 따라서 `t02`는 최종 B0의 자연 실행 성능 검증으로 세지 않는다.

- `t01`: 준비 전 A1 무시, READY 2초 유지, 새 A1, 3초 wall budget 자율 종료.
- `t02`: 모델을 바꾸지 않은 최대 20초 자연 실행. 11.66초에 IK 연속 실패로 종료.
  마지막 오차는 위치 0.002612 m, 회전 0.041205 rad로 회전 한계 0.04 rad를 넘었다.
  Failure score 최대 0.075832, alarm 0, proposal 0, reentry 0이다.
- `t03`: READY에서 B1, active tick 0.
- `t04b`: backend와 실제 SSH 준비 연결이 존재하는 READY 이전에 B1, active tick 0.
- `t05`: active command 이후 소유한 backend만 종료. native가 `backend disconnected`로
  종료하고 재시작하지 않았다. 강제 종료한 Python의 결과 파일은 요구하지 않았다.
- `t06`: 첫 A1/B1 종료 후 명시적으로 새 controller를 load하여 다른 episode ID와
  backend로 다시 READY/A1/B1을 통과했다. 자동 같은-load 재무장은 구현하지 않는다.
- `t07`: 수정한 B0 DSO를 읽기 전용으로 덮어 마운트하여 `t01` 경로를 재검사했다.
  backend의 실제 loaded file inode/device/SHA를 확인했다. 기존 prefix 1,375개
  파일·링크와 bundle 98개 파일·링크는 전후 동일했다.

`n01b`–`n08`은 test-only backend가 실제 native pre-start 입력에 잘못된 응답을
보냈다. 잘못된 JSON, episode ID, tick, NaN, EE 범위, dynamic pregrasp 범위,
응답 지연, 중복 응답 8개 모두 해당 원인으로 거부됐다. active tick과 물체 release는
0이었다. 이 검사는 pre-start 통신·준비 범위이며 active PD target gate 전체의
실행 증명은 아니다. 최초 `n01`의 socket 경로 불일치와 `t04`의 자식 PID 탐색
실패는 하네스 오류로 구분해 원본을 보존했다.

## 모델·센서 경계

실제 `t02`의 연속 tick 107–110을 동결된 Actor/Selector로 replay했다. 실제 6개
후보가 모두 IK projector를 통과했고 Selector가 후보 2를 골랐다. proposer 호출을
위한 decision label은 합성 입력이다. 자연 발생 alarm이나 복구 성공의 근거가 아니다.
별도의 합성 측정·IK fixture에서는 실제 B0의 `0→1→2→3→0`, 접촉 해제 대기,
위치·회전 gate, attempt_1 전환과 실제 Controller/Failure의 상태 reset을 확인했다.
이 숫자는 재파지의 idle→open→retreat→new-pregrasp→idle 단계다. `attempt_1`은
같은 episode 안의 두 번째 파지 시도이며 close/lift 같은 B0 파지 phase와 구분한다.

registry의 미인증/shadow/offline 메타데이터는 유지했다. episode의
`grasp_pose_generator.enabled=true`는 그 metadata와 별도로 실행을 허용한다.
앞선 27-tick 실행에서 호출되지 않았다는 이유로 실행이 차단됐다고 볼 수 없다.
[앞선 기록](grasp_episode_continue.md)의 설명도 정정했다.

hold 관측기와 정상 hold/tick budget/wall budget/IK 종료는 dependency double을
사용한 8개 테스트·33개 조건으로 검사했다. AnyGrasp 입력·geometry·SSH·센서·모델
좌표 검사 및 설치된 **74개 물체 선택과 상판 배치 검사**도 통과했다. 저장된 RGBD의
geometry replay는 clock을 동결한 검사로 live freshness와 구분한다.

NONREAL의 RGBD는 rayrai, 실센서 검사는 railab USB D405를 사용한다. D405는
로봇과 분리하여 실제 RGBD, production plugin의 v2 입력 및 기존 local licensed
SDK의 camera-frame 후보 생성을 확인했다. 연속 캡처 중 실제 FrameAdapter 검사는
957점, 입력 age **0.00797초 ≤ 0.25초**, 결과 age **0.29719초 ≤ 2초**로 통과했다.
원래 timestamp를 유지했으며 SDK·카메라를 정리했다. 세부 수치는 검증 JSON과
evidence의 `d405/`, `d405_fresh/`를 따른다. EE/world 변환이
좌표 fixture인 검사는 실측 FK·실물체 인식·WORLD grasp·IK/경로 자격을 뜻하지 않는다.

기존 D405 통합 probe의 일부는 callback/RPC timeout과 부족한 timestamp triplet으로
실패했다. 원본은 유지했다. 별도 client pool의 CameraInfo RPC 5/5와 baseline의
depth/RGB/v2 31개 timestamp 일치는 통과했지만 모든 probe가 통과한 것은 아니다.

## C 세션과 배포

사용자가 별도로 발주한 세션 `grasp_iteration C`는 기존 운영 설치의 private `install/config` 복사본에서
두 팔 14-DOF와 고정 EE-stub인 `tool-empty_L`을 검사했다. joint_test/get_ready,
rest, 저장 trajectory, editor 선택 상태, swing의 실행·hold·handoff가 확인됐다.
실측 joystick 조작·GUI·swing 정확도는 검사 범위 밖이다. 동일한 `stub-0_L`로
교체하는 설정은 설치된 고정 profile이 거부했다.

controller root manifest 5개는 이미 source commit `453b987`에서 고쳐져 있었다.
C는 그 정확한 파일을 private config에만 반영해 검증했다. 운영 source/install의
배포 지연을 확인한 것이며 운영 설치를 교체하지 않았다. 기본 EE-stub profile의
grasp 직접 load는 default-relative episode JSON 부재로 실패했다. 이 경로를
성공으로 세지 않으며 본 검사의 single-L+EE 전용 episode와 구분한다.
C의 최종 결과·334개 증거 checksum·185개 보호 파일 불변·정리를 확인했다.

상판 설정은 `grasp_iteration:configs/deployment_preparation.yaml`에 유지한다.
크기 **0.70×0.44 m**, 높이 **0.78 m**, 주석 `height of railab sliding table`,
제안 중심 **[0, 0.60] m**, 기본 물체 중심 XY **[0, 0.55] m**다.
선택한 L arm/EE/D405/tactile URDF revision은 `4b313b7e`다.
Git 주소·fork 사유·최종 source pin은 [source lock](../config/raipal_sources.lock.json)에 있다.

## 증거와 재실행

local root는 `/home/jiseong/raisim_ws/isolation/nonreal_episode_closure_20261024`,
railab root는 `/home/raipal/workspace/jiseong/isolation/nonreal_episode_closure_20261024`다.
각 root의 `evidence/`와 `C_default_nonreal/`에 raw 입력·실패·최종 결과·명령을 보관한다.
격리 node는 이전 `episode_continue_20261024/master/install`과 봉인된
`inference_bundle_002`를 재사용한다. 변경한 B0만 별도 빌드·마운트했으며 기존
학습 simulator extension을 다시 빌드했다고 주장하지 않는다.

다음은 **railab에서 node/SDK를 시작하지 않는 t07 preflight**다. 전체 검사 목록을
재실행하는 예시는 아니다. t07을 실제 재검사할 때는 가용 자원과 공용 작업을
확인하고 `subprocess.run` 바로 앞에 `argv.append('--execute')`를 추가한다. 새 run 이름을
사용해야 하며 과거 결과를 덮어쓰지 않는다.

```sh
python3 - <<'PY'
import json, subprocess
from pathlib import Path
root = Path('/home/raipal/workspace/jiseong/isolation/nonreal_episode_closure_20261024')
argv = json.loads((root / 'evidence/harness/launch_argv_t07.json').read_text())
assert '--execute' not in argv
run = root / 'evidence/recheck_001'
if run.exists():
    raise FileExistsError(run)
argv[argv.index('--run-dir') + 1] = str(run)
subprocess.run(argv, check=True)
PY
```

node namespace는 private network/PID/IPC/shm, 읽기 전용 root, 표준 pseudo device와
렌더용 NVIDIA device만 사용한다. USB/로봇/joy/EE/EtherCAT 장치를 노출하지 않는다.
실센서 probe만 별도 namespace에서 D405를 연다. 임시 job과 소유한 process만 정리한다.

## 남은 자격 범위

지속 100 Hz deadline 자격은 확보하지 않았다. 2,000-tick synthetic IPC 검사에는
232 tick의 10 ms 초과가 있었으며 짧은 30-tick 검사 통과가 이를 대체하지 않는다. 실제 native
검사도 relaxed timing이었다. 자연 실행의 grasp 성공과 자율 실패 복구 성능은
추가 독립 episode 평가가 필요하다. 이번 작업에서 학습·모델 승격은 하지 않았다.

실제 base/world 정합·테이블/카메라 시야·현재 자세→ready/관측 이동은 실기에서
검증해야 한다. 두 자세 모두 실제 구동한 적이 없다. 물체 내려놓기·개방·ready 복귀·
재배치·다음 episode 자동화는 [기존 후속 계획](grasp_episode_preparation.md#에피소드-이후-동작의-구현-위치)을 유지한다.
