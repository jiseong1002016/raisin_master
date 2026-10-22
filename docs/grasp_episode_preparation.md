# Grasp episode 준비 기록 — 2026-10-23

작업 브랜치: `feat/grasp-episode-preparation-20261023`.
코드로 물체를 선택하고 L arm + EE + D405 + tactile을 준비한 뒤,
joy A1으로 한 에피소드를 시작한다. B1 또는 기존 종료 조건으로 끝나면
측정 자세를 유지하고 EE STOP을 보낸다. 이번 작업은 소스 준비와 격리 검증이며,
실제 로봇 동작·학습·운영 중인 설치본 변경은 수행하지 않았다.

## 현재 실기 실행을 막는 조건

- 테이블 상면 기본값은 **0.78 m**다. 코드에 `height of railab sliding table`
  주석을 두었다. 기존 테이블 면적과 ready 자세를 그대로 높이면 충돌 또는 IK
  제한 검사가 실패한다. 실제 상판의 XY 위치·유효 면적과 안전한 ready 경로를
  확정해야 한다. 실패한 export에서 실행용 receipt를 만들지 않는다.
- local의 AnyGrasp SDK warmup은 `libcrypto.so.1.1` 누락으로 실패했다.
  SDK가 요구하는 Python ABI·MinkowskiEngine·graspnetAPI·pointnet2·CUDA 환경도
  맞춰야 한다. railab에는 SDK가 없으므로 해당 기기의 유효한 라이선스와
  checkpoint를 설치해야 한다. 초기 pose를 임의 값으로 대체하지 않는다.
- 카메라의 최신 파일 간 변환은 일치시켰지만 mount의
  `calibration_verified`는 여전히 `false`다. 실제 hand-eye 보정 완료를 뜻하지 않는다.
- 두 tactile pad의 실제 장치 연결·baseline, EE homing·enable·구동 제한과
  학습 모델의 실기 적합성은 기존 commissioning 절차로 확인해야 한다.

이 조건을 해결하기 전에는 아래 준비 명령의 출력만 검토한다.
`.78 m`에서 통과하는 실기용 `source_start.json`은 아직 없다.

조건 | 설정·절차 | 필요한 완료 근거
EE homing/구동 | [HOMING.md](../src/raisin_plugin/ee/raisin_raipal_ee/HOMING.md), [maintenance 검사](../src/raisin_plugin/ee/test/ee_maintenance_sequence_README.md) | 실제 core의 qualified·homed·enabled와 읽힌 제한값
tactile | [plugin README](../src/raisin_plugin/tactile/raisin_tactile_plugin/README.md), 해당 params.yaml | 좌우 pad 식별·unloaded baseline·fresh frame
camera | [mount.nominal.json](../src/raisin_plugin/raisin_d405_plugin/resource/mount.nominal.json) | 별도로 측정한 extrinsic과 optical/world 기준점 일치
table/ready/world | source environment의 ready_pose·고정 base 변환, export의 placement 옵션 | 측정 좌표와 native IK·접촉 검사 통과 receipt
AnyGrasp | episode의 camera.anygrasp와 아래 backend --check | 해당 기기에서 ABI·license·CUDA warmup 통과

table/world/hand-eye의 측정·보정 절차를 자동으로 수행하는 도구는 이번 범위에
없다. 이 항목은 측정값과 별도 commissioning 기록을 받아야 해제할 수 있다.

## Git 연결과 소스 소유권

정확한 package revision·fetch 주소는 [raipal_sources.lock.json](../config/raipal_sources.lock.json)에
기록한다. root 저장소가 `src/*`의 독립 저장소를 무시하는 기존 구조는 유지하고,
버전 고정은 이 lock과 각 저장소의 submodule gitlink로 남긴다.

저장소 | 기본 연결 / 결과 | 이번 변경을 받는 곳
raisin_master | raionrobotics/raisin_master, 읽기 가능·push 권한 없음 | jiseong1002016/raisin_master
grasp_iteration | jiseong1002016/grasp_iteration | 동일 저장소
raisin_raipal | railabatkaist/raisin_raipal, push 가능 | 동일 공식 저장소
raipal_controller | railabatkaist/raipal_controller, 읽기 가능·push 권한 없음 | jiseong1002016/raipal_controller
raisin_third_party_raipal | railabatkaist/raisin_third_party_raipal, 읽기 가능·push 권한 없음 | jiseong1002016/raisin_third_party_raipal
raisimRaipal | railabatkaist/raisimRaipal, 읽기 가능·push 권한 없음 | jiseong1002016/raisimRaipal
raipal_kinematics | railabatkaist/raipal_kinematics, 최신 main을 submodule로 고정 | 공식 저장소 그대로
HEAD-URDF | railabatkaist/HEAD-URDF, 최신 main | 공식 저장소 그대로
raisin_raipal_seed | railabatkaist/raisin_raipal_seed, 최신 main | 공식 저장소 그대로
RAIPAL-URDF | railabatkaist upstream도 연결, 현재 L/EE 모델 revision은 개인 fork에 있음 | jiseong1002016/RAIPAL-URDF
raisin_gui | raionrobotics 주소 접근 실패, 개인 fork fetch 성공 | jiseong1002016/raisin_gui
vcpkg | microsoft/vcpkg | 기존 submodule 유지

요청한 공식 다섯 주소는 모두 읽을 수 있었다. 그중 third-party와 raisimRaipal은
변경분을 공식 저장소에 push할 권한이 없어 개인 fork에 게시했다. 공식 `origin`은
유지하고 개인 remote를 추가했다. wrapper submodule은 새 수정 commit을 가져올 수
있는 개인 fork URL로 고정했다. kinematics·HEAD-URDF·seed는 공식 주소를 사용한다.
RAIPAL-URDF는 공식 main에 없는 현재 모델을 재현하기 위해 개인 URL을 유지한다.

local 정식 소스는 `~/raisin_master`와 `~/raisim_ws/grasp_iteration`이다.
railab 정식 소스는 `/home/raipal/workspace/jiseong/raisin_master_sources/`의 각 저장소와
`/home/raipal/workspace/jiseong/grasp_iteration`이다. root의 `src/`에서 독립 package
소스를 연결한다. 기존 `/home/raipal/raisin_master`와 그 runtime은 별도 운영본이며
이번 소스 동기화로 바이너리가 갱신되지는 않는다.

EE·D405·tactile은 root 저장소 소유다. tactile의 새 위치는
`src/raisin_plugin/tactile/{raisin_tactile_sensor,raisin_tactile_plugin,raisin_gui_tactile_window}`다.
로봇·plugin·GUI CMake와 grasp include 경로도 함께 바꿨다. 메시지 생성은 기존
workspace setup을 사용하며 `RAISIN_GENERATED_INCLUDE`로 별도 생성 경로를 지정할 수 있다.
`src/raisin_raipal/resource/raipal_urdf`는 사용하지 않는 과거 snapshot이고,
현재 입력은 git submodule인 `resource/urdf_raipal`이다. 과거 snapshot과 실험 복사본은
이번 작업에서 제거하거나 덮어쓰지 않았다.

## 물체와 준비 자세 선택

기존 scene index 0–2를 보존하고 설치된 `rsc/objects/*/textured_vhacd.obj` 전체를
노출한다. 현재 **74개 물체**다. 추가 scene의 질량·anchor는 시뮬레이션용 초기값이며
측정된 물체 정보나 AnyGrasp 결과가 아니다. 기존 학습 catalog와 학습 테이블 높이는
변경하지 않고 deployment export에만 0.78 m를 적용한다.

local에서 선택 목록만 확인하는 명령은 native 환경을 로드하지 않는다.

```bash
python src/raipal_controller/raisin_raipal_grasp_controller/scripts/export_start_scene.py \
  --project /home/jiseong/raisim_ws/grasp_iteration --list-objects
```

`--scene-index N` 또는 `--object-id ID`로 선택한다. export 시
`--object-xy-m X Y`, `--support-center-xy-m X Y`, `--support-size-xy-m X Y`로
측정한 물체 중심과 테이블의 유효 바닥 면적을 반영할 수 있다. 좌표는 **source world**
기준이며 실제 base와 source world의 고정 변환을 먼저 맞춰야 한다.
support size는 source bucket의 내부 바닥 XY이고, 기존 wall 두께는 외곽에 더해진다.
source environment의 ready pose도 함께 검토한다. 기존 bucket의 벽·lip 설정까지
실제 테이블과 같은 것으로 간주하면 안 된다.

export는 원래 native reset IK·열린 손가락·관절 제한·TCP·접촉 검사를 수행한다.
통과한 파일의 `start.ready_pose_q7`, `start.arm_q7`, `start.finger_rad2`가 준비 동작의
근거다. 실제 물체 배치는 그 파일의 `scene.centered_object_pose`와
`raw_body_origin_pose`를 구분해서 읽는다. mesh 원점은 물체 중심과 다를 수 있다.
선택 가능하다는 것은 74개 물체가 모두 현재 상판에서 도달 가능하다는 뜻이 아니다.

local에서의 전체 준비 예시는 다음과 같다. `grasp_work`는 새 디렉터리를 사용한다.
첫 export는 현재 기본 footprint/ready에서는 실패하는 것이 확인된 상태다.
측정값과 ready 경로를 반영한 `--environment-config`·placement 옵션으로 qualification을
통과시킨 뒤에만 두 번째 명령 묶음으로 진행한다.

```bash
cd /home/jiseong/raisin_master
grasp_project=/home/jiseong/raisim_ws/grasp_iteration
grasp_package=/home/jiseong/raisin_master/src/raipal_controller/raisin_raipal_grasp_controller
grasp_work=/home/jiseong/raisim_ws/isolation/commissioned_table_episode
/home/jiseong/anaconda3/bin/python "$grasp_package/scripts/export_start_scene.py" \
  --project "$grasp_project" --object-id 005_tomato_soup_can \
  --table-top-z-m 0.78 --output "$grasp_work/source_start.json"
```

`source_start.json`이 실제로 생성된 경우에만, receipt가 지정한 원본 combined URDF로
별도 mimic plant를 생성한다. source URDF는 수정하지 않는다.

```bash
grasp_source_urdf=$(/home/jiseong/anaconda3/bin/python -c \
  'import json,sys; print(json.load(open(sys.argv[1]))["resources"]["combined_urdf"]["path"])' \
  "$grasp_work/source_start.json")
/home/jiseong/anaconda3/bin/python "$grasp_package/tools/generate_native_mimic_plant.py" \
  --source-urdf "$grasp_source_urdf" --output-urdf "$grasp_work/native_model/combined_native_mimic.urdf"
/home/jiseong/anaconda3/bin/python "$grasp_package/scripts/run_example.py" \
  --prefix /path/to/fresh/compatible/install --run-dir "$grasp_work/run" \
  --source-start "$grasp_work/source_start.json" \
  --plant-urdf "$grasp_work/native_model/combined_native_mimic.urdf" \
  --anygrasp-sdk-dir /path/to/licensed/grasp_detection \
  --anygrasp-checkpoint /path/to/checkpoint_detection.tar \
  --anygrasp-python /path/to/sdk/python --prepare-only
```

`--prefix`에는 새 코드·호환 SDK·`configure_runtime.py`로 연결한 inference bundle이
설치되어 있어야 한다. [빌드 문서](raipal_build.md)와 controller README의 install 절차를
따른다. 출력은 `run/episode.json`, `run/home`, `run/scene.json`, `run/example.json`이다.
backend만 확인할 때는 같은 prefix에서 아래 명령을 사용한다. SDK 문제가 있으면
`setup_required`, 성공하면 `loaded`와 `hardware_opened: false`를 출력한다.

```bash
GRASP_PYTHON=/path/to/inference/python \
  /path/to/fresh/compatible/install/scripts/raisin_raipal_grasp_controller/scripts/run_backend.sh \
  --config /path/to/run/episode.json \
  --b0-library /path/to/fresh/compatible/install/lib/libraisin_grasp_b0.so --check
```

railab에서는 이 프로젝트를 `/home/raipal/workspace/jiseong/grasp_iteration`으로,
package를 `raisin_master_sources/raipal_controller/raisin_raipal_grasp_controller`로
치환하고 run/prefix도 해당 workspace 아래 새 경로로 지정한다. 실기 준비에는
`--mode real --hardware-config <실제 값이 채워진 JSON>`을 추가한다.

## real/nonreal의 공통 초기 grasp

실센서는 D405 depth optical frame으로 color를 정렬하고 실제 intrinsics로
XYZ를 만든다. nonreal은 rayrai가 현재 world와 동일한 camera mount에서 RGBD를
렌더한다. 기본 `simulation_source`는 `rayrai`다. `world`/`plane`은 명시적인 검사
fixture이며 AnyGrasp 실행 입력으로 허용하지 않는다.

두 입력은 `raisin_d405_grasp_input_v2`의 XYZ + `colors_rgb_u8`로 합쳐지고,
이후 robot self filter → AnyGrasp → 측정 L_ee와 camera extrinsic으로 world 변환 →
기존 IK/B0/정책 경로를 공유한다. `acquisition_source`만 `rayrai`/`d405`로 다르다.
최대 12,000점과 4 MiB IPC 한도를 사용한다. AnyGrasp가 grasp를 찾지 못하거나
관측·결과가 오래되면 실행을 거부하며 scripted pose로 fallback하지 않는다.

SDK 모델은 최초 촬영 전에 별도 process에서 준비한다. A1 이후의 새 RGBD를
사용하고 추론 후에는 새로운 EE/tactile 상태로 정책을 시작한다. 추론 중 팔이
움직였거나 결과가 `max_result_age_s`(기본 2초)를 넘으면 거부한다.
SDK 원본의 ABI symlink나 라이선스는 변경하지 않는다.

`run_example.py`의 공통 준비 옵션:

```text
--mode real|nonreal
--source-start <검사를 통과한 source_start.json>
--plant-urdf <그 receipt에 대응하는 combined native mimic URDF>
--anygrasp-sdk-dir <현재 기기용 SDK의 grasp_detection 폴더>
--anygrasp-checkpoint <checkpoint_detection.tar>
--anygrasp-python <SDK ABI/CUDA에 맞는 Python>
--prepare-only
```

동일 설정은 `ANYGRASP_SDK_DIR`, `ANYGRASP_CHECKPOINT`, `ANYGRASP_PYTHON`으로 전달할 수 있다.
real은 `--hardware-config`를 추가한다. 예시는 grasp project의
`configs/raisin_real.example.json`이며 tactile 포트·EtherCAT interface는 실제 값을
채워야 한다. 같은 값을 임의 장치 두 개로 복사하지 않는다.
`--prepare-only`는 node·카메라·모터를 실행하지 않고 별도 run 폴더에 설정과 명령을 쓴다.
backend `--check`는 카메라 없이 모델과 AnyGrasp warmup을 검사한다.

## 제어기·joy·종료 흐름

real 기본 profile은 `single_L_ee`, nonreal은 `single_L_ee_nonreal`이다.
기존 `dual_arm_left_ee`도 지원하고 R arm은 측정 자세를 유지한다.
7/14 DOF를 명시적으로 검사하며 실행 중 DOF 변경을 거부한다.
실기 core 축 순서 `[joint9,joint8]`와 source `[joint8,joint9]`는
`devices.ee_coordinate_map=left_ethercat_joint8_joint9`로 한 번만 서로 바꾼다.
nonreal은 `source_joint8_joint9`다. 부호나 calibration 한계값은 임의 변경하지 않았다.

1. 기존 commissioning으로 homing·enable·센서·baseline을 준비한다.
2. 생성된 `raipal_grasp_start_episode` 명령이 손가락을 열고
   `raipal_trajectory`로 ready → pregrasp → 정지 대기 행을 실행한다.
   이 pregrasp는 source export의 anchor와 열린 손가락을 바탕으로 IK 검사한
   staging 자세다. A1 뒤의 AnyGrasp G0와는 별개이며, 이후 접근은 G0를 사용한다.
3. 도착·속도·손가락 안정화 후 `raipal_grasp`를 로드한다.
4. `[grasp][ready] ... [A1]` 표시 후 자세와 EE heartbeat를 유지하며 기다린다.
   준비 전 A1은 버린다. 대기 중 센서 유효성이나 준비 자세를 잃으면 종료한다.
5. **A1**으로 에피소드를 요청한다. 최신 RGBD와 초기 grasp 검사가 끝나야
   접근 → 닫기 → lift → hold와 기존 실패 감지·재제안이 진행된다.
6. **B1** 또는 기존 hold/시간/센서/IK/제어 제한 종료로 측정 자세 유지 + EE STOP을
   수행한다. B1은 trajectory의 rest 복귀와 달리 즉시 종료한다.
7. 동일 controller load에서 재시작하지 않는다. 다음 에피소드는 물체와 환경을
   다시 준비하고 명시적으로 reload한다.

`result.json`의 `operator_B1` 등 종료 사유는 실제 grasp 성공 판정과 별개다.
real의 `independent_grasp_success`는 계속 `null`이다.
B1은 초기 추론 중에도 native 제어에서 정지하지만, 추론 process를 강제 종료해야
하는 경우 backend 결과 요약이 저장되지 않을 수 있으므로 native 종료 로그도 보관한다.
실제 8개 linkage 상태·effort는 2개 motor encoder만으로 독립 측정되지 않는다.
기존 추정·모델 입력 적합성 제한은 그대로 적용된다.

## 최신 mount와 입력 계약

선택한 combined URDF의 `L_ee → ee_base`와
`src/raisin_plugin/raisin_d405_plugin/resource/mount.nominal.json`을 합성한다.
현재 L_ee 기준 camera translation은 약 **[0.08660254, -0.027, -0.05] m**다.
템플릿과 launcher를 이 변환에 맞췄으며 run 준비 시 설치된 mount로 다시 계산한다.
회전까지 포함한 원본 행렬과 mount를 함께 검토해야 한다.
임시 `node_contract.py`와 관련 검사는 삭제 상태를 반영했고 복구하지 않았다.

## 에피소드 이후 동작의 구현 위치

현재 terminal hold 지점은 유지한다. 다음 순서가 필요할 때만 별도 recovery
orchestrator를 start utility와 같은 package의 `scripts/`에 추가한다.

단계 | 기존 구현을 재사용할 위치 | 완료 판단
내려놓기 | `raipal_trajectory`의 별도 검증된 placement preset | EE를 닫은 상태로 위치·속도 안정화
손가락 열기 | core EE service의 TARGET/상태 검사 | 실제 opening 피드백, 물체 해제 확인
ready 복귀 | 기존 trajectory ready preset | 관절·속도·충돌 검사 통과
물체 재배치 | 우선 operator 단계로 유지 | 실제 배치와 새 RGBD 확인
다음 에피소드 | 새 run/episode id와 controller reload | 새 baseline/센서/pose 검사 후 A1 대기

정책의 `Episode.finish()`나 real-time `advance()`에 이 동작을 덧붙이지 않는다.
terminal 결과를 먼저 남기고 각 단계 실패 시 hold + EE STOP으로 끝내며,
사용자가 recovery를 명시적으로 시작하도록 연결한다. 기존 B1은 계속 즉시 정지한다.

## 이번 검증 범위

- 74개 object 선택·0.78 m 배치·기존 학습 기본값 보존 검사.
- real/nonreal launcher, 모델 설정, EE 좌표 변환, RGBD/AnyGrasp 경계와
  실제 Unix socket 초기화·B1 종료 기록 회귀 검사.
- A1 상태 게이트와 7/14 DOF CSV의 실제 production writer 검사.
- RaiSim 2.9.4 격리 CMake에서 grasp/controller/B0/start utility 빌드와 CTest 2/2 통과.
- D405 및 relocated tactile은 기존 railab SDK에서 전체 plugin 빌드 통과.
  tactile의 USB process·GUI 빌드와 hardware-free CTest 5/5도 통과했다.
  이 SDK는 RaiSim 1.1.8을 표시하므로 전체 2.9.4 workspace ABI 검증은 별도로 필요하다.
- 실제 rayrai의 알려진 빨간 상자 렌더: 예상 전면 깊이 0.200 m와 색상 확인.

이 검사는 장치 없는 소프트웨어 검증이다. AnyGrasp의 학습된 실제 추론 결과,
실제 로봇 grasp 성공, 상판 위 안전한 전체 이동을 입증하지 않는다.
격리 산출물은 local `~/raisim_ws/isolation/episode_preparation_20261023*`,
railab `/home/raipal/workspace/jiseong/isolation/` 아래에 둔다.
소스 lock에 대응하는 명령·결과·증거 위치는
[grasp_episode_validation.json](grasp_episode_validation.json)에 기록한다.
