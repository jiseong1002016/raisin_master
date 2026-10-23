# Grasp episode 준비 — 2026-10-23

2026-10-24 추가: railab에서 local의 기존 licensed SDK를 호출하는
[AnyGrasp SSH 폴더](anygrasp_ssh.md)를 준비했다. 새 bundle과 저장된 RGBD 왕복,
장치 없는 backend 준비 검사를 통과했다. 아래의 railab 기기용 라이선스 미등록은
기존 직접 SDK 실행 경로의 결과이며, 새 SSH 경로는 local에서 추론한다.

물체 선택 → 관측 자세 → RGBD/AnyGrasp → 검증된 pregrasp → joy A1 →
한 에피소드 → 결과 기록·측정 자세 유지·EE STOP을 연결한다.
현재 근거는 격리된 시뮬레이터와 새 설치본에서 얻었다. 실제 로봇 grasp 성공이나
실제 테이블로의 전체 이동을 검증한 상태는 아니다.

## 확인한 것과 남은 것

- 기존 licensed AnyGrasp SDK를 재사용해 warmup과 rayrai front RGBD 추론을 통과했다.
  그 후보의 native pregrasp IK·접촉·열린 손가락·reset readback도 통과했다.
- node와 direct simulator의 EE self filter를 공유하며, 같은 rayrai RGBD에서
  retained XYZ/RGB가 정확히 일치했다. baseline direct simulator는 2,000 tick과
  terminal 기록까지 실행했지만 독립 grasp 판정은 `other_failure`, `success=false`였다.
- **0.78 m** deployment profile의 원본 source export와 ready → front 관측 자세
  경로를 검증했다. 7.55초 quintic의 10 ms 간격 **756개** native query를 통과했으며,
  최고 관절 속도는 0.29968 rad/s다. 원본 source receipt와 D2 결과는 보존했다.
- 원시 front/fresh RGBD를 production FK·camera transform·공유 robot filter·geometry
  코드로 재생했다. 같은 물체의 translation 차이는 **4.49 × 10⁻⁷ m**였고,
  원시 관측의 +2 cm 이동은 거부했다. 최초 SDK G0는 다시 추론하거나 바꾸지 않았다.
- railab의 새 RaiSim/rayrai **2.9.4** 전체 workspace build/install, 장치 없는 CTest
  **8/8**, 설치 **82 ELF**의 의존성 검사를 통과했다. local의 `local_prefix`도
  운영 설치와 분리했다. 자세한 경계는 [빌드 문서](raipal_build.md)를 따른다.
- 최종 local NONREAL node에서 AnyGrasp·pregrasp 준비, **A1 전 2.03초 대기 →
  A1 → 실제 제어 26 tick → B1/result → 0.5초 정지 유지**를 통과했다.
  controller `1fa3066`, D405 renderer `c1947a2`를 사용했다. C의 실제 장치는 사용하지 않았다.
- 순수 CTest 8개는 controller `1fa3066`에서 마지막으로 실행했다.
  Python 6개 검사의 역사적 결과는 `node_evidence/final_python_checks.json`,
  EE schema 2/3 회귀 검사 3개는 `node_evidence/ee_schema_95edbd4_checks.json`에 있다.

fresh pregrasp에서 SDK를 다시 실행했을 때 기존 G0와 후보가 일치하지 않는 현상을
확인해 controller `b32a19d`에서 새 geometry 검사로 대체했다. 최초 SDK 후보 중
선택한 mesh의 범위·jaw aperture·IK·native 경로를 통과한 G0와 원래 추론 시각을
고정한다. A1 뒤의 별도 관측 시각으로 환경을 재검증하며 SDK 후보 순위를 다시
맞추지 않는다. 최신 수정의 실행 근거는 아래 최종 smoke 항목에서 관리한다.

nonreal의 geometry 검사는 알려진 mesh와 source orientation을 전제로 한 translation
검사다. REAL은 source pose 주변의 local rigid 정합에서 6 DoF 관측성과 품질을
모두 요구한다. 기본 캔은 yaw 관측성이 부족해 이 검사에서 거부되므로
**실기 준비가 검증된 물체가 아니다**. 정합이 통과해도 전역 물체 식별, 대칭 형상의
방향 구분, 가려진 handle·주변 장애물이나 실제 grasp 성공을 증명하지 않는다.

실기에는 다음 근거가 더 필요하다.

- 실제 table XY·유효 면적·base/world 정합과 최근 camera 설정의 실측 확인자료.
  현재 0.78 m profile의 XY/면적은 합성 설정이다.
- 실제 현재 자세 → ready 경로와 주변 장애물·다른 팔까지 포함한 검사.
  ready → front의 샘플 검사는 이 구간이나 연속 충돌 무발생을 증명하지 않는다.
- 두 tactile pad 식별·baseline, EE homing·enable·구동 제한, 독립적인 실기 성공 판정.
  실기 `independent_grasp_success`는 계속 `null`이다.
- C가 사용하는 공용 실기 장치·기존 노드의 사용 해제 확인. 격리 NONREAL 작업 범위의
  확인을 실기 장치 사용 허가로 확대하지 않는다.

여기서 C는 사용자가 `grasp_iteration C`라고 부르는 동시 작업 세션이다.
local 격리 범위의 합의는 evidence root의 `C_nonreal_slot_request.txt`와
`C_nonreal_slot_reply.txt`에 보존했다. 실기 실행 전에는 해당 세션에 host·profile·
장치·실행 범위를 전달하고 명시적인 node/device slot 해제 응답을 받은 뒤
프로세스·포트 점검을 다시 수행한다.

commissioning 절차는 [EE homing](../src/raisin_plugin/ee/raisin_raipal_ee/HOMING.md),
[maintenance 검사](../src/raisin_plugin/ee/test/ee_maintenance_sequence_README.md),
[tactile](../src/raisin_plugin/tactile/raisin_tactile_plugin/README.md)를 따른다.

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
노출한다. 현재 **74개 물체를 선택**할 수 있다. 전체 74개의 IK·경로·실기 grasp를
인증한 것은 아니다. 추가 scene의 질량·anchor는 시뮬레이션용 초기값이며
측정된 물체 정보나 AnyGrasp 결과가 아니다. 기존 학습 catalog와 학습 테이블 높이는
변경하지 않고 deployment export에만 0.78 m를 적용한다. 코드 주석은
`height of railab sliding table`이다. 기본 물체 XY는 `[0, 0.55]` m,
support 중심은 `[0, 0.60]` m, 내부 바닥은 `0.4 × 0.4` m다.

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

## 장치 없는 재현 명령

아래는 검증에 사용한 격리 경로다. 다른 clone에서는 package·project·prefix·SDK 경로를
해당 checkout에 맞춘다. `grasp_work`와 출력 JSON은 새 경로를 사용한다.
선택한 source native extension도 같은 source/URDF와 RaiSim 2.9.4로 빌드되어 있어야 한다.

```bash
grasp_base=/home/jiseong/raisim_ws/isolation/episode_completion_20261023
grasp_project="$grasp_base/project"
grasp_package="$grasp_base/controller/raisin_raipal_grasp_controller"
grasp_prefix="$grasp_base/local_prefix"
grasp_sdk=/home/jiseong/raisim_ws/raisim2Lib_v294
grasp_python=/home/jiseong/anaconda3/bin/python
grasp_work=/home/jiseong/raisim_ws/isolation/episode_reproduction_001
export PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export LD_LIBRARY_PATH="$grasp_prefix/lib:$grasp_sdk/raisim/lib:$grasp_sdk/rayrai/lib:$grasp_sdk/raisim/lib/openusd"
"$grasp_python" "$grasp_package/scripts/export_start_scene.py" \
  --project "$grasp_project" --object-id 005_tomato_soup_can \
  --table-top-z-m 0.78 --output "$grasp_work/source_start.json"
```

`source_start.json`이 실제로 생성된 경우에만, receipt가 지정한 원본 combined URDF로
별도 mimic plant를 생성한다. source URDF는 수정하지 않는다.

```bash
grasp_source_urdf=$("$grasp_python" -c \
  'import json,sys; print(json.load(open(sys.argv[1]))["resources"]["combined_urdf"]["path"])' \
  "$grasp_work/source_start.json")
"$grasp_python" "$grasp_package/tools/generate_native_mimic_plant.py" \
  --source-urdf "$grasp_source_urdf" --output-urdf "$grasp_work/native_model/combined_native_mimic.urdf"
"$grasp_python" "$grasp_package/scripts/qualify_observation_start.py" \
  --prefix "$grasp_prefix" --source-start "$grasp_work/source_start.json" \
  --plant-urdf "$grasp_work/native_model/combined_native_mimic.urdf" \
  --arm-q7 .9158186869758642 2.445622310895565 -2.0323909230609063 \
    -2.139308544919076 -.9102024715179059 -1.9266070815518939 1.6452597416690993 \
  --duration-s 7.55 --output "$grasp_work/front_observation_start.json"
```

관측 자세 수치는 이 scene용 NONREAL 설정이다. CLI는 source ready/fingers, URDF
mimic의 의미상 q8, source FK, 실제 설치 `libraisin_grasp_b0.so`의 native 경로 검사를
사용한다. 속도·가속도·관절 제한·충돌 검사 실패 시 plan을 쓰지 않으며 기존 파일을
덮어쓰지 않는다. 다른 물체·table 배치·mount·plant에는 새 plan이 필요하다.

출력 schema는 `raipal_anygrasp_observation_start_v1`이다. `source_start_sha256`,
`camera_to_ee_sha256`, `plant_urdf_sha256`, `effective_scene_sha256`은 원본 receipt,
camera transform, plant, front 초기 자세를 포함한 최종 scene을 묶는다.
`arm_q7`, `ee_pose_world`, `trajectory_duration_s`, `preparation_timeout_s=180`,
`cache_max_age_s=2`와 qualification도 기록한다.
`nonreal_only: true`, `real_robot_qualified: false`이며 real launcher는 이를 거부한다.
원래 staging 자세의 D2 receipt를 front 자세의 검증으로 바꿔 붙이지 않는다.

다음은 설정 생성 예시다. prefix에는 `configure_runtime.py`로 검증된 inference bundle을
연결하고, 그때 사용한 별도 seed HOME을 사용한다. C의 HOME override를 가져오지 않는다.

```bash
grasp_seed_home=/path/to/isolated/seed_home
HOME="$grasp_seed_home" "$grasp_python" "$grasp_package/scripts/run_example.py" \
  --prefix "$grasp_prefix" --python "$grasp_python" --run-dir "$grasp_work/run" \
  --source-start "$grasp_work/source_start.json" \
  --plant-urdf "$grasp_work/native_model/combined_native_mimic.urdf" \
  --observation-plan "$grasp_work/front_observation_start.json" \
  --anygrasp-sdk-dir /home/jiseong/external/anygrasp_sdk/grasp_detection \
  --anygrasp-checkpoint /home/jiseong/Desktop/Anygrasp/checkpoint_detection.tar \
  --anygrasp-python "$grasp_base/anygrasp_runtime/python" --prepare-only
```

`--prefix`에는 새 코드·호환 SDK·`configure_runtime.py`로 연결한 inference bundle이
설치되어 있어야 한다. [빌드 문서](raipal_build.md)와 controller README의 install 절차를
따른다. 출력은 `run/episode.json`, `run/home`, `run/scene.json`, `run/example.json`이다.
이번 격리 설치는 읽기 전용 배포를 지원하는 `inference_bundle_003`을 연결했다.
95개 파일의 seal에는 빈 registry lock도 포함한다. 읽기는 기존 lock을 read-only로
열며, 새 학습 registry의 생성·쓰기는 기존 방식대로 유지한다. bundle export는 현재
`rsc/RAIPAL-URDF/urdf/`의 `raipal_L_arm.urdf`와 EE·tactile을 포함한
`raipal_L_grasp_springtac.urdf`를 포함하며,
관측 plan은 해당 원본과 별도 생성한 mimic plant의 해시를 묶는다.
backend만 확인할 때는 같은 prefix에서 아래 명령을 사용한다. SDK 문제가 있으면
`setup_required`, 성공하면 `loaded`와 `hardware_opened: false`를 출력한다.
`backend_check_002/backend_check.log`는 이전 bundle002의 `loaded` 근거다.
bundle003에서는 실제 노드 backend가 네 모델을 로드한 뒤 `listening`까지 진행했다.
이것은 모델 성능 검증이 아니다.

```bash
GRASP_PYTHON=/path/to/inference/python \
  /path/to/fresh/compatible/install/scripts/raisin_raipal_grasp_controller/scripts/run_backend.sh \
  --config /path/to/run/episode.json \
  --b0-library /path/to/fresh/compatible/install/lib/libraisin_grasp_b0.so --check
```

### railab 격리 설치

railab에는 `/home/raipal/workspace/jiseong/isolation/episode_completion_20261023/` 아래
별도 prefix, bundle003, CPU 정책 Python, SDK 의존성, private HOME과 NONREAL 설정을
준비했다. bundle 95개 파일과 실제 read-only mount에서 네 모델의 registry 조회를
검증했다. 경로를 옮긴 source receipt는 원본을 보존해 따로 만들었고, remote native
library로 756개 관측 경로 sample을 새로 검사했다.

```bash
runtime_base=/home/raipal/workspace/jiseong/isolation/episode_completion_20261023
"$runtime_base/policy_runtime/python" -m raipal_grasp.runtime \
  --config "$runtime_base/node_evidence/runtime_staging/nonreal_prepared_003/episode.json" \
  --b0-library "$runtime_base/master/install/lib/libraisin_grasp_b0.so" --check
```

이 검사의 실제 결과는 네 모델 로드 후 exit 2, `setup_required`다.
`anygrasp_runtime/sdk/license`에 **railab 기기용 AnyGrasp 등록 자료가 없다**.
SDK wrapper `anygrasp_runtime/python`은 이 경우 Python 실행 전에 exit 78로 멈춘다.
기존 local 기기의 license는 복사하지 않았다. 약 7.85 GiB의 분리된 CUDA 의존성과
14개 import를 검사했지만 CUDA 초기화·SDK warmup·추론은 수행하지 않았다.
Open3D/MinkowskiEngine/graspnetAPI의 device 조회를 포함한 전체 import도 남아 있다.
원격 노드·장치는 시작하지 않았다.

정확한 명령과 경계는 `node_evidence/runtime_staging/remote_receipts/readiness.json` 및
`prepared_backend_check_003.json`에 있다. 현재 관측 plan은 NONREAL 전용이므로
`--mode real`을 붙여 실기 plan으로 전환할 수 없다. license 등록 뒤의 SDK 동작 확인과
실측·장치 commissioning은 앞의 실기 준비 항목에 따라 진행한다.

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

SDK 모델은 최초 촬영 전에 별도 process에서 준비한다. 초기 G0 추론과 pregrasp
준비는 A1 전에 수행한다. G0의 추론 시각과 SDK receipt는 유지하고, 준비 이동이
6초를 넘겨도 새 geometry 검사 시각을 별도로 기록한다. A1보다 새 프레임으로
검사하고, activation까지 최대 2초 이내인 결과와 새 EE/tactile 상태를 확인한다.
물체가 기준 관측에서 5 mm 넘게 움직이면 종료하며 자동 재추론·재계획하지 않는다.
REAL은 기본 ±30°·3 cm의 source-pose 주변 정합에서 회전도 관측 가능해야 하며,
기준 관측과 회전 차이가 0.087 rad를 넘으면 거부한다. nonreal은 회전을 검증하지
않았다는 제한을 receipt에 남긴다. 관측·추론 중 팔이 움직이거나 검사 관측이
만료되면 거부한다.
SDK 원본의 ABI symlink나 라이선스는 변경하지 않는다.

`run_example.py`의 공통 준비 옵션:

```text
--mode real|nonreal
--source-start <검사를 통과한 source_start.json>
--plant-urdf <그 receipt에 대응하는 combined native mimic URDF>
--observation-plan <raipal_anygrasp_observation_start_v1 JSON>
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
start utility는 CSV에 R/L 14개 관절 좌표를 명시해 7개 관절 템플릿의 L mirroring을
피한다. single-L rest override는 기존 right-arm 템플릿 좌표로 직렬화한다.
실기 core 축 순서 `[joint9,joint8]`와 source `[joint8,joint9]`는
`devices.ee_coordinate_map=left_ethercat_joint8_joint9`로 한 번만 서로 바꾼다.
nonreal은 `source_joint8_joint9`다. 하드웨어 부호·기어비·구동 제한은 유지한다.

1. 기존 commissioning으로 homing·enable·센서·baseline을 준비한다.
2. `raipal_grasp_start_episode`가 손가락을 열고 `raipal_trajectory`로
   현재 → ready, ready → front 관측 자세를 각각 실행한다(`7c05492`). 각 구간 끝은
   정지 대기 행이며, 실제 위치·양팔 속도·연속 안정 시간이 통과해야 다음 구간을
   로드한다. 오른팔 target과 누적 drift 기준은 최초 측정 자세로 유지한다.
   각 load는 새 controller instance의 preset을 한 번 실행한다. 같은 instance의
   B1/reset은 재생하지 않는다. observation-start plan은 실행용 `episode.start`와
   scene 초기 arm q만 관측 자세로 선택하며, 없으면 원래 source staging 자세를 쓴다.
   NONREAL에서는 움직이기 전에 `GetCameraInfo`의 성공과 유효한 영상 크기를 기다려
   rayrai의 첫 프레임 준비를 확인한다.
3. 도착·속도·손가락 안정화 후 `raipal_grasp`를 로드한다.
   NONREAL은 native scene·renderer cold init 동안 hold하고 첫 검증 ACK 전까지
   callback 간격 검사만 유예한다(`8efc713`). REAL은 처음부터, NONREAL도 첫 ACK
   이후에는 기존 간격 제한을 적용한다. 상태·센서·명령 검증은 그대로다.
4. 현재 RGBD로 AnyGrasp G0를 구하고, 후보가 선택한 mesh에 속하는지와
   4 cm pregrasp IK·native 경로를 검사해 이동한다. 안정화·geometry 재검증 뒤
   `[grasp][ready] ... [A1]`에서 자세와 EE heartbeat를 유지한다.
   준비 전 A1은 버린다. 대기 중 센서 유효성이나 준비 자세를 잃으면 종료한다.
5. **A1**으로 에피소드를 요청한다. 버튼 이후의 새 geometry 관측으로 고정된 G0의
   환경을 확인하고 새 EE/tactile packet을 받아야
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
이 소스 합성은 사용자가 최근 맞춘 설정과 일치한다. 템플릿과 launcher도 같은
변환을 사용하며 run 준비 시 설치된 mount로 다시 계산한다.
회전까지 포함한 원본 행렬과 mount를 함께 검토해야 한다.
`calibration_verified: false`는 저장된 flag다. 이번 시험에는 별도 물리 실측 receipt가
없으며, 이 flag만으로 최근 보정 작업을 부정하지 않는다.
임시 `node_contract.py`와 관련 검사는 삭제 상태를 반영했고 복구하지 않았다.

## 에피소드 이후 동작의 구현 위치

현재 terminal hold 지점은 유지한다. 아래는 **후속 구현 계획**이며 자동 수행하지 않는다.
start utility와 같은 package의 `scripts/`에 recovery orchestrator를 두고,
각 동작은 기존 trajectory와 EE service가 소유하도록 한다.

단계 | 기존 구현을 재사용할 위치 | 완료 판단
하강 | `raipal_trajectory`의 별도 검증된 placement preset | EE를 닫은 상태로 하강 경로·위치·속도 확인
놓기 | 지지면 접근 후 별도 force/vision 판정 | 물체 지지 확인 후에만 release 허용
손가락 열기 | core EE service의 TARGET/상태 검사 | 실제 opening 피드백, 물체 해제 확인
ready 복귀 | 기존 trajectory ready preset | 관절·속도·충돌 검사 통과
물체 재배치 | 우선 operator 단계로 유지 | 실제 배치와 새 RGBD 확인
재무장 | 새 run/episode id와 controller reload | 새 baseline/센서/관측 검증 후 다음 A1 대기

정책의 `Episode.finish()`나 real-time `advance()`에 이 동작을 덧붙이지 않는다.
terminal 결과를 먼저 남기고 각 단계 실패 시 hold + EE STOP으로 끝내며,
사용자가 recovery를 명시적으로 시작하도록 연결한다. 기존 B1은 계속 즉시 정지한다.

## 근거 위치

local 기준 root는 `~/raisim_ws/isolation/episode_completion_20261023`이다.

근거 | 확인한 범위
`anygrasp_runtime/warmup.json` | 기존 licensed runtime과 checkpoint로 실제 warmup
`sim_evidence/guarded_filtered_front_045_001/` | rayrai RGBD → 실제 SDK → native D2/reset, EE filter 정확한 parity
`sim_evidence/verified_filtered_episode_summary.json` | direct simulator baseline terminal, 독립 grasp 성공은 false
`sim_evidence/fresh_filtered_pregrasp_001/match.json` | fresh SDK 후보 matching 거부 사례
`sim_evidence/translation_freshness_002/results.json` | 알려진 mesh/orientation의 translation freshness, +2 cm 변위 거부
`sim_evidence/production_geometry_replay_001.json` | 원시 RGBD의 production FK/filter/geometry 재생, 이동 거부·캔 회전 모호성 거부
`sim_evidence/rigid_freshness_001/results.json` | 실제 mesh의 local rigid 회전 측정·6 DoF 관측성 거부 사례
`node_evidence/source_start.json` | 0.78 m source export 원본 receipt
`node_evidence/front_observation_qualification/cli_reproduction.json` | 설치 native library를 사용한 plan 재생성 명령·해시
`node_evidence/backend_check_002/backend_check.log` | 최신 URDF를 포함한 inference_bundle_002로 backend loaded, 장치 미사용
`node_evidence/dependency_manifest.json` | 82 ELF의 의존성·소스 snapshot, 누락 0·운영 framework 의존 0
`node_evidence/final_python_checks.json` | controller 071f9b1에서 Python 검사 프로그램 6개 통과
`node_evidence/isolated_node_smoke_006/` | nominal ready command보다 measured arm이 지연되어 ready 통과 검사 실패
`node_evidence/gpu_offscreen_diagnostic.txt` | 격리 namespace에서 NVIDIA offscreen OpenGL 확인
`node_evidence/isolated_node_smoke_007/` | start utility 성공·관측 자세 안정화 후 실제 ready 전 callback 간격 오류
`node_evidence/remote_refresh/final_audit_result_c1947a2.json` | 최종 source·82 ELF·설치본 일치 및 보존 검사
`node_evidence/rayrai_cache_regression/verification.json` | 동일 RGBD 검사, 이전 header 실패·수정 header 통과
`node_evidence/isolated_node_smoke_013/smoke_result.json` | 실제 격리 node의 준비·A1·26 tick·B1·정지 유지 통과
`node_evidence/isolated_node_smoke_013/postcheck.json` | bundle 95개 해시·프로세스 정리·host 8080 보존 확인
`node_evidence/runtime_staging/remote_receipts/readiness.json` | railab offline staging과 기기용 AnyGrasp 등록 미완료 구분

앞선 smoke 001(Xvfb), 002(plugin metadata), 003·004(cold renderer freshness),
005(L 좌표 mirroring), 006(measured ready 지연), 007(cold callback 간격),
008(read-only registry lock), 009(EE schema 버전), 010·011(중복 core ghost의 시야 가림)의
실패 기록을 각 실행 폴더에 보존한다. 011의 실제 SDK 입력은 6,994점이었지만
선택한 mesh 주변의 3 cm 범위에는 0점이었다. camera world pose는 관측 plan과 같았다.
`1fa3066`은 Hub와 Sim 양쪽에서 중복 L/EE 표시를 숨기고 해제 시 복원한다.
카메라와 다른 팔 표시는 유지한다. renderer의 collision filter와 visual 목록은
별개이므로 contact만 끄는 것으로는 중복 손의 시야 가림을 막을 수 없었다.
`95edbd4`의 Python은 EE schema 2/3을 받되, v3의 passive position_valid만으로는
구동을 허용하지 않는다. real/nonreal 보정값·한계·구동 자격 회귀 검사 3개를 통과했다.
012는 실제 ready·A1·B1/result까지 도달했지만 harness가 native 종료 문구를 잘못
기대해 실패로 기록했다. 또한 core visual 개수가 바뀐 뒤 rayrai가 남은 visual까지
건너뛰는 경고가 있었다. `c1947a2`는 world topology 변경 시 기존 lock 안에서
`swapWorld`로 renderer wrapper를 갱신한다. 독립된 녹색·파란색 box 검사에서
동일 새 test + 이전 header는 남은 box를 잃고 exit 1, 수정 header는 삭제·유지·복원
모두 통과해 exit 0이었다. 013에서도 이 visual-count 경고가 없었다.

### 최종 smoke

**통과 — `isolated_node_smoke_013/`**, episode `3-422293462941676`.
두 trajectory 구간의 실제 settling 후 licensed SDK가 rayrai의 self-filtered
6,994점에서 후보 rank 0을 생성했다. mesh·aperture·IK와 777개 native 경로 sample을
통과해 pregrasp에 도착했다. A1 전 **2.02975초** 동안 policy tick과 물체 staging
해제가 없었다. 한 번의 A1 이후 독립 새 geometry를 확인하고 첫 accepted command와
후속 물리 tick을 확인했다. 최초 SDK G0는 그대로 보존됐으며, 별도 geometry 관측은
A1보다 새로웠고 handoff 시 나이는 **0.05195초**였다.

한 번의 B1으로 같은 episode의 `operator_B1` 결과와 **26 active tick**을 기록했다.
그 뒤 0.5초 동안 추가 policy tick이 없었다. active arm acquisition의 최대 간격은
10.538 ms, 15 ms reference 초과는 0회였다. 이후 harness가 자기 node·joy·SDK worker를
종료했다. runtime maps는 선택한 prefix와 2.9.4 SDK에 한정됐고, 남은 작업 프로세스는
없으며 bundle 95개 해시와 host TCP 8080의 기존 PID 1944가 유지됐다.

재현용 격리 launcher와 정확한 명령은 evidence root의 `node_evidence/README.md`에
있다. 이 harness는 새 local run 폴더를 쓰며 실제 노드를 private namespace 안에서
시작한다. railab에서는 노드·장치·licensed SDK를 실행하지 않았다.
이 검증은 시작·종료 연결을 확인하며 grasp 성공을 판정하지 않는다.
실기 성공은 미측정이고 별도 direct simulator baseline의 독립 성공 판정은 false다.

아래는 **local 격리 node를 실제 시작하는** 재현 명령이다. 실행 전 메모리·GPU·
기존 node 점유를 확인한다. `node_evidence`와 bundle·SDK runtime·`local_prefix`는
이 host의 별도 근거 폴더이며 Git에 포함하지 않는다. 검증된 prefix/bundle 연결,
`node_evidence/seed_home`, compiled joy helper, local 기기용 licensed SDK와
bwrap·다섯 NVIDIA 렌더링 장치가 필요하다. 기존 출력 폴더는 거부하므로 새 번호를 쓴다.

```bash
grasp_base=/home/jiseong/raisim_ws/isolation/episode_completion_20261023
/home/jiseong/anaconda3/bin/python "$grasp_base/node_evidence/run_isolated.py" \
  --run-dir "$grasp_base/node_evidence/isolated_node_smoke_014" \
  --bundle "$grasp_base/inference_bundle_003" \
  --observation-plan "$grasp_base/node_evidence/front_observation_start.reproduced.json"
```

launcher가 node·start utility·A1/B1 helper와 SDK bridge를 시작하고 정리한다.
`inside_driver.py`나 joy helper를 host에서 직접 실행하지 않는다. 일반 실행 설정은
앞의 `run_example.py --prepare-only`가 생성하는 `example.json`의 명령을 따르되,
그 명령 자체가 C의 실기 slot 해제를 대신하지 않는다.

railab build와 CTest 로그는
`/home/raipal/workspace/jiseong/isolation/episode_completion_20261023/master/build/`에 있다.
최종 revision과 검증 기록은 [source lock](../config/raipal_sources.lock.json),
[검증 manifest](grasp_episode_validation.json)로 고정한다. direct simulator나 단위 검사를
node의 A1 → active → B1 통과 근거로 대체하지 않는다.
