# RAIPAL Linux 빌드와 격리 설치

2026-10-23 기준 railab의 별도 workspace에서 전체 RaiSim/rayrai **2.9.4**
build/install과 장치 없는 CTest **8/8**을 통과했다. 설치된 **82 ELF**의 의존성은
누락 0건, 운영 설치본의 Raisin framework 참조 0건이었다. 이를 local의 별도
`local_prefix`에 준비했다. 이 근거는 해당 manifest의 빌드 시점에 해당한다.
최종 native 입력은 controller `1fa3066`, master D405 renderer `c1947a2`다.
start utility·trajectory는 변경되지 않은 `7c05492` 빌드다. local 격리 NONREAL node의
실행 maps와 준비 → A1 → 실제 tick → B1 → 정지 유지도 확인했다.
railab offline staging은 끝났으며 기기용 AnyGrasp 등록이 남아 있다.
빌드 결과만으로 실기 장치나 에피소드 통과를 판단하지 않는다.

현재 local 증거는
`~/raisim_ws/isolation/episode_completion_20261023/node_evidence/dependency_manifest.json` 및
`node_evidence/remote_refresh/final_audit_result_c1947a2.json`,
remote build/CTest 로그는
`/home/raipal/workspace/jiseong/isolation/episode_completion_20261023/master/build/`에 있다.
manifest의 source/dependency snapshot은 해당 빌드 시점을 식별한다. 이후 source 변경은
재빌드와 해당 검사를 거쳐야 한다.

## 입력과 SDK

package revision과 fetch 주소는 [raipal_sources.lock.json](../config/raipal_sources.lock.json),
profile은 [robot configurations](robot_configurations.md)를 따른다.
`cmake/raipal_workspace.cmake`가 `src/` package 순서와 install 규칙을 소유한다.
framework·robot·controller·GUI·third-party source, submodule, generated interface
headers가 갖춰진 새 checkout에서 빌드한다. `GRASP_ITERATION_ROOT`에는 B0 header와
공유 Python 코드를 제공하는 선택한 grasp_iteration checkout을 지정한다.

EE·D405·tactile은 master가 소유한다. tactile 경로는 `src/raisin_plugin/tactile/`이며,
현재 URDF 입력은 `src/raisin_raipal/resource/urdf_raipal` submodule이다.
source-owned header를 `generated/include/`에 복사해 가리지 않는다.
ControllerManager가 읽는 각 controller package root `config/params.yaml`에는
`plugin: {}` metadata가 있어야 한다(`453b987`). 로봇별 파라미터는 별도의
`config/default/`·`config/L_ee/`에 둔다.

RaiSim과 rayrai는 같은 versioned SDK를 사용한다.

기기 | 2.9.4 SDK
local | `/home/jiseong/raisim_ws/raisim2Lib_v294`
railab | `/home/raipal/workspace/jiseong/raisim_sdks/raisim2Lib_v294`

기존 1.1.8 SDK·운영 install·runtime은 보존한다. SDK 선택을 바꾸면 새 configure,
build, install, 의존성 검사가 필요하다. vendor binary의 RPATH나 SDK license를
수정하지 않는다.

## 새 prefix로 빌드

운영 checkout의 build/install을 덮어쓰지 않고 별도 checkout과 prefix를 사용한다.
다음 변수는 새 workspace에 맞춘다. `free -h`로 메모리 여유를 확인하고 병렬도는
현재 작업과 함께 정한다. 아래 예시는 `-j2`다.

```bash
cd /path/to/isolated/raisin_master
grasp_build="$PWD/build"
grasp_prefix="$PWD/install"
grasp_project=/path/to/selected/grasp_iteration
grasp_sdk=/path/to/raisim2Lib_v294
free -h
./raisin setup --cmake-only
cmake --preset raipal-release -B "$grasp_build" \
  -DCMAKE_INSTALL_PREFIX="$grasp_prefix" \
  -DRAISIM_SDK_ROOT="$grasp_sdk" \
  -DGRASP_ITERATION_ROOT="$grasp_project"
cmake --build "$grasp_build" -j2
cmake --install "$grasp_build"
```

`--cmake-only`는 generated root CMake를 갱신하며 기존 install, 외부 의존성,
사용자 설정을 삭제하지 않는다. interface header는 미리 생성되어 있어야 한다.
일반 `raisin setup`의 clean/deploy 동작이나 기존 `CMakeCache.txt` 편집으로
workspace를 옮기는 방식은 이 절차에 사용하지 않는다.

설치 기본값과 per-user `~/.raisin/` override·home calibration은 별개다.
새 episode는 별도 seed HOME에서 `configure_runtime.py --prefix ... --bundle ...
--python ...`로 inference bundle을 연결한다. 이는 새 prefix와 그 HOME의 설정을
쓰므로 운영 prefix/HOME에 적용하지 않는다.
이번 local 노드 검증은 읽기 전용 registry를 지원하는 `inference_bundle_003`을 사용한다.
빈 `.current.lock`을 포함한 95개 파일을 seal로 검증하고, 공유 읽기는 lock을 쓰기
모드로 열지 않는다. 기존 mutable registry의 생성·쓰기와 모델 가중치는 유지한다.
exporter가 현재
`rsc/RAIPAL-URDF/urdf/`의 `raipal_L_arm.urdf`와 EE·tactile을 포함한
`raipal_L_grasp_springtac.urdf`를 export하도록 수정했고,
이전 bundle002의 backend `--check`는 `loaded`, `hardware_opened: false`를 확인했다.
bundle003의 실제 노드 backend도 네 모델을 로드한 뒤 `listening`까지 진행했다.
이 로딩 검사는 학습 정책의 성능 검증을 포함하지 않는다.

prefix에는 `release.txt`, `lib/`, `include/`가 함께 있어야 설치 경로를 올바르게
식별한다. 복사본을 만들 때 marker와 config/resource/scripts도 함께 보존한다.
일부 library만 복사한 뒤 운영 install을 덧붙여 해결하지 않는다.

## 장치 없는 검사

2.9.4 SDK와 해당 prefix만 사용하도록 library 경로를 명시한다. 선택한 외부
비-framework 의존성은 dependency manifest에 경로와 해시를 남긴다.

```bash
export LD_LIBRARY_PATH="$grasp_prefix/lib:$grasp_sdk/raisim/lib:$grasp_sdk/rayrai/lib:$grasp_sdk/raisim/lib/openusd"
ldd "$grasp_prefix/bin/raisin_raipal_node"
ldd "$grasp_prefix/lib/libraisin_grasp_b0.so"
ctest --test-dir "$grasp_build" --output-on-failure \
  -R '^(raipal_episode_start_gate_check|raipal_start_trajectory_check|tactile_contract|tactile_usb_protocol|tactile_usb_backend|tactile_v2_pipeline|tactile_shadow|ee_hardware_profile)$'
```

위 8개는 remote `Testing/Temporary/LastTest.log`에서 통과한 순수 검사다.
위 8개는 controller `1fa3066`에서 마지막으로 실행했다. D405만 바꾼 `c1947a2`에서는
그 의존성이 바뀌지 않아 반복하지 않았고, 동일 RGBD test의 이전 header 실패·수정
header 통과와 최종 node013을 확인했다. Python 6개 검사는 `071f9b1`의 역사적 결과이며
`node_evidence/final_python_checks.json`에 있다. EE schema 2/3의 별도 회귀 검사 3개는
`node_evidence/ee_schema_95edbd4_checks.json`에 있다.
GUI·plugin lifecycle·프로세스 integration은 이 묶음에 포함하지 않는다.
전체 의존성 검사에는 node/GUI뿐 아니라 설치된 controller·EE·D405·tactile plugin과
그 transitive dependency도 포함한다. `libraisim.so.2.9.4`는 선택한 SDK에서,
Raisin framework는 선택한 prefix에서 읽혀야 한다. 1.1.8, 운영 `runtime/grasp_294`·
`runtime/gui_294`, 다른 설치본의 framework가 섞이면 통과로 기록하지 않는다.

관측 시작 plan의 재현 명령과 실제 licensed AnyGrasp 근거는
[grasp episode 준비](grasp_episode_preparation.md)에 있다.
`qualify_observation_start.py`는 설치된 native library로 별도 RaiSim world를 만들며
node·USB·EtherCAT·카메라를 열지 않는다. 현재 plan은 NONREAL 전용이다.

## node 실행 검증의 경계

C가 사용하는 실제 장치와 기존 node는 아직 사용 해제가 확인되지 않았다.
실행 검증은 조정한 범위에서만 진행한다. 별도 HOME만으로 고정 POSIX shm 이름이나
TCP port 충돌이 격리되지는 않는다. 현재 local NONREAL 검증은 Raisin
node·start utility·joy를 모두 private network/PID/IPC/mount namespace에서 실행하며,
별도 `/dev/shm`·최소 `/dev`·job 전용 HOME/TMPDIR를 사용한다. C의 기존 실기 slot과
장치는 이 검증에서 사용하지 않는다. licensed SDK worker만 host에서 실행하고
job 전용 Unix socket으로 연결해 host fingerprint를 유지한다.

GPU 렌더링은 필요한 NVIDIA device만 namespace에 연결하고
`SDL_VIDEODRIVER=offscreen`과 NVIDIA EGL vendor를 선택한다. Xvfb/software GL은
사용하지 않는다. `node_evidence/gpu_offscreen_diagnostic.txt`는 namespace 안에서
RTX 4070 OpenGL context를 확인한 근거다. 카메라·USB·EtherCAT 장치는 연결하지 않는다.

001–012의 실패 기록은 보존한다. 006은 measured ready 지연, 007은 callback 간격,
008은 읽기 전용 registry lock, 009는 EE schema 버전, 010·011은 중복 core ghost가
rayrai 시야를 가린 경우다. `95edbd4`는 기존 calibration/actuation 검사를 유지한 채
EE schema 2/3을 받고, `1fa3066`은 Hub·Sim 양쪽의 중복 L/EE 표시를 숨기고 복원한다.
012는 B1 결과까지 도달한 뒤 harness의 종료 문구 비교에서 실패했고 visual-count
경고도 남았다. `c1947a2`는 topology 변경 시 world lock 안에서 rayrai wrapper를
갱신한다. 동일 새 test를 이전/수정 header로 빌드해 각각 실패/통과를 확인했고
최종 node013에는 해당 경고가 없었다.
최종 smoke 상태와 완료 판정은 [준비 문서](grasp_episode_preparation.md#최종-smoke)에서
한 곳으로 관리한다.

start utility `071f9b1`은 NONREAL 이동 전에 `GetCameraInfo`로 첫 프레임 준비를
기다리고, R/L 14개 관절을 명시한 CSV를 쓴다. single-L rest override는 기존
right-arm 템플릿 좌표로 직렬화하며 하드웨어 부호·기어비·구동 제한은 유지한다.
`7c05492`는 현재 → ready와 ready → 관측 자세를 별도 load로 나누고 각각 실제 위치·
양팔 속도·연속 안정 시간을 확인한다. 오른팔은 최초 측정 기준을 유지한다.
trajectory의 최초 preset 실행은 instance별 한 번이며, 같은 instance reset은 재생하지 않는다.
`8efc713`은 NONREAL native scene·renderer 초기화를 hold 상태로 마친 뒤 첫 검증 ACK를
허용한다. 첫 ACK 전 callback 간격 검사만 유예하며 REAL에는 처음부터 적용한다.

실제 node 검증에서는 `/proc/<pid>/maps`로 동적 plugin까지 해당 prefix/SDK에서
로드됐는지 확인하고, 준비 대기 → 새 A1 → 첫 accepted command → B1/result의
동일 episode 기록을 남긴다. 현재 build·8개 검사·82 ELF 통과만으로 이 결과를
대신하지 않는다. 실기 XY/world·최근 camera 설정의 실측 확인자료·현재 자세 → ready·
독립 grasp 성공도 별도 commissioning 범위다. mount 소스 합성은 최근 설정과
일치한다. `calibration_verified: false`는 저장된 flag이며 이번 시험에는 실측
receipt가 없다. 물리 보정이 수행되지 않았다는 판정으로 사용하지 않는다.

controller `b32a19d`는 최초 licensed SDK G0를 유지하고 A1 이후 별도 geometry
검사 시각으로 시작을 확인한다. REAL은 알려진 mesh의 local 6 DoF 관측성과 품질이
충분해야 하며, 기본 캔은 yaw 모호성으로 거부된다. NONREAL node smoke를 통과해도
이 물체나 현재 관측 plan이 실기용으로 검증되는 것은 아니다.
