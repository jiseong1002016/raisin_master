# railab에서 local AnyGrasp 호출

railab의 SDK 폴더는 RGBD에서 얻은 XYZ/RGB를 SSH로 local에 보내고 grasp 후보를
받는다. 기존 라이선스·SDK·checkpoint는 local에 둔다. 카메라 촬영, 좌표 변환,
후보의 IK·경로 검사, joy와 에피소드 제어는 railab의 기존 경로를 사용한다.

저장 RGBD·backend 준비에 이어 새 통합 설치의 **실제 NONREAL node + SSH SDK +
A1/B1 episode**도 통과했다. 최신 결과·재현 경로와 실기 미검증 범위는
[후속 준비 기록](grasp_episode_continue.md)을 따른다. 실제 로봇·D405 장치는 열지 않았다.

## 설치된 폴더

- railab: `/home/raipal/workspace/jiseong/isolation/anygrasp_remote_20261024/sdk`
- local worker: `/home/jiseong/raisim_ws/isolation/anygrasp_remote_20261024/sdk_local`
- 최신 통합 설치·bundle: railab `isolation/episode_continue_20261024/master/install`,
  두 기기의 같은 작업 root 아래 `inference_bundle_002`.
- 최초 SSH 준비의 bundle·설정: `isolation/anygrasp_remote_20261024/inference_bundle`,
  같은 작업 root의 `nonreal_prepared/episode.json` (역사적 준비 검사).

SDK 폴더에는 `check`, `anygrasp.py`, `anygrasp_ssh.py`, `anygrasp.json`, `ssh_config`,
`known_hosts`, 전용 `identity`와 공개키가 있다. client 쪽 native SDK·CUDA·라이선스는
필요하지 않다. Python+NumPy, SSH, local로의 네트워크 연결은 필요하다.

local은 기존 SSH 서비스를 사용하며 새 상시 서버를 설치하지 않았다. 연결할 때
worker와 SDK를 준비하고, 호출자가 닫거나 연결이 끊기면 종료한다. 전용 SSH 키는
railab 주소에서 고정된 worker 명령만 실행할 수 있으며 shell·PTY·forwarding은
허용하지 않는다. 서버 공개키는 `known_hosts`에 고정했다. 기존 local 인증 항목은
보존하고 제한된 키 한 개를 추가했다. private key와 라이선스는 Git에 넣지 않았다.

client 폴더의 설정·인증 파일은 폴더를 기준으로 찾는다. 같은 railab 안에서 폴더를
옮길 수 있고, 실제로 다른 위치·작업 디렉터리에서 warmup을 통과했다. 이동 후
에피소드의 `runtime_config`/`ANYGRASP_CONFIG` 경로도 새 위치로 지정해야 한다.
local worker·SDK 경로와 local 서버 주소는 `ssh_config` 및 제한된 서버 인증 명령에
묶여 있으므로 local 배치를 바꾸면 함께 갱신한다.

## 확인 명령

railab에서 실행한다. 첫 명령은 실제 local SDK를 준비하며, 두 번째는 저장된
시뮬레이터 관측을 추론한다. 카메라나 로봇 장치는 열지 않는다.

```bash
base=/home/raipal/workspace/jiseong/isolation/anygrasp_remote_20261024
"$base/sdk/check"
"$base/sdk/check" --input "$base/evidence/captured_rgbd.npz"
```

에피소드 launcher에는 다음 인자를 추가한다. 기존 source·plant·observation plan
선택은 유지한다. `--prepare-only`는 node 없이 설정을 생성한다.

```bash
--anygrasp-config /home/raipal/workspace/jiseong/isolation/anygrasp_remote_20261024/sdk/anygrasp.json
```

같은 설정을 node launcher와 direct simulator에서 환경변수로 지정할 수도 있다.
시뮬레이터의 기존 `--vision-config`는 카메라 extrinsic을 제공하므로 계속 필요하다.

```bash
export ANYGRASP_CONFIG=/home/raipal/workspace/jiseong/isolation/anygrasp_remote_20261024/sdk/anygrasp.json
```

장치 없는 전체 backend 검사:

```bash
base=/home/raipal/workspace/jiseong/isolation/anygrasp_remote_20261024
old=/home/raipal/workspace/jiseong/isolation/episode_completion_20261023
"$base/policy_runtime/python" -m raipal_grasp.runtime \
  --config "$base/nonreal_prepared/episode.json" \
  --b0-library "$old/master/install/lib/libraisin_grasp_b0.so" --check
```

위 명령은 최초 SSH 준비의 backend 검사다. 최신 전체 node 검사는 C 변경을 통합한
새 native prefix와 bundle을 사용한다. 기존 bundle003과 운영 prefix는 보존했다.
일반 운영 설치가 이 설정을 자동으로 선택하는 것은 아니므로 사용할 SDK config와
호환되는 source·prefix·bundle을 명시한다.

## 입력·오류·시간

real과 nonreal 모두 기존 공유 `grasp_integration.anygrasp.prepare/infer`를 사용한다.
`camera.anygrasp.runtime_config`가 SDK 폴더의 설정을 불러오고, episode에 적힌
gripper·workspace·시간 설정이 있으면 우선한다. SDK 준비는 촬영 전에 끝내며,
현재 warmup 제한은 90초, 촬영부터 결과 사용까지의 제한은 기존 2초다.

SSH는 한 번 준비한 worker를 재사용한다. 64–12,000개의 유한한 XYZ와 정규화 RGB를
크기가 제한된 float32 NPZ로 보낸다. 서버는 압축·배열 header·shape·dtype·실제 길이를
확인하고, 결과는 요청 번호·입력 hash·checkpoint hash와 함께 돌려준다.
연결 오류·시간 초과·다른 입력/모델의 결과는 호출을 종료하며 대체 pose를 만들지 않는다.

프레임 시각을 다른 기기의 시계로 바꾸지 않는다. 기존 호출자에서 촬영 시각부터
직렬화·전송·추론·반환까지 경과한 시간을 검사한다. A1 후의 별도 geometry 재검증과
고정된 최초 G0도 유지한다. 100 Hz 제어 요청은 이 SSH 경로를 거치지 않는다.

## 최초 저장 RGBD 검사와 범위

- 공유 AnyGrasp 회귀 검사 12개, launcher 설정 검사 7개 통과.
- 새 bundle 96개 파일의 hash를 railab에서 확인했다. source Git 상태는 clean이다.
- 새 bundle의 실제 공유 API로 저장된 rayrai 관측 6,994점을 세 번 추론했다.
  warmup 4.86초, 왕복 0.612 / 0.059 / 0.062초였다. 모두 기존 2초 제한 안이었다.
- 새 launcher의 `--anygrasp-config --prepare-only` 결과로 backend가 `loaded`,
  `hardware_opened: false`를 반환했다. 연결 불가 설정에서는 exit 2와
  `setup_required`를 반환했다. 해당 시도에 pose fallback은 없다.
- SDK 폴더 위치와 호출 작업 디렉터리를 바꾼 warmup 통과. SSH·SDK worker 종료 확인.

이 수치는 저장된 한 관측의 연결·실제 추론 확인이다. 후속 전체 node 검사는 새
rayrai 관측 8,057점, SSH 왕복 0.310초, A1 전 대기·실제 제어 27 tick·B1 종료를
통과했다. [최신 검증 JSON](grasp_episode_continue_validation.json)을 따른다.
어느 검사도 다른 장면의 지연, 실제 D405 취득·로봇 grasp 성공을 보장하지 않는다.

정확한 source pin과 검증 파일 hash는 [검증 기록](anygrasp_ssh_validation.json)에 있다.
주요 실행 근거는 두 기기의 작업 경로 `evidence/`에 보존했다.
