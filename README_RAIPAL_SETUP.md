# RAIPAL Setup Notes

This document records the RAIPAL setup flow used with this workspace.

## 1. Activate the RAISIN Python Environment

If the `venv raisin` shell helper is configured:

```bash
venv raisin
```

Otherwise activate the virtual environment directly:

```bash
source ~/.venvs/raisin_master/bin/activate
```

## 2. Create `src` and Plant the Seed

Create the source directory, clone the seed repository, then generate the seed package.

```bash
mkdir src && cd src
git clone git@github.com:jiseong1002016/_raisin_seed.git
python3 _raisin_seed/plant_seed.py {seed name}
cd ..
```

Replace `{seed name}` with the package or robot seed name you want to create.

## 3. Configure RAISIN

Create the local configuration file from the example:

```bash
cp configuration_setting_example.yaml configuration_setting.yaml
```

Then update the required fields in `configuration_setting.yaml`:

```yaml
# Select which package release channel RAISIN should use.
# - "user": stable release packages.
# - "devel": latest pre-release packages under development.
user_type: "devel"

# GitHub tokens used by raisin.py when it needs GitHub release access.
# Replace the placeholder with your own token if GitHub fallback or publishing is required.
gh_tokens:
  "raionrobotics": "ghp_your_token"
```

Keep personal access tokens local and do not commit them to the repository.

## 4. Install Compatible Release Packages

For the current RAIPAL source packages, install the compatible release package set from GitHub releases:

```bash
raisin install \
  raisin==0.2.1 \
  raisin_gui==0.1.1 \
  raisin_plugin==0.1.1 \
  raisin_robot_packages==0.1.1 \
  raisin_ros2_messages==0.1.1 \
  raisin_third_party_common==0.1.1 \
  raisin_third_party_gui==0.0.3 \
  raisin_third_party_robot==0.1.1 \
  --from-github
```

This keeps the release package set aligned with `raisin_gui==0.1.1` and avoids mixing it with newer packages such as `raisin==0.3.0` or `raisin_robot_packages==0.1.2`.

Check the local package versions after install:

```bash
raisin index local
```

## 5. Generate the RAIPAL URDF

Generate the URDF resources from the RAIPAL source package:

```bash
python3 ./src/raisin_raipal/resource/raipal_urdf/scripts/urdf-writer.py
```

## 6. Build

Build the release configuration:

```bash
raisin build --type release
```

If the executable artifacts should be copied into `install/bin`, run the install step as part of the build:

```bash
raisin build --type release --install
```

After that, executables such as `raisin_raipal_node` should be available under:

```bash
install/bin
```
