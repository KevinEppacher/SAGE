# SAGE
**SAGE (Semantic-Aware Guided Exploration)**

Repo uses **Isaac Lab** as a submodule.

https://github.com/matterport/habitat-matterport-3dresearch

## Setup
```bash
git submodule update --init --recursive --remote

rosdep update
rosdep install --rosdistro humble --from-paths src --ignore-src -r -y
```

### Install weights

1. SAGE/exploitation_ws/src/openfusion_ros/openfusion_ros/openfusion_ros/zoo/xdecoder_seem/checkpoints/seem_focall_v1.pt:
https://huggingface.co/xdecoder/SEEM/resolve/main/seem_focall_v1.pt

2. SAGE/detection_ws/src/seem_ros/seem_ros/seem_ros/seem_focall_v0.pt:
https://huggingface.co/xdecoder/SEEM/resolve/main/seem_focall_v0.pt

### X11 Forwarding for container
```bash
# on host
xhost +local:
```

### NVIDIA dGPU on laptops
```bash
sudo prime-select nvidia
sudo reboot
```

---

## Common GPU Laptop Crash (Vulkan / Isaac Sim/Lab)
**Symptoms**
- Early crash, e.g. `ERROR_DEVICE_LOST` or segfault in `libomni.kit.renderer.plugin.so`.
- `glxinfo -B` shows Intel, `vulkaninfo` defaults to Intel/llvmpipe.

**Detect**
```bash
glxinfo -B | egrep "OpenGL vendor|OpenGL renderer"
vulkaninfo  | egrep -i "GPU id|deviceName|selected gpu"
nvidia-smi
```

**Quick test**
```bash
# Run headless (minimizes render problems)
./isaaclab.sh --headless --no-window -p scripts/tutorials/00_sim/create_empty.py
```

**Fix (per session, keep iGPU default)**
```bash
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
# optional:
export __NV_PRIME_RENDER_OFFLOAD_PROVIDER=NVIDIA-G0

vulkaninfo | egrep -i "deviceName|selected gpu"
glxinfo -B | egrep "OpenGL vendor|OpenGL renderer"
```

**Fix (persistent)**
```bash
sudo prime-select nvidia
sudo reboot
```

**Recommendation:** Prefer **headless** mode (with WebRTC client for visualization). Use GUI only if strictly required.

---

## Recommended Headless Workflow
**Inside the container**
```bash
# start Isaac Sim headless
./_isaacsim/runheadless.sh
```

**On the laptop**
If AppArmor blocks unprivileged user namespaces:
```bash
sudo sysctl -w kernel.apparmor_restrict_unprivileged_userns=0
```
Then start the WebRTC client:
```bash
./isaacsim-webrtc-streaming-client-1.1.4-linux-x64.AppImage
```

**Note on this error**
```text
FATAL ... SUID sandbox helper binary ... chrome-sandbox ... needs to be owned by root and mode 4755
```
This comes from Chromium sandboxing inside the AppImage. Workarounds (setuid bits) are unsafe. **Better:** run Isaac headless and use the official WebRTC client.


# IsaacSim

docker pull nvcr.io/nvidia/isaac-sim:5.0.0

```bash
docker run --name isaac-sim --entrypoint bash -it --runtime=nvidia --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
    -e "PRIVACY_CONSENT=Y" \
    -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
    -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
    -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
    -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
    -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
    -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
    -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
    -v ~/docker/isaac-sim/documents:/root/Documents:rw \
    -v ../../SAGE/isaac_sim:/isaac-sim/ros2_ws:rw \
    nvcr.io/nvidia/isaac-sim:5.0.0
```