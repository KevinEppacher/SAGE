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

**Container run (important)**
```bash
docker run --gpus all --runtime=nvidia   -e OMNI_KIT_ALLOW_ROOT=1   -e VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json   -e __NV_PRIME_RENDER_OFFLOAD=1   -e __GLX_VENDOR_LIBRARY_NAME=nvidia   --shm-size=2g --ulimit memlock=-1 --ulimit stack=67108864   ...
```

**Stability flags for Isaac**
```bash
./isaaclab.sh   --/renderer/multiGpu/enabled=false   --/renderer/aftermath/enabled=false   --/rtx/gi/enable=false   --/app/window/drawMouse=false
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

---

## USD/Stage stability notes
- Avoid self-references or cycles in payloads/references.
- Set a valid `defaultPrim` (e.g. `"World"`).
- Load the scene first, then enable ROS2-Bridge, and add cameras step by step.
