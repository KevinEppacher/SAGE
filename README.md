# SAGE
SAGE (Semantic-Aware Guided Exploration)

https://github.com/matterport/habitat-matterport-3dresearch

```bash
git submodule update --init --recursive --remote
```

```bash
rosdep update && rosdep install --rosdistro humble --from-paths src --ignore-src -r -y
```

For container visualization, type at host this:
```bash
xhost +local:
```

sudo prime-select nvidia
sudo reboot

### For launching Streaming Client on laptop
```bash
sudo sysctl -w kernel.apparmor_restrict_unprivileged_userns=0
```
Error Line
```bash
kevin@ubuntu:~/Documents/IsaacOmniverse$ ./isaacsim-webrtc-streaming-client-1.1.4-linux-x64.AppImage 
[40222:0923/001051.412053:FATAL:sandbox/linux/suid/client/setuid_sandbox_host.cc:169] The SUID sandbox helper binary was found, but is not configured correctly. Rather than run without sandboxing I'm aborting now. You need to make sure that /tmp/.mount_isaacsFWtRDq/chrome-sandbox is owned by root and has mode 4755.
Trace/breakpoint trap (core dumped)
```
