
```bash
docker exec -it ros2_lavis_container bash

docker exec -it ros2_detection_container bash

docker exec -it ros2_explorer_container bash

docker exec -it ros2_exploitation_container bash

docker exec -it isaac_sim_container bash

```


```bash
docker exec -it ros2_exploitation_container bash -c "
cd /app
source install/setup.bash
ros2 run gui gui
"
```

```bash
docker exec -it ros2_explorer_container bash -c "
cd /app
source install/setup.bash
ros2 launch sage_commander explorer_ws.launch.py
"
```

```bash
docker exec -it ros2_detection_container bash -c "
cd /app
source install/setup.bash
ros2 launch sage_commander detection_ws.launch.py
"
```

# Launch SAGE Containers (Local)
```bash
cd /home/kevin/Documents/4_Semester/SAGE/docker && docker compose up -d detection_container explorer_container lavis_container isaac_sim
```

# Launch Groot2 (Local)
```bash
cd /home/kevin/Groot2/bin && ./groot2
```

# Launch Streaming Client (Local)
```bash
cd /home/kevin/Documents/IsaacOmniverse && ./isaacsim-webrtc-streaming-client-1.0.6-linux-x64.AppImage 
```