
```bash
docker exec -it ros2_exploitation_container bash

docker exec -it ros2_detection_container bash

docker exec -it ros2_explorer_container bash

docker exec -it ros2_lavis_container bash
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