# SAGE
SAGE (Semantic-Aware Guided Exploration)

https://github.com/matterport/habitat-matterport-3dresearch

git submodule update --init --recursive --remote

rosdep update && rosdep install --rosdistro humble --from-paths src --ignore-src -r -y

For container visualization, type at host this:
```bash
xhost +local:
```