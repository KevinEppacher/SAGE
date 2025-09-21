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