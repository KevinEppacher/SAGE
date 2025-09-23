# TODO: Instance-Aware PointCloud Clustering

Goal: prevent nearby detections with similar scores from merging into one cluster by carrying **instance IDs** from the detector into 3D and clustering **per instance**.

---

## 1) Python producer: add instance field to PointCloud2

**Inputs**
- `/yoloe/score_mask_raw` (Image, 32FC1, score in [0,1])
- `/yoloe/instance_id_mask` (Image, 16UC1, 0=background, 1..N per detection)
- `/depth` (Image, 32FC1 [m] or 16UC1 [mm])
- `/camera_info` (CameraInfo)

**Output**
- `/yoloe/score_cloud` (PointCloud2) with fields: `x,y,z,intensity,instance` where:
  - `intensity` = score (float32)
  - `instance`  = per-detection id (uint16)

**Pseudo**
```
for each synchronized frame:
    get fx, fy, cx, cy from CameraInfo
    resize score/instance to depth size if needed

    mask = finite(depth) & (depth_min < depth < depth_max) & (score > thr) & (instance > 0)
    for each (u,v) where mask:
        z = depth[v,u]
        x = (u - cx) / fx * z
        y = (v - cy) / fy * z
        push point (x, y, z, intensity=score[v,u], instance=instance[v,u])
publish PointCloud2(fields=[x,y,z,intensity,instance])
```

---

## 2) C++ consumer: split by instance id before clustering

**Transform**
- Transform incoming cloud to `target_frame` via TF2 (e.g., `odom`).

**Split**
```
for each point in cloud_tf:
    read x,y,z,intensity,instance
    if instance == 0: continue
    append point to by_instance[instance]
cache by_instance for timerCallback
```

---

## 3) Cluster per instance and publish

```
for (instance_id, cloud) in by_instance:
    voxel_filter(cloud, leaf=voxel_leaf_size)
    euclidean_cluster(cloud, tol=cluster_tolerance, min=min_cluster_size, max=max_cluster_size)
    for each cluster:
        compute bbox + centroid
        score = percentile(intensity, 75%) * log(cluster_size + 1)
        publish Marker(s) and GraphNode with centroid and score
```

**Marker/Graph frames**
- Use the same frame as `target_frame` for MarkerArray and GraphNodeArray.

---

## 4) Notes

- `instance` separates detector outputs; intensity remains the score.
- Complexity ~ linear in points + per-instance clustering.
- Optional: publish per-instance clouds on `/yoloe/score_cloud/instance_<k>` instead of one labeled cloud.
