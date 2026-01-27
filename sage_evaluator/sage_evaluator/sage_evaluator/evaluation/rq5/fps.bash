ubuntu@kevin:/app$ ros2 topic hz /fused/exploration_graph_nodes/graph_nodes
WARNING: topic [/fused/exploration_graph_nodes/graph_nodes] does not appear to be published yet
average rate: 1.571
        min: 0.593s max: 0.685s std dev: 0.03776s window: 3
average rate: 1.582
        min: 0.482s max: 0.770s std dev: 0.09602s window: 5
^Cubuntu@kevin:/app$ ros2 topic hz /fused/detection_graph_nodes/graph_nodes 
average rate: 1.740
        min: 0.501s max: 0.622s std dev: 0.05274s window: 3
average rate: 1.660
        min: 0.501s max: 0.680s std dev: 0.05783s window: 5
^Cubuntu@kevin:/app$ ros2 topic hz /yoloe/
/yoloe/detections                     /yoloe/overlay                        /yoloe/score_mask_raw                 /yoloe/yolo_wrapper/transition_event  
/yoloe/instance_id_mask               /yoloe/score_mask_debug               /yoloe/semantic_pointcloud_xyzi       
ubuntu@kevin:/app$ ros2 topic hz /yoloe/overlay 
WARNING: topic [/yoloe/overlay] does not appear to be published yet
average rate: 6.043
        min: 0.130s max: 0.342s std dev: 0.06729s window: 8
average rate: 6.408
        min: 0.077s max: 0.342s std dev: 0.06096s window: 16
^Cubuntu@kevin:/app$ ros2 topic hz /value_map/
/value_map/camera_pose                 /value_map/cosine_similarity           /value_map/value_map                   /value_map/value_map/transition_event  /value_map/value_map_raw               
ubuntu@kevin:/app$ ros2 topic hz /value_map/cosine_similarity 
WARNING: topic [/value_map/cosine_similarity] does not appear to be published yet
average rate: 10.016
        min: 0.097s max: 0.103s std dev: 0.00173s window: 11
average rate: 10.009
        min: 0.097s max: 0.103s std dev: 0.00136s window: 22
^Cubuntu@kevin:/app$ ros2 topic hz /openfusion_ros/
/openfusion_ros/camera_pose              /openfusion_ros/panoptic_pointcloud_rgb  /openfusion_ros/query_pointcloud_xyzi    /openfusion_ros/slam_pointcloud          
/openfusion_ros/camera_pose/debug        /openfusion_ros/pose_array               /openfusion_ros/semantic_pointcloud_rgb  
ubuntu@kevin:/app$ ros2 topic hz /openfusion_ros/query_pointcloud_xyzi 
average rate: 0.355
        min: 2.815s max: 2.816s std dev: 0.00049s window: 2
average rate: 0.279
        min: 2.815s max: 5.136s std dev: 1.09396s window: 3
^Cubuntu@kevin:/app$ ros2 topic hz /openfusion_ros/
/openfusion_ros/camera_pose              /openfusion_ros/panoptic_pointcloud_rgb  /openfusion_ros/query_pointcloud_xyzi    /openfusion_ros/slam_pointcloud          
/openfusion_ros/camera_pose/debug        /openfusion_ros/pose_array               /openfusion_ros/semantic_pointcloud_rgb  
ubuntu@kevin:/app$ ros2 topic hz /openfusion_ros/slam_pointcloud 
^Cubuntu@kevin:/app$ ros2 topic hz /openfusion_ros/slam_pointcloud 
WARNING: topic [/openfusion_ros/slam_pointcloud] does not appear to be published yet
^[[A^Cubuntu@kevin:/app$ ros2 topic hz /openfusion_ros/slam_pointcloud 
^Cubuntu@kevin:/app$ ros2 topic hz /exploitation_
/exploitation_graph_nodes/bounding_boxes      /exploitation_graph_nodes/graph_nodes         /exploitation_graph_nodes/graph_nodes/marker  /exploitation_relevance_map/relevance_map     
ubuntu@kevin:/app$ ros2 topic hz /exploitation_graph_nodes/graph_nodes
average rate: 10.036
        min: 0.096s max: 0.102s std dev: 0.00156s window: 12
average rate: 10.033
        min: 0.094s max: 0.104s std dev: 0.00210s window: 23
^Cubuntu@kevin:/app$ ros2 topic hz /exploration_graph_nodes/graph_nodes
average rate: 6.641
        min: 0.136s max: 0.184s std dev: 0.01608s window: 8
average rate: 6.058
        min: 0.136s max: 0.203s std dev: 0.02252s window: 14