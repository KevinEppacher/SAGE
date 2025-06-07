# VLFM Psuedocode
initialize_empty_value_map()
initialize_confidence_map()
load_pretrained_vision_language_model()  # e.g. BLIP-2

for each_timestep:
    # Step 1: Daten erfassen
    rgb_image = get_current_rgb_image()
    depth_image = get_current_depth_image()
    pose = get_current_robot_pose()

    # Step 2: Sichtfeldmaske erzeugen
    fov_mask = generate_topdown_cone_mask(pose, camera_fov, max_range)

    # Step 3: Hindernisse ausmaskeiren
    fov_mask = exclude_obstacle_regions(fov_mask, depth_image)

    # Step 4: Semantischen Wert berechnen
    similarity_score = compute_cosine_similarity(rgb_image, text_prompt)

    # Step 5: Confidence-Karte erzeugen (cos²-gewichtete Verteilung über FOV)
    confidence = compute_confidence_map(pose, fov_mask, camera_fov)

    # Step 6: Werte in top-down map schreiben (Transformation Pose → Map Koordinaten)
    for pixel in fov_mask:
        prev_value = value_map[pixel]
        prev_conf = confidence_map[pixel]
        new_value = similarity_score
        new_conf = confidence[pixel]

        # Step 7: Fusionsstrategie (gewichtetes Mittel)
        fused_value = (prev_value * prev_conf + new_value * new_conf) / (prev_conf + new_conf)
        fused_conf = (prev_conf**2 + new_conf**2) / (prev_conf + new_conf)

        value_map[pixel] = fused_value
        confidence_map[pixel] = fused_conf

    # Optional: publish occupancy grid / point cloud / visualization
