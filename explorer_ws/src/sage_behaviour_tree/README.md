# SAGE - Semantic Aware Guided Exploration:

```python
for all searching_objects_list:
    rotate_robot(360)

    found_object = false
    loop until object found:
        for fusion_node in fusionaded_graph_nodes:
            if(fusion_node.score > threshold)
                navigate_to(fusion_node.pos)
                take_picture
                found_object = true
                return
        if found_object = false:
            node = get_likiest_graph_node()
            if node.is_empty():
                rotate_robot(360)
            navigate_to(node.pos) 
```