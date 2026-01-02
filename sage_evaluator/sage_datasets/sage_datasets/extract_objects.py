import json

# Read from your data file
input_file = "/app/src/sage_evaluator/sage_datasets/matterport_isaac/0891-cvZr5TUy5C5/assets/cvZr5TUy5C5.semantic.txt"
output_file = "/app/src/sage_evaluator/sage_datasets/matterport_isaac/0891-cvZr5TUy5C5/assets/classes.json"

unique_objects = set()

# Read each line and extract the third field (object name)
with open(input_file, "r", encoding="utf-8") as f:
    for line in f:
        parts = line.strip().split(",")
        if len(parts) >= 3:
            name = parts[2].strip().strip('"')
            unique_objects.add(name)

# Convert to a sorted list for readability
unique_objects = sorted(unique_objects)

# Save to JSON file
with open(output_file, "w", encoding="utf-8") as f:
    json.dump(unique_objects, f, indent=4, ensure_ascii=False)

print(f"Saved {len(unique_objects)} unique objects to {output_file}")
