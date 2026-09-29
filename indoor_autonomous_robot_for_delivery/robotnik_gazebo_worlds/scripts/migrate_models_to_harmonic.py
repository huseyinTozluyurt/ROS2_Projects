import os
import xml.etree.ElementTree as ET

def clean_sdf_model(sdf_path):
    print(f"Processing: {sdf_path}")
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    print(f"Original SDF version: {root.attrib.get('version', 'unknown')}")
    root.attrib['version'] = '1.7'
    removed_count = 0
    for collision in root.findall(".//collision"):
        for tag in ["surface"]:
            for elem in collision.findall(tag):
                collision.remove(elem)
                removed_count += 1
                print(f"Removed <{tag}> from collision in {sdf_path}")
    output_path = sdf_path.replace('.sdf', '.sdf')
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    print(f"Migrated {sdf_path} to Harmonic SDF: {output_path} ({removed_count} <surface> tags removed)")

models_dir = "models"
print(f"Scanning models directory: {models_dir}")
for model_name in os.listdir(models_dir):
    model_path = os.path.join(models_dir, model_name, "model.sdf")
    print(f"Checking: {model_path}")
    if os.path.exists(model_path):
        clean_sdf_model(model_path)
    else:
        print(f"File not found: {model_path}")