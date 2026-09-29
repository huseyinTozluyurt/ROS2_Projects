import xml.etree.ElementTree as ET

INPUT_FILE = "electrical_station.world"
OUTPUT_FILE = "electrical_station_harmonic.world"

def remove_surface_blocks(element):
    for collision in element.findall(".//collision"):
        for surface in collision.findall("surface"):
            collision.remove(surface)

def migrate_world(input_path, output_path):
    tree = ET.parse(input_path)
    root = tree.getroot()

    # Update SDF version
    root.attrib['version'] = '1.7'

    # Remove all <surface> blocks from <collision>
    remove_surface_blocks(root)

    # Save the migrated world
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    print(f"Migrated world saved to {output_path}")

if __name__ == "__main__":
    migrate_world(INPUT_FILE, OUTPUT_FILE)