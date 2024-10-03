import xml.etree.ElementTree as ET
import xml.etree.ElementInclude as EI
import os


def include_xml(elem, base_path):
    for include in elem.findall("include"):
        filename = include.text
        filepath = os.path.join(base_path, filename)
        print(f"Processing include: {filepath}")
        if os.path.exists(filepath):
            tree = ET.parse(filepath)
            include_root = tree.getroot()
            # Recursively process includes in the included file
            include_xml(include_root, os.path.dirname(filepath))
            # Replace the include element with the contents of the included file
            index = list(elem).index(include)
            # elem.remove(include)
            print("Removing include element {}".format(include))
            for child in reversed(list(include_root)):
                elem.insert(index, child)


# Parse the XML file
main_file = "xmls/ardupilotmega.xml"
tree = ET.parse(main_file)
root = tree.getroot()
include_xml(root, os.path.dirname(os.path.abspath(main_file)))
msg_elements = root.find("messages")
if msg_elements is not None:
    print("MSG found in ardupilotmega.xml:")
    for msg in msg_elements.findall("message"):
        msg_name = msg.get("name")
        print(f"msg: {msg_name}")

        # Optionally, print entries of each enum
        for entry in msg.findall("field"):
            entry_name = entry.get("name")
            entry_value = entry.get("type")
            print(f"  Field: {entry_name}, Type: {entry_value}")
        print()
else:
    print("No msgs found in the XML file.")
