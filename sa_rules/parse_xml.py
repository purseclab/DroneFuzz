import xml.etree.ElementTree as ET

# Parse the XML file
tree = ET.parse("ardupilotmega.xml")
root = tree.getroot()

# Find the 'enums' element
enums_element = root.find("enums")

if enums_element is not None:
    print("Enums found in ardupilotmega.xml:")
    for enum in enums_element.findall("enum"):
        enum_name = enum.get("name")
        print(f"Enum: {enum_name}")

        # Optionally, print entries of each enum
        for entry in enum.findall("entry"):
            entry_name = entry.get("name")
            entry_value = entry.get("value")
            print(f"  Entry: {entry_name}, Value: {entry_value}")
        print()
else:
    print("No enums found in the XML file.")

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
