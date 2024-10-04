from lxml import etree
import os


def include_xml(elem, base_path):
    for include in elem.xpath(".//include"):
        filename = include.text
        filepath = os.path.join(base_path, filename)
        print(f"Processing include: {filepath}")
        if os.path.exists(filepath):
            parser = etree.XMLParser(remove_blank_text=True)
            include_tree = etree.parse(filepath, parser)
            include_root = include_tree.getroot()
            # Recursively process includes in the included file
            include_xml(include_root, os.path.dirname(filepath))
            # Replace the include element with the contents of the included file
            parent = include.getparent()
            index = parent.index(include)
            parent.remove(include)
            for child in reversed(list(include_root)):
                parent.insert(index, child)


def parse_xml_file(file_path):
    parser = etree.XMLParser(remove_blank_text=True)
    tree = etree.parse(file_path, parser)
    root = tree.getroot()
    include_xml(root, os.path.dirname(os.path.abspath(file_path)))
    return root


def print_messages(root, filter):
    msg_elements = root.xpath("//messages/message")
    if msg_elements:
        print("Messages found in XML:")
        for msg in msg_elements:
            msg_name = msg.get("name")
            if msg_name in filter:
                print(f"Message: {msg_name}")

                for field in msg.xpath(".//field"):
                    field_name = field.get("name")
                    field_type = field.get("type")
                    print(f"  Field: {field_name}, Type: {field_type}")
                print()
    else:
        print("No messages found in the XML file.")


def main():
    main_file = "xmls/ardupilotmega.xml"
    root = parse_xml_file(main_file)
    filter = ["OBSTACLE_DISTANCE_3D"]
    print_messages(root, filter)


if __name__ == "__main__":
    main()
