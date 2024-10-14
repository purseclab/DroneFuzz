from lxml import etree
import os


def include_xml(elem, base_path, processed_files=None):
    if processed_files is None:
        processed_files = set()

    for include in elem.xpath(".//include"):
        filename = include.text
        filepath = os.path.join(base_path, filename)
        print(f"Processing include: {filepath}")

        # Check if the file has already been processed
        if filepath in processed_files:
            print(f"Skipping already processed file: {filepath}")
            continue

        if os.path.exists(filepath):
            parser = etree.XMLParser(remove_blank_text=True)
            include_tree = etree.parse(filepath, parser)
            include_root = include_tree.getroot()

            # Mark the file as processed
            processed_files.add(filepath)

            # Recursively process includes in the included file
            include_xml(include_root, os.path.dirname(filepath), processed_files)

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
    msg_elements += root.xpath("//entry")
    if msg_elements:
        print("Messages found in XML:")
        for msg in msg_elements:
            msg_name = msg.get("name")
            if msg_name in filter:
                print(f"Message: {msg_name}")
                if msg.xpath(".//field"):
                    for field in msg.xpath(".//field"):
                        field_name = field.get("name")
                        field_type = field.get("type")
                        print(f"  Field: {field_name}, Type: {field_type}")
                elif msg.xpath(".//param"):
                    for param in msg.xpath(".//param"):
                        param_name = param.get("label")
                        if param_name is None:
                            continue
                        if param.get("minValue"):
                            param_min = param.get("minValue")
                        else:
                            param_min = "float"
                        if param.get("maxValue"):
                            param_max = param.get("maxValue")
                        else:
                            param_max = "float"
                        print(f"  Param: {param_name}, Range: {param_min}, {param_max}")
                print()
    else:
        print("No messages found in the XML file.")


def main():
    main_file = "xmls/ardupilotmega.xml"
    root = parse_xml_file(main_file)
    filter = ["MAV_CMD_DO_MOUNT_CONTROL"]
    print_messages(root, filter)


if __name__ == "__main__":
    main()
