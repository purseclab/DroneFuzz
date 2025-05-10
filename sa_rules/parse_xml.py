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


def get_enum(root, enum_name):
    """
    Find and return a list of enum values for the given enum name.
    Returns a dictionary with entry names as keys and their values and descriptions.
    """
    enum_elements = root.xpath(f"//enum[@name='{enum_name}']")
    if not enum_elements:
        print(f"Enum '{enum_name}' not found")
        return None

    enum_values = []
    for enum in enum_elements:
        for entry in enum.xpath(".//entry"):
            name = entry.get("name")
            value = entry.get("value")
            # description = entry.xpath("./description")
            # desc_text = description[0].text if description else "No description"
            # enum_values[name] = {"value": value}
            enum_values.append(value)

    return enum_values


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
                        field_enum = field.get("enum", None)
                        print(
                            f"  Field: {field_name}, Type: {field_type} Enum: {field_enum}"
                        )
                        if field_enum:
                            enum_values = get_enum(root, field_enum)
                            if enum_values:
                                print(f"Enum values for {field_enum}:")
                                print(f"  {enum_values}")
                elif msg.xpath(".//param"):
                    for param in msg.xpath(".//param"):
                        param_name = param.get("label")
                        if param_name is None:
                            continue
                        param_min = param.get("minValue", "float")
                        param_max = param.get("maxValue", "float")
                        param_inc = param.get("increment", None)
                        param_text = param.text
                        print(
                            f"  Param: {param_name} Desc: {param_text} Range: {param_min}, {param_max}, {param_inc}"
                        )
                print()
    else:
        print("No messages found in the XML file.")


def main():
    main_file = "xmls/ardupilotmega.xml"
    root = parse_xml_file(main_file)
    filter_list = ["MAV_CMD_DO_MOUNT_CONTROL","OBSTACLE_DISTANCE_3D"]
    print_messages(root, filter_list)

    # Example of using load_xml_messages
    # messages = load_xml_messages(main_file, filter_list)
    # if messages:
    #     print("\nLoaded message details:")
    #     for msg in messages:
    #         print(f"Message: {msg['msg_name']} (ID: {msg['msg_id']})")
    #         for field in msg["fields"]:
    #             print(f"  Field: {field['name']}, Type: {field['type']}")
    #             if "enum_values" in field:
    #                 print(f"    Enum values for {field['enum']}:")
    #                 print(f"      {field['enum_values']}")


def load_xml_messages(file_path: str, filter_list: list) -> list:
    """
    Load and parse XML messages, returning a list of message definitions.
    Each message includes its fields and any enum values for those fields.
    """
    root = parse_xml_file(file_path)
    messages = []

    msg_elements = root.xpath("//messages/message")
    msg_elements += root.xpath("//entry")

    for msg in msg_elements:
        msg_name = msg.get("name")
        if msg_name in filter_list:
            msg_id = msg.get("id")
            message_info = {"msg_name": msg_name, "msg_id": msg_id, "fields": []}

            if msg.xpath(".//field"):
                for field in msg.xpath(".//field"):
                    field_name = field.get("name")
                    field_type = field.get("type")
                    field_enum = field.get("enum")

                    field_info = {
                        "name": field_name,
                        "type": field_type,
                    }

                    if field_enum:
                        enum_values = get_enum(root, field_enum)
                        if enum_values:
                            field_info["enum"] = field_enum
                            field_info["enum_values"] = enum_values

                    message_info["fields"].append(field_info)

            messages.append(message_info)

    return messages


if __name__ == "__main__":
    main()
