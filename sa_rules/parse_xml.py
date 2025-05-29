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


def print_messages(root, filter_list):
    filter_set = set(filter_list)
    enum_cache = {}

    msg_elements = root.findall(".//messages/message") + root.findall(".//entry")
    if not msg_elements:
        print("No messages found in the XML file.")
        return

    output = ["Messages found in XML:"]

    for msg in msg_elements:
        name = msg.get("name")
        if name not in filter_set:
            continue

        output.append(f"Message: {name}")

        # collect only <field> children that come before the <extensions> tag
        fields = []
        for child in msg:
            if child.tag == "extensions":
                break
            if child.tag == "field":
                fields.append(child)

        if fields:
            for fld in fields:
                fld_name = fld.get("name")
                fld_type = fld.get("type")
                fld_enum = fld.get("enum")
                output.append(
                    f"  Field: {fld_name}, Type: {fld_type}, Enum: {fld_enum}"
                )

                if fld_enum:
                    if fld_enum not in enum_cache:
                        enum_cache[fld_enum] = get_enum(root, fld_enum) or []
                    vals = enum_cache[fld_enum]
                    if vals:
                        output.append(f"Enum values for {fld_enum}:")
                        output.append(f"  {vals}")
        else:
            # no pre-extension fields; fall back to params
            params = msg.findall(".//param")
            for prm in params:
                label = prm.get("label")
                if not label:
                    continue
                # Check if there's a enum attribute
                if prm.get("enum"):
                    enum_name = prm.get("enum")
                    # Get the values for the enum
                    enum_values = get_enum(root, enum_name)
                    mn = min(enum_values) if enum_values else "N/A"
                    mx = max(enum_values) if enum_values else "N/A"
                    inc = prm.get("increment", "N/A")
                    desc = (prm.text or "").strip(".")
                    desc += " and is ENUM of type " + enum_name
                else:
                    mn = prm.get("minValue", "float")
                    mx = prm.get("maxValue", "float")
                    inc = prm.get("increment")
                    desc = (prm.text or "").strip()
                output.append(f"  Param: {label} Desc: {desc} Range: {mn}, {mx}, {inc}")

        output.append("")

    print("\n".join(output))


def main():
    main_file = "../xmls/common.xml"
    root = parse_xml_file(main_file)
    filter_list = ["MAV_CMD_DO_MOUNT_CONTROL"]
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
