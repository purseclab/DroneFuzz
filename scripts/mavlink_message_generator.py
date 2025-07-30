import os
import re
import random
import argparse
import time
from lxml import etree
from pymavlink import mavutil


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
            value = entry.get("value")
            enum_values.append(value)

    return enum_values


def include_xml(elem, base_path, processed_files=None):
    if processed_files is None:
        processed_files = set()

    for include in elem.xpath(".//include"):
        filename = include.text
        filepath = os.path.join(base_path, filename)
        if filepath in processed_files:
            continue

        if os.path.exists(filepath):
            parser = etree.XMLParser(remove_blank_text=True)
            include_tree = etree.parse(filepath, parser)
            include_root = include_tree.getroot()

            processed_files.add(filepath)
            include_xml(include_root, os.path.dirname(filepath), processed_files)

            parent = include.getparent()
            index = parent.index(include)
            parent.remove(include)
            for child in reversed(list(include_root)):
                parent.insert(index, child)


def load_xml_messages(file_path: str, filter_list: list) -> list:
    """
    Parse the XML at file_path, include any referenced XML via include_xml(),
    and return a list of messages (matching filter_list) as dicts.
    """
    parser = etree.XMLParser(remove_blank_text=True)
    tree = etree.parse(file_path, parser)
    root = tree.getroot()
    include_xml(root, os.path.dirname(os.path.abspath(file_path)))

    filter_set = set(filter_list)
    enum_cache = {}
    messages = []

    msg_elements = root.findall(".//messages/message") + root.findall(".//entry")

    for msg in msg_elements:
        msg_name = msg.get("name")
        if msg_name not in filter_set:
            continue

        msg_id = msg.get("id") or msg.get("value")
        fields = []
        for child in msg:
            # if child.tag == "extensions":
            #     break
            if child.tag == "field" or child.tag == "extensions":
                name = child.get("name")
                typ = child.get("type")
                desc = (child.text or "").strip()
                units = child.get("units", "")
                enum_name = child.get("enum")
                entry = {"name": name, "type": typ, "desc": desc, "units": units}

                if enum_name:
                    if enum_name not in enum_cache:
                        enum_cache[enum_name] = get_enum(root, enum_name) or []
                    if enum_cache[enum_name]:
                        entry["enum_vals"] = enum_cache[enum_name]

                fields.append(entry)
        messages.append({"msg_id": msg_id, "msg_name": msg_name, "fields": fields})

    return messages


def generate_field_value(
    field_type, field_desc=None, field_units=None, field_range=[None, None, None]
):
    """
    Generate a random value for a field based on its type.
    """
    if "[" in field_type:
        generated_value = []
        match = re.search(r"\[(\d+)\]", field_type)
        if match:
            field_size = int(match.group(1))
        else:
            raise ValueError(
                f"field_type '{field_type}' does not contain a size in brackets"
            )

        base_type = field_type.split("[")[0]
        for _ in range(field_size):
            generated_value.append(
                generate_field_value(base_type, field_desc, field_units, field_range)
            )

        if base_type == "char":
            return bytes(generated_value)
        return generated_value

    if field_range[0] is not None and field_range[1] is not None:
        if field_type in ["float", "double"]:
            return random.uniform(field_range[0], field_range[1])
        elif field_range[2] is not None:
            return random.randrange(field_range[0], field_range[1], field_range[2])
        else:
            return random.randint(field_range[0], field_range[1])
    else:
        if field_type.startswith("uint8"):
            return random.randint(0, 255)
        elif field_type.startswith("uint16"):
            return random.randint(0, 65535)
        elif field_type.startswith("uint32"):
            return random.randint(0, 4294967295)
        elif field_type.startswith("int8"):
            return random.randint(-128, 127)
        elif field_type.startswith("int16"):
            return random.randint(-32768, 32767)
        elif field_type.startswith("int32"):
            return random.randint(-2147483648, 2147483647)
        elif field_type.startswith("float"):
            return random.uniform(-10, 10)
        elif field_type.startswith("char"):
            return random.randint(0, 255)
        else:
            return 0


def generate_mavlink_message(message_name, xml_file):
    """
    Generates a MAVLink message with random data.
    """
    message_specs = load_xml_messages(xml_file, [message_name])
    if not message_specs:
        raise ValueError(f"Message '{message_name}' not found in {xml_file}")

    spec = message_specs[0]
    field_values = []
    for field in spec["fields"]:
        value = generate_field_value(field["type"])
        field_values.append(value)

    # Construct the message
    mavlink_msg_name = f"MAVLink_{message_name.lower()}_message"
    try:
        msg_constructor = getattr(mavutil.mavlink, mavlink_msg_name)
        message = msg_constructor(*field_values)
        return message
    except AttributeError:
        raise ValueError(
            f"Could not find a constructor for '{message_name}' in pymavlink."
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generate MAVLink messages for testing."
    )
    parser.add_argument(
        "--xml",
        required=True,
        help="Path to the MAVLink XML definition file (e.g., ardupilotmega.xml).",
    )
    parser.add_argument(
        "--message-name",
        required=True,
        help="The name of the MAVLink message to generate.",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=0,
        help="Number of messages to generate (0 for infinite).",
    )
    parser.add_argument(
        "--connect",
        help="MAVLink connection string (e.g., tcp:localhost:5760, udp:localhost:14550, /dev/ttyUSB0).",
    )
    parser.add_argument(
        "--baud", type=int, default=115200, help="Baud rate for serial connections."
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=1.0,
        help="Delay in seconds between sending messages.",
    )
    args = parser.parse_args()

    master = None
    if args.connect:
        try:
            master = mavutil.mavlink_connection(args.connect, baud=args.baud)
            master.wait_heartbeat()
            print(f"Connected to {args.connect} and got heartbeat.")
        except Exception as e:
            print(f"Failed to connect to {args.connect}: {e}")
            exit(1)

    message_iterator = 0
    try:
        while True:
            # If a specific count is set, break the loop when it's reached
            if args.count != 0 and message_iterator >= args.count:
                break

            loop_start_time = time.time()

            count_str = "infinity" if args.count == 0 else str(args.count)
            print(f"--- Generating message {message_iterator + 1}/{count_str} ---")

            generated_message = generate_mavlink_message(args.message_name, args.xml)
            if master:
                master.mav.send(generated_message)
                print(f"Sent: {generated_message}")
            else:
                print(generated_message)

            message_iterator += 1

            # If this is the last message in a fixed-count run, don't sleep
            if args.count != 0 and message_iterator >= args.count:
                continue

            # Precise delay logic
            elapsed_time = time.time() - loop_start_time
            sleep_duration = args.delay - elapsed_time
            if sleep_duration > 0:
                time.sleep(sleep_duration)
            else:
                print(
                    f"Warning: Loop execution ({elapsed_time:.4f}s) is longer than the desired delay ({args.delay}s)."
                )

    except (ValueError, FileNotFoundError) as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("\nStopping message generation.")
    finally:
        if master:
            master.close()
            print("Connection closed.")
