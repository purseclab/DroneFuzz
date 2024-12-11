#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# SPDX-License-Identifier: BSD-3-Clause or GPL-3.0-or-later
# SPDX-FileCopyrightText: (C) 2024 silipwn (Ashwin)
# Finis coronat opus; Run at this code at your own peril ~ silipwn;
# File                   : dumb_fuzzer.py
# Author                 : silipwn <contact at as-hw.in>
# Description            : This file is responsible for generating the random inputs
# Date                   : 2024-12-11T07:59:44-0500
# Last Modified          : 2024-12-11T07:59:47-0500
# Last Modified By       : silipwn <contact at as-hw.in>

from pymavlink import mavutil
import argparse
import logging
import json
from lxml import etree
import random
import os
import string
import time
import numpy

# Setup logger
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
format = logging.Formatter(
    "%(asctime)s - %(levelname)s %(filename)s:%(lineno)d- %(message)s"
)
logger.addHandler(logging.StreamHandler())
logger.handlers[0].setFormatter(format)


def current_milli_time(start_time) -> int:
    return int(round(time.time() * 1000) - start_time)


# Custom loader function to include XML files
def include_xml(elem, base_path, processed_files=None):
    if processed_files is None:
        processed_files = set()

    for include in elem.xpath(".//include"):
        filename = include.text
        filepath = os.path.join(base_path, filename)
        logger.info(f"Processing include: {filepath}")

        # Check if the file has already been processed
        if filepath in processed_files:
            logger.debug(f"Skipping already processed file: {filepath}")
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


def load_xml_messages(file_path: str, filter: list) -> list:
    xml_msg = []
    parser = etree.XMLParser(remove_blank_text=True)
    tree = etree.parse(file_path, parser)
    root = tree.getroot()
    include_xml(root, os.path.dirname(os.path.abspath(file_path)))
    # Find the 'msg' element
    msg_elements = root.xpath("//messages/message")  # Default messages
    msg_elements += root.xpath(
        "//entry"
    )  # Handle the CMD scenarios which exist as enums
    if msg_elements is not None:
        for msg in msg_elements:
            msg_id = msg.get("id") or msg.get("value")
            msg_name = msg.get("name")
            # Check if the message name is inside the filter list
            if msg_name in filter:
                fields = []
                logger.debug("Debug: Found the message {}".format(msg_name))
                if msg.xpath(".//field"):
                    for entry in msg.xpath(".//field"):
                        entry_name = entry.get("name")
                        entry_value = entry.get("type")
                        entry_desc = entry.text
                        fields.append(
                            {
                                "name": entry_name,
                                "type": entry_value,
                                "desc": entry_desc,
                            }
                        )
                elif msg.xpath(".//param"):
                    for param in msg.xpath(".//param"):
                        param_name = param.get("label")
                        if param_name is None:
                            continue
                        param_min = param.get("minValue", "float")
                        param_max = param.get("maxValue", "float")
                        param_inc = param.get("increment", None)
                        param_text = param.text
                        fields.append(
                            {
                                "name": param_name,
                                "type": [param_min, param_max, param_inc],
                                "desc": param_text,
                            }
                        )
                xml_msg.append(
                    {"msg_id": msg_id, "msg_name": msg_name, "fields": fields}
                )

    else:
        logger.info("No msgs found in the XML file.")

    if xml_msg == []:
        logger.info("No messages found in the XML file.")
        exit(1)
    return xml_msg


def load_json_messages(file_path: str) -> list:
    json_msg = []
    with open(file_path, "r") as f:
        json_msg = json.load(f)
    return json_msg


def generate_field_value(field_type):
    c_type_to_py = {
        "float": "float",
        "double": "float",
        "char": "bytes",
        "int8_t": "int",
        "uint8_t": "int",
        "uint8_t_mavlink_version": "int",
        "int16_t": "int",
        "uint16_t": "int",
        "int32_t": "int",
        "uint32_t": "int",
        "int64_t": "int",
        "uint64_t": "int",
    }
    is_array = False

    # Handle the special case for array
    if "[" in field_type:
        logger.debug("Custom case for array")
        # Extract the value between brackets
        length = int(field_type.split("[")[1][:-1])
        field_type = field_type.split("[")[0]
        is_array = True

    py_type = c_type_to_py.get(field_type)

    float_range = 1e2  # Reducing to avoid FPE on the SITL binary

    if is_array:
        return [generate_field_value(field_type) for _ in range(length)]
    if py_type == "float":
        return numpy.random.uniform(-float_range, float_range)
    elif py_type == "int":
        if field_type.startswith("uint"):
            return numpy.random.randint(
                0, 2 ** (int(field_type[4:-2])), dtype=numpy.uint64
            )
        elif field_type == "uint8_t_mavlink_version":
            return numpy.random.randint(0, 256)
        else:
            bits = int(field_type[3:-2])
            return numpy.random.randint(
                -(2 ** (bits - 1)), 2 ** (bits - 1) - 1, dtype=numpy.int64
            )
    elif py_type == "bytes":  # Should be the case where we have chars
        length = int(field_type.split("[")[1][:-1])
        # Return a random string of length
        return "".join(random.choice(string.ascii_letters) for _ in range(length))
    else:
        raise ValueError(f"Unsupported type: {field_type}")


class FuzzClass:
    def __init__(self, args, xml_info):
        self.target = args.target
        self.port = args.port
        self.xml_file_path = args.xml_file
        self.msg_id = xml_info[0]["msg_id"]
        self.msg_name = xml_info[0]["msg_name"]
        self.fields = xml_info[0]["fields"]
        self.input_cnt = 0
        self.mission_cnt = 0
        self.potential_deviation = 0  # TODO
        self.connection = None
        self.start_time = time.time()

    def connect(self):
        try:
            connection = mavutil.mavlink_connection(f"udp:{self.target}:{self.port}")
            connection.wait_heartbeat()
            logger.debug(f"Connected to {self.target} on port {self.port}")
            self.connection = connection
        except Exception as e:
            logger.error(f"Error connecting to {self.target} on port {self.port}: {e}")
            pass

    def send(self, msg):
        if self.connection is None:
            logger.warning("No connection exists")
            return
        try:
            self.connection.mav.send(msg)
        except Exception as e:
            logger.error(f"Error connecting to {self.target} on port {self.port}: {e}")
            exit(-1)

    def mavlink_send_msg_list(
        self, msg_name: str | None, msg_id: int | None, msg: list
    ):
        if msg_name is None:
            logger.critical("MavlinkSend: Message name is None")
            return
        if "MAV_CMD" in msg_name:
            logger.debug("mavlinksend: Sending command")
            # Create a CMD_LONG message with the msg_id
            packed_msg = mavutil.mavlink.MAVLink_command_long_message(
                0,  # target_system
                0,  # target_component
                msg_id,  # command
                0,  # confirmation
                *msg,
            )
        else:
            msg_name = (
                msg_name.lower()
            )  # To ensure we match the expression inside generated MAVLINK messages
            # Get the message class dynamically
            message_class = getattr(mavutil.mavlink, f"MAVLink_{msg_name}_message")
            # Create an instance of the message
            packed_msg = message_class(*msg)
            self.connect()
            self.send(packed_msg)

    def generate_peripheral_msg(self) -> list:
        msg = []
        msg_id = self.msg_id
        msg_name = self.msg_name
        fields = self.fields
        current_time = 0
        # Generate a random value for each field
        for field in fields:
            field_name = field["name"]
            field_type = field["type"]
            # Ignore field if contains usec OR ....
            if "usec" in field_name:
                logger.debug(
                    "Ignoring field {} as it based on boot time".format(field_name)
                )
                current_time = current_milli_time(self.start_time)
                msg.append(current_time)
            elif type(field_type) is list:
                # This is a special condition where we have a custom designed values because the message is param
                min, max, inc = field_type
                field_value = None
                if (min == "float") & (max == "float") & (inc is None):
                    field_value = generate_field_value("float")
                    logger.debug(
                        "param_value: Added field {} with value {}".format(
                            field_name, field_value
                        )
                    )
                elif type(min) is int:
                    field_value = numpy.random.randint(min, max)
                    logger.debug(
                        "param_value: Added field {} with value {}".format(
                            field_name, field_value
                        )
                    )
                else:
                    logger.critical("Unsupported type for param_value")
                msg.append(field_value)
            else:
                field_value = generate_field_value(field_type)
                logger.debug(
                    "Added field {} with value {}".format(field_name, field_value)
                )
                msg.append(field_value)
        logger.info("Generated message: {}".format(msg))
        self.mavlink_send_msg_list(msg_name, msg_id, msg)
        return msg

    def fuzz(self):
        self.connect()
        self.generate_peripheral_msg()
        self.input_cnt += 1

    def status(self):
        logger.info("Tried {} inputs".format(self.input_cnt))


def main(args):
    json_msgs = []
    xml_info = []
    if args.message:
        logger.info(f"Message to fuzz: {args.message}")
        xml_info = load_xml_messages(args.xml_file, [args.message])
    # if args.json_file:
    #     json_msgs = load_json_messages(args.json_file)
    #     xml_info_array = load_xml_messages(args.xml_file, json_msgs)
    #     # Select a single message
    #     xml_info = random.choice(xml_info_array)
    fuzz = FuzzClass(args, xml_info)
    while True:
        fuzz.fuzz()
        time.sleep(0.5)
        fuzz.status()


if __name__ == "__main__":
    args = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    args.add_argument(
        "--target",
        "-t",
        help="Target IP Address",
        default="127.0.0.1",
        type=str,
    )
    args.add_argument("--port", "-p", help="Target Port", type=int, default=1337)
    args.add_argument("--xml_file", help="Message to fuzz in XML format", required=True)
    args.add_argument("--json_file", help="Message to fuzz in JSON format")
    # TODO: Allow specifying multiple values
    args.add_argument("--message", "-m", help="Message to fuzz", required=True)
    args = args.parse_args()
    main(args)
