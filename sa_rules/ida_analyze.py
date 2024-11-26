#!/usr/bin/env python3
# -*- coding:utf-8 -*-
###
# Finis coronat opus; Run this at your own peril ~ silipwn
# File: ida_analyze.py
# SPDX-License-Identifier: BSD-3-Clause or GPL-3.0-or-later
# Author: silipwn (contact@as-hw.in)
# Description: For extracting MAVLINK messages from a binary using IDA Pro
# Date: 2024-11-22T12:24:56-0500
# Last-Modified: 2024-11-26T11:03:27-0500
# uv deps
# Package        Version
# -------------- -------
# appdirs        1.4.4
# headless-ida   0.6.1
# ida            0.0.1
# jedi           0.19.2
# jpype1         1.5.1
# lxml           5.3.0
# packaging      24.2
# parso          0.8.4
# plumbum        1.9.0
# prompt-toolkit 3.0.48
# ptpython       3.0.29
# pygments       2.18.0
# pyhidra        1.3.0
# rpyc           6.0.1
# wcwidth        0.2.13
###
from argparse import ArgumentParser
import re
import logging
import os
import json
from lxml import etree

# Parse command-line arguments
parser = ArgumentParser()
parser.add_argument("--binary","--bin", help="Path to binary/IDB")
parser.add_argument("--ida",help="Path to IDAt")
parser.add_argument("--xml",help="Path to public XML file")
args = parser.parse_args()

# Setup logging with colors 
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
logger = logging

# Initialize HeadlessIda
from headless_ida import HeadlessIda
headlessida = HeadlessIda(
    "/home/silipwn/Documents/idapro_9/idat64",
    args.binary
)
# Import IDA Modules (make sure you have initialized HeadlessIda first)
from headless_ida.ida_headers import *
import idaapi

ida_auto.auto_wait()
# Address of the function to decompile
identified_addr = 0x80ABFC4


# =======
# Custom loader function to include XML files
def include_xml(elem, base_path, processed_files=None):
    if processed_files is None:
        processed_files = set()

    for include in elem.xpath(".//include"):
        filename = include.text
        filepath = os.path.join(base_path, filename)
        logger.debug(f"Processing include: {filepath}")

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
            if int(msg_id) in filter:
                fields = []
                logger.debug("Found the potential message {} for ID {}".format(msg_name,msg_id))
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
        logger.info("No msgs loaded from the XML file.")
    if xml_msg == []:
        logger.info("No messages matched found in the XML file.")
        exit(1)
    return xml_msg

def decompile_function(ea):
    """
    Decompile the function at the specified address.
    :param ea: Effective address of the function.
    :return: Decompiled function as a string or an error message.
    """
    if not ida_hexrays.init_hexrays_plugin():
        logger.warning("Hex-Rays decompiler is not available.")
        return None

    func = idaapi.get_func(ea)
    if not func:
        logger.warning(f"No function found at address 0x{ea:X}")
        return None

    try:
        cfunc = ida_hexrays.decompile(func)
        if not cfunc:
            logger.info(f"Failed to decompile function at 0x{ea:X}")
            return None
        return str(cfunc)
    except ida_hexrays.DecompilationFailure as e:
        logger.error(f"Decompilation failed: {e}")

decompiled_code = decompile_function(identified_addr)

# Extract the case statements from the decompiled code
case_pattern = re.compile(r"case (\w+):")
case_statements = case_pattern.findall(decompiled_code)
import copy
case_statements_clone = copy.deepcopy(case_statements)
if case_statements is None:
    logger.warning("No case statements found in the decompiled code.")
logger.info(f"Located {len(case_statements)} case statements in the decompiled code.")
# Remove the 'u' suffix from the case statements
common_messages = [0,2] # Currently manually blacklisted 
case_statements = [int(case.replace('u',''),16) for case in case_statements]
for msg in common_messages:
    if msg in case_statements:
        case_statements.remove(msg)
xml_messages = load_xml_messages(args.xml, case_statements)
if xml_messages is None:
    logger.error("No messages found in the XML file.")
    exit(1)
else:
    logger.info("Messages extracted successfully.")
    logger.info(f"Found {len(xml_messages)} potential matches in the binary. Saving to messages.json for further processing.")
# Save the xml_messages as a JSON file
with open("messages.json", "w") as f:
    json.dump(xml_messages, f, indent=2)