#!/usr/bin/env python3

import argparse
import ast
import pathlib
import struct


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("qt_source", type=pathlib.Path)
    parser.add_argument("output", type=pathlib.Path)
    args = parser.parse_args()

    tree = ast.parse(args.qt_source.read_text())
    table_node = next(
        node.value
        for node in tree.body
        if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == "CRC8_TABLE"
                for target in node.targets)
    )
    crc_table = bytes(ast.literal_eval(table_node.args[0]))
    packets = bytearray()

    for packet_index in range(12):
        packet = bytearray((0x02, 0x01, 0xEA, 0xF1, 0x01, packet_index))
        for point in range(243):
            wire_index = packet_index * 243 + point
            if wire_index < 40 * 71:
                value = wire_index - 1458
            elif wire_index < 71 * 41:
                value = 0
            else:
                value = 30000  # sentinel proving the five trailing values are padding
            packet.extend(struct.pack(">h", value))
        crc = 0
        for byte in packet[1:492]:
            crc = crc_table[crc ^ byte]
        packet.extend((crc, 0x03))
        if len(packet) != 494:
            raise RuntimeError("Qt tactile packet size changed")
        packets.extend(packet)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(packets)


if __name__ == "__main__":
    main()
