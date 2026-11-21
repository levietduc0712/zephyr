#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

"""Check a FRDM-i.MX95 M7 ELF, raw binary and map without accessing hardware."""

import argparse
import json
import re
import struct
import sys
from pathlib import Path

from elftools.elf.elffile import ELFFile


def require(condition, message):
    if not condition:
        raise ValueError(message)


def verify(build):
    directory = build / 'zephyr'
    config = dict(
        re.findall(r'^(CONFIG_\w+)=(.*)$', (directory / '.config').read_text(), re.MULTILINE)
    )
    require(config.get('CONFIG_BOARD_FRDM_IMX95') == 'y', 'Wrong board')
    require(config.get('CONFIG_SOC_MIMX9596_M7') == 'y', 'Wrong CPU')
    ddr = config.get('CONFIG_BOARD_FRDM_IMX95_MIMX9596_M7_DDR') == 'y'
    require((config.get('CONFIG_XIP') == 'y') != ddr, 'Unexpected XIP setting')
    rom = (0x80000000, 0x8005A000) if ddr else (0, 0x40000)
    ram = rom if ddr else (0x20004000, 0x2001E000)
    binary = (directory / 'zephyr.bin').read_bytes()
    map_text = (directory / 'zephyr.map').read_text()

    def inside(address, size, region):
        return region[0] <= address and address + size <= region[1]

    with (directory / 'zephyr.elf').open('rb') as stream:
        elf = ELFFile(stream)
        require(elf.elfclass == 32 and elf.little_endian, 'Expected little-endian ELF32')
        require(elf['e_machine'] == 'EM_ARM', 'Expected ARM ELF')
        symbols = {s.name: s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
        vector = symbols['_vector_start']
        reset = symbols['z_arm_reset']
        msp, reset_word = struct.unpack_from('<II', binary)
        require(vector == rom[0], 'Vector table is not at the image base')
        require(reset_word == (reset | 1), 'Reset vector does not name z_arm_reset in Thumb state')
        require(elf['e_entry'] == (reset | 1), 'Unexpected ELF entry point')
        require(inside(reset & ~1, 2, rom), 'Reset handler is outside executable memory')
        require(inside(msp - 1, 1, ram) and msp % 8 == 0, 'Invalid initial MSP')
        require(
            msp == symbols['z_main_stack'] + int(config['CONFIG_MAIN_STACK_SIZE'], 0),
            'MSP does not match the main stack top',
        )
        for name in ('_vector_start', 'z_arm_reset'):
            match = re.search(r'^\s*(0x[0-9a-fA-F]+)\s+' + name + r'\b', map_text, re.MULTILINE)
            require(
                match is not None and int(match[1], 16) == (symbols[name] & ~1),
                f'Map disagrees with ELF: {name}',
            )

        loads = [p for p in elf.iter_segments() if p['p_type'] == 'PT_LOAD']
        file_loads = [p for p in loads if p['p_filesz']]
        require(min(p['p_paddr'] for p in file_loads) == vector, 'Unexpected binary load base')
        require(
            all(inside(p['p_paddr'], p['p_filesz'], rom) for p in file_loads),
            'File-backed load extends outside the image window',
        )
        require(
            len(binary) == max(p['p_paddr'] + p['p_filesz'] for p in file_loads) - vector,
            'Binary length disagrees with ELF load segments',
        )

        checked = 0
        for section in elf.iter_sections():
            if not section['sh_flags'] & 2 or not section['sh_size']:
                continue
            addr, size = section['sh_addr'], section['sh_size']
            require(
                inside(addr, size, rom) or inside(addr, size, ram),
                f'Allocated section outside linker windows: {section.name}',
            )
            if section['sh_flags'] & 4:
                require(inside(addr, size, rom), f'Code outside executable window: {section.name}')
            if section['sh_type'] == 'SHT_NOBITS':
                continue
            segment = next((p for p in loads if p.section_in_segment(section)), None)
            require(segment is not None, f'No load segment for {section.name}')
            offset = segment['p_paddr'] + addr - segment['p_vaddr'] - vector
            require(
                binary[offset : offset + size] == section.data(),
                f'Binary bytes disagree with ELF: {section.name}',
            )
            checked += 1

        sections = {}
        for name in ('rom_start', 'text', 'datas', 'bss', 'noinit'):
            section = elf.get_section_by_name(name)
            require(section is not None, f'Missing section: {name}')
            sections[name] = {
                'address': f"0x{section['sh_addr']:08x}",
                'size': f"0x{section['sh_size']:x}",
            }
        require(
            inside(
                symbols['__data_region_start'],
                symbols['__data_region_end'] - symbols['__data_region_start'],
                ram,
            ),
            'Initialized writable data is outside RAM',
        )
        require(
            inside(symbols['__bss_start'], symbols['__bss_end'] - symbols['__bss_start'], ram),
            'BSS is outside RAM',
        )

        return {
            'build': str(build.resolve()),
            'variant': 'ddr' if ddr else 'tcm',
            'vector': f'0x{vector:08x}',
            'initial_msp': f'0x{msp:08x}',
            'reset_instruction': f'0x{reset & ~1:08x}',
            'reset_vector_thumb': f'0x{reset_word:08x}',
            'elf_entry': f"0x{elf['e_entry']:08x}",
            'data_load': f"0x{symbols['__data_load_start']:08x}",
            'image_ram_end': f"0x{symbols['_image_ram_end']:08x}",
            'binary_bytes': len(binary),
            'verified_file_sections': checked,
            'sections': sections,
            'load_segments': [
                {
                    'vma': f"0x{p['p_vaddr']:08x}",
                    'lma': f"0x{p['p_paddr']:08x}",
                    'file_bytes': p['p_filesz'],
                    'memory_bytes': p['p_memsz'],
                }
                for p in loads
            ],
            'result': 'PASS (static checks only)',
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('build', type=Path, help='West build directory')
    args = parser.parse_args()
    try:
        print(json.dumps(verify(args.build), indent=2))
    except (KeyError, OSError, ValueError, struct.error) as error:
        print(f'FAIL: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
