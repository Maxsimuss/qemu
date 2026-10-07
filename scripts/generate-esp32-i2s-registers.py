#!/usr/bin/env python3
"""Generate register masks/reset values from Espressif's published ESP32 header."""
import argparse
import pathlib
import re

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("header", type=pathlib.Path, help="ESP-IDF ESP32 i2s_reg.h")
parser.add_argument("--output", type=pathlib.Path,
                    default=pathlib.Path(__file__).resolve().parents[1] /
                    "hw/misc/esp32_i2s_regs.inc")
args = parser.parse_args()
source = args.header.read_text()
regs={}
for block in re.split(r'(?=^#define I2S_\w+_REG\(i\))',source,flags=re.M):
    m=re.match(r'#define (I2S_\w+_REG)\(i\).*\+ (0x[0-9a-fA-F]+)',block)
    if not m: continue
    name,offset=m.group(1),int(m.group(2),16)
    reset=read=write=0
    for f in re.finditer(r'/\* (I2S_\w+)\s*: (R/W|RO|WO|R/WTC)\s*;bitpos:\[(\d+)(?::(\d+))?\]\s*;default:\s*(\d+)\s*\'([hbd])([0-9a-fA-F]+)',block):
        fname,access,hi,lo,width,base,value=f.groups()
        hi=int(hi); lo=int(lo or hi)
        mask=((1<<(hi-lo+1))-1)<<lo
        val=int(value,{'h':16,'d':10,'b':2}[base])
        reset|=(val<<lo)&mask
        if access!='WO': read|=mask
        if access!='RO': write|=mask
    regs[offset]=(name,reset,read,write)
# FIFO aliases are omitted from vendor header, specified by TRM 22.7.
regs[0]=('I2S_FIFO_WR_REG',0,0,0xffffffff)
regs[4]=('I2S_FIFO_RD_REG',0,0xffffffff,0)
text='/* Generated from ESP-IDF ESP32 i2s_reg.h; see scripts/generate-esp32-i2s-registers.py. */\n'
text+='static const I2SRegisterInfo i2s_reg_info[64] = {\n'
for off,(name,reset,read,write) in sorted(regs.items()):
    text+=f'    [0x{off:02x} / 4] = {{ 0x{reset:08x}, 0x{read:08x}, 0x{write:08x} }}, /* {name} */\n'
text+='};\n'
args.output.write_text(text)
print(len(regs),'register definitions')
