#!/usr/bin/env python3
"""生成无需外部 RISC-V 工具链的选题三 PicoRV32 服务固件。

该脚本只使用 RV32I 指令，输出 Gowin PicoRV32 ITCM 可直接加载的
32 位十六进制字文件。这样即使测试电脑尚未安装 GMD/GCC，也能证明
CPU 取指、Simple UART 和开放 Wishbone 总线均可工作。
"""

from __future__ import annotations

import argparse
import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, List, Tuple, Union


ITCM_BASE = 0x0200_0000
UART_BASE = 0x0300_0000
SYS_BASE = 0x2000_0000
STATUS_BASE = 0x2000_1000
COMMAND_BASE = 0x2000_2000
SELFTEST_BASE = 0x2000_F000
UART_DIVISOR_50M_115200 = 434
SCRATCH_MAGIC = 0xC0DE_C0DE

X0, RA = 0, 1
T0, T1, T2 = 5, 6, 7
S0, S1 = 8, 9
A0, A1 = 10, 11
T3, T4 = 28, 29


def _check_signed(value: int, bits: int, what: str) -> None:
    lo = -(1 << (bits - 1))
    hi = (1 << (bits - 1)) - 1
    if not lo <= value <= hi:
        raise ValueError(f"{what}={value} 超出 {bits} 位有符号范围")


def _i(opcode: int, rd: int, funct3: int, rs1: int, imm: int) -> int:
    _check_signed(imm, 12, "I-immediate")
    return ((imm & 0xFFF) << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode


def _s(opcode: int, funct3: int, rs1: int, rs2: int, imm: int) -> int:
    _check_signed(imm, 12, "S-immediate")
    value = imm & 0xFFF
    return (
        ((value >> 5) << 25)
        | (rs2 << 20)
        | (rs1 << 15)
        | (funct3 << 12)
        | ((value & 0x1F) << 7)
        | opcode
    )


def _r(opcode: int, rd: int, funct3: int, rs1: int, rs2: int, funct7: int = 0) -> int:
    return (funct7 << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode


def _b(funct3: int, rs1: int, rs2: int, offset: int) -> int:
    if offset & 1:
        raise ValueError("分支偏移必须 2 字节对齐")
    _check_signed(offset, 13, "branch offset")
    value = offset & 0x1FFF
    return (
        (((value >> 12) & 1) << 31)
        | (((value >> 5) & 0x3F) << 25)
        | (rs2 << 20)
        | (rs1 << 15)
        | (funct3 << 12)
        | (((value >> 1) & 0xF) << 8)
        | (((value >> 11) & 1) << 7)
        | 0x63
    )


def _j(rd: int, offset: int) -> int:
    if offset & 1:
        raise ValueError("JAL 偏移必须 2 字节对齐")
    _check_signed(offset, 21, "jal offset")
    value = offset & 0x1F_FFFF
    return (
        (((value >> 20) & 1) << 31)
        | (((value >> 1) & 0x3FF) << 21)
        | (((value >> 11) & 1) << 20)
        | (((value >> 12) & 0xFF) << 12)
        | (rd << 7)
        | 0x6F
    )


def _u(opcode: int, rd: int, imm20: int) -> int:
    return ((imm20 & 0xF_FFFF) << 12) | (rd << 7) | opcode


Fixup = Callable[[int, Dict[str, int]], int]
Word = Union[int, Fixup]


@dataclass
class ListingItem:
    address: int
    word: int
    note: str


class Program:
    def __init__(self) -> None:
        self.words: List[Tuple[Word, str]] = []
        self.labels: Dict[str, int] = {}
        self.data_labels: Dict[str, bytes] = {}

    @property
    def pc(self) -> int:
        return ITCM_BASE + 4 * len(self.words)

    def label(self, name: str) -> None:
        if name in self.labels:
            raise ValueError(f"重复标签：{name}")
        self.labels[name] = self.pc

    def emit(self, word: Word, note: str) -> None:
        self.words.append((word, note))

    def addi(self, rd: int, rs1: int, imm: int, note: str = "addi") -> None:
        self.emit(_i(0x13, rd, 0, rs1, imm), note)

    def andi(self, rd: int, rs1: int, imm: int, note: str = "andi") -> None:
        self.emit(_i(0x13, rd, 7, rs1, imm), note)

    def sltiu(self, rd: int, rs1: int, imm: int, note: str = "sltiu") -> None:
        self.emit(_i(0x13, rd, 3, rs1, imm), note)

    def srli(self, rd: int, rs1: int, shamt: int, note: str = "srli") -> None:
        if not 0 <= shamt < 32:
            raise ValueError("移位量超出范围")
        self.emit(_i(0x13, rd, 5, rs1, shamt), note)

    def lui(self, rd: int, imm20: int, note: str = "lui") -> None:
        self.emit(_u(0x37, rd, imm20), note)

    def lw(self, rd: int, rs1: int, imm: int, note: str = "lw") -> None:
        self.emit(_i(0x03, rd, 2, rs1, imm), note)

    def lbu(self, rd: int, rs1: int, imm: int, note: str = "lbu") -> None:
        self.emit(_i(0x03, rd, 4, rs1, imm), note)

    def sw(self, rs2: int, rs1: int, imm: int, note: str = "sw") -> None:
        self.emit(_s(0x23, 2, rs1, rs2, imm), note)

    def sb(self, rs2: int, rs1: int, imm: int, note: str = "sb") -> None:
        self.emit(_s(0x23, 0, rs1, rs2, imm), note)

    def jalr(self, rd: int, rs1: int, imm: int = 0, note: str = "jalr") -> None:
        self.emit(_i(0x67, rd, 0, rs1, imm), note)

    def csrrs(self, rd: int, csr: int, rs1: int = X0, note: str = "csrrs") -> None:
        if not 0 <= csr <= 0xFFF:
            raise ValueError("CSR 地址超出范围")
        self.emit((csr << 20) | (rs1 << 15) | (2 << 12) | (rd << 7) | 0x73, note)

    def li(self, rd: int, value: int, note: str = "li") -> None:
        value &= 0xFFFF_FFFF
        signed = value if value < 0x8000_0000 else value - 0x1_0000_0000
        if -2048 <= signed <= 2047:
            self.addi(rd, X0, signed, f"{note} 0x{value:08X}")
            return
        hi = (value + 0x800) >> 12
        lo = value - ((hi & 0xF_FFFF) << 12)
        if lo >= 0x8000_0000:
            lo -= 0x1_0000_0000
        self.lui(rd, hi, f"{note}.hi 0x{value:08X}")
        if lo:
            self.addi(rd, rd, lo, f"{note}.lo 0x{value:08X}")

    def la(self, rd: int, label: str) -> None:
        def hi_fixup(_pc: int, labels: Dict[str, int]) -> int:
            value = labels[label]
            return _u(0x37, rd, (value + 0x800) >> 12)

        def lo_fixup(_pc: int, labels: Dict[str, int]) -> int:
            value = labels[label]
            hi_value = ((value + 0x800) >> 12) << 12
            return _i(0x13, rd, 0, rd, value - hi_value)

        self.emit(hi_fixup, f"la.hi {label}")
        self.emit(lo_fixup, f"la.lo {label}")

    def branch(self, funct3: int, rs1: int, rs2: int, label: str, note: str) -> None:
        self.emit(lambda pc, labels: _b(funct3, rs1, rs2, labels[label] - pc), note)

    def beq(self, rs1: int, rs2: int, label: str) -> None:
        self.branch(0, rs1, rs2, label, f"beq -> {label}")

    def bne(self, rs1: int, rs2: int, label: str) -> None:
        self.branch(1, rs1, rs2, label, f"bne -> {label}")

    def blt(self, rs1: int, rs2: int, label: str) -> None:
        self.branch(4, rs1, rs2, label, f"blt -> {label}")

    def jal(self, rd: int, label: str, note: str = "jal") -> None:
        self.emit(lambda pc, labels: _j(rd, labels[label] - pc), f"{note} -> {label}")

    def call(self, label: str) -> None:
        self.jal(RA, label, "call")

    def jump(self, label: str) -> None:
        self.jal(X0, label, "jump")

    def ret(self) -> None:
        self.jalr(X0, RA, 0, "ret")

    def add_string(self, label: str, value: str) -> None:
        self.data_labels[label] = value.encode("ascii") + b"\0"

    def finalize(self) -> Tuple[List[int], List[ListingItem]]:
        # 先将字符串紧跟在代码后并按 4 字节对齐。
        data = bytearray()
        for name, blob in self.data_labels.items():
            while data and len(data) % 4:
                data.append(0)
            self.labels[name] = ITCM_BASE + 4 * len(self.words) + len(data)
            data.extend(blob)
        while len(data) % 4:
            data.append(0)

        resolved: List[int] = []
        listing: List[ListingItem] = []
        for index, (entry, note) in enumerate(self.words):
            pc = ITCM_BASE + 4 * index
            word = entry(pc, self.labels) if callable(entry) else entry
            word &= 0xFFFF_FFFF
            resolved.append(word)
            listing.append(ListingItem(pc, word, note))
        for offset in range(0, len(data), 4):
            word = int.from_bytes(data[offset : offset + 4], "little")
            address = ITCM_BASE + 4 * len(self.words) + offset
            resolved.append(word)
            listing.append(ListingItem(address, word, "ASCII/data"))
        return resolved, listing


def build_program() -> Program:
    p = Program()

    def puts(label: str) -> None:
        p.la(A0, label)
        p.call("putstr")

    p.label("reset")
    p.li(T0, UART_BASE, "UART base")
    p.li(T1, UART_DIVISOR_50M_115200, "50MHz/115200 divisor")
    p.sw(T1, T0, 4, "UART divisor")
    p.li(S0, SELFTEST_BASE, "SELFTEST base")
    p.li(S1, SYS_BASE, "SYS base")
    puts("msg_boot")
    puts("msg_cpu")

    # 先用 scratch 真正完成一次 Wishbone 写回读，再允许任务循环。
    p.li(T1, SCRATCH_MAGIC, "scratch magic")
    p.sw(T1, S0, 0, "Wishbone scratch write")
    p.lw(T2, S0, 0, "Wishbone scratch read")
    p.bne(T1, T2, "wb_fail")
    puts("msg_wb_prefix")
    p.addi(A0, T2, 0, "scratch -> a0")
    p.call("puthex")
    puts("msg_pass")
    p.jump("after_wb")

    p.label("wb_fail")
    puts("msg_wb_prefix")
    p.addi(A0, T2, 0, "bad scratch -> a0")
    p.call("puthex")
    puts("msg_fail")

    p.label("after_wb")
    puts("msg_cycle_prefix")
    p.csrrs(A0, 0xC00, X0, "rdcycle a0")
    p.call("puthex")
    puts("msg_newline")
    puts("msg_ready")
    p.li(A1, 1, "heartbeat sequence")

    p.label("command_loop")
    # 每次轮询前写心跳；CPU 卡死时独立 RTL 看门狗会撤销运动许可。
    p.sw(A1, S1, 12, "CPU heartbeat")
    p.addi(A1, A1, 1, "heartbeat++")
    p.lw(T3, T0, 8, "UART nonblocking read")
    p.blt(T3, X0, "command_loop")
    p.andi(T3, T3, 0xFF, "RX byte")
    # '?' 的 bit5 为 1，必须在 ASCII 大小写归一化之前判断。
    p.li(T4, ord("?"))
    p.beq(T3, T4, "cmd_help")
    p.li(T4, ord("0"))
    p.beq(T3, T4, "cmd_stop")
    p.andi(T3, T3, 0xDF, "ASCII upper")
    for command, label in (
        ("P", "cmd_ping"), ("I", "cmd_id"), ("S", "cmd_status"),
        ("V", "cmd_demo"), ("A", "cmd_arm"), ("D", "cmd_disarm"),
        ("G", "cmd_go"), ("X", "cmd_clear"), ("F", "cmd_forward"),
        ("B", "cmd_backward"), ("L", "cmd_left"), ("R", "cmd_right"),
        ("Q", "cmd_spin_left"), ("E", "cmd_spin_right")
    ):
        p.li(T4, ord(command))
        p.beq(T3, T4, label)
    p.li(T4, 10)
    p.beq(T3, T4, "command_loop")
    p.li(T4, 13)
    p.beq(T3, T4, "command_loop")
    puts("msg_error")
    p.jump("command_loop")

    p.label("cmd_ping")
    puts("msg_pong")
    p.jump("command_loop")

    p.label("cmd_id")
    puts("msg_id_prefix")
    p.lw(A0, S1, 0, "runtime SYS_ID")
    p.call("puthex")
    puts("msg_id_suffix")
    p.jump("command_loop")

    p.label("cmd_status")
    # putstr/puthex 会使用 T1/T2，因此状态页基址放在 A1，
    # 避免打印前缀后将基址破坏并造成无效总线访问。
    p.li(A1, STATUS_BASE, "STATUS base")
    puts("msg_status0")
    p.lw(A0, A1, 0, "status0")
    p.call("puthex")
    puts("msg_range")
    p.lw(A0, A1, 4, "range/status")
    p.call("puthex")
    puts("msg_angles0")
    p.lw(A0, A1, 8, "roll/pitch")
    p.call("puthex")
    puts("msg_angles1")
    p.lw(A0, A1, 12, "yaw/route")
    p.call("puthex")
    puts("msg_newline")
    p.jump("command_loop")

    p.label("cmd_demo")
    p.li(A1, STATUS_BASE, "STATUS base")
    puts("msg_demo")
    p.lw(A0, A1, 16, "demo evidence")
    p.call("puthex")
    puts("msg_demo_suffix")
    p.jump("command_loop")

    def emit_command(label: str, value: int, message: str) -> None:
        p.label(label)
        p.li(T1, COMMAND_BASE, "COMMAND base")
        p.li(T2, value, "command bits")
        p.sw(T2, T1, 0, "command pulse")
        puts(message)
        p.jump("command_loop")

    emit_command("cmd_arm", 1, "msg_ack_arm")
    emit_command("cmd_disarm", 2, "msg_ack_disarm")
    emit_command("cmd_go", 4, "msg_ack_go")
    emit_command("cmd_clear", 8, "msg_ack_clear")
    emit_command("cmd_stop", 0x100, "msg_ack_manual")
    emit_command("cmd_forward", 0x300, "msg_ack_manual")
    emit_command("cmd_backward", 0x500, "msg_ack_manual")
    emit_command("cmd_left", 0x700, "msg_ack_manual")
    emit_command("cmd_right", 0x900, "msg_ack_manual")
    emit_command("cmd_spin_left", 0xB00, "msg_ack_manual")
    emit_command("cmd_spin_right", 0xD00, "msg_ack_manual")

    p.label("cmd_help")
    puts("msg_help")
    p.jump("command_loop")

    p.label("putstr")
    p.lbu(T1, A0, 0, "load char")
    p.beq(T1, X0, "putstr_done")
    p.sb(T1, T0, 8, "UART TX")
    p.addi(A0, A0, 1, "next char")
    p.jump("putstr")
    p.label("putstr_done")
    p.ret()

    p.label("puthex")
    for index, shift in enumerate(range(28, -1, -4)):
        decimal = f"hex_decimal_{index}"
        emit = f"hex_emit_{index}"
        p.srli(T1, A0, shift, f"hex nibble >> {shift}")
        p.andi(T1, T1, 0xF)
        p.sltiu(T2, T1, 10)
        p.bne(T2, X0, decimal)
        p.addi(T1, T1, ord("A") - 10)
        p.jump(emit)
        p.label(decimal)
        p.addi(T1, T1, ord("0"))
        p.label(emit)
        p.sb(T1, T0, 8, "UART TX hex")
    p.ret()

    p.add_string("msg_boot", "CHOICE3 ROBOT BOOT\r\n")
    p.add_string("msg_cpu", "CPU=PICORV32-RV32IMC;TARGET=GW5A-25B\r\n")
    p.add_string("msg_wb_prefix", "WB_SCRATCH=0x")
    p.add_string("msg_pass", ";RESULT=PASS\r\n")
    p.add_string("msg_fail", ";RESULT=FAIL\r\n")
    p.add_string("msg_cycle_prefix", "BOOT_CYCLE=0x")
    p.add_string("msg_newline", "\r\n")
    p.add_string("msg_ready", "READY:P/I/S/V/A/D/G/X/F/B/L/R/Q/E/0/?\r\n")
    p.add_string("msg_pong", "PONG;CHOICE3=READY;CORE=PICORV32\r\n")
    p.add_string("msg_id_prefix", "ID=0x")
    p.add_string("msg_id_suffix", ";TARGET=GW5A-25B;ABI=2\r\n")
    p.add_string("msg_status0", "STATUS0=0x")
    p.add_string("msg_range", ";RANGE=0x")
    p.add_string("msg_angles0", ";ROLL_PITCH=0x")
    p.add_string("msg_angles1", ";YAW_ROUTE=0x")
    p.add_string("msg_demo", "DEMO=0x")
    p.add_string("msg_demo_suffix", ";EXPECT=000000FF\r\n")
    p.add_string("msg_ack_arm", "ACK;ARM\r\n")
    p.add_string("msg_ack_disarm", "ACK;DISARM\r\n")
    p.add_string("msg_ack_go", "ACK;GO=WAYPOINTS\r\n")
    p.add_string("msg_ack_clear", "ACK;FAULT_CLEAR\r\n")
    p.add_string("msg_ack_manual", "ACK;MANUAL\r\n")
    p.add_string("msg_help", "COMMANDS:P,I,S,V,A,D,G,X,F,B,L,R,Q,E,0,?\r\n")
    p.add_string("msg_error", "ERR;USE=?\r\n")
    return p


def _sign_extend(value: int, bits: int) -> int:
    sign = 1 << (bits - 1)
    return (value & (sign - 1)) - (value & sign)


def simulate_boot(words: List[int]) -> str:
    """用极小 RV32I 解释器验证启动路径，直到固件等待首个串口命令。"""
    image = b"".join(word.to_bytes(4, "little") for word in words)
    regs = [0] * 32
    pc = ITCM_BASE
    cycle = 0
    uart_divisor = None
    scratch = 0
    heartbeat = 0
    command = 0
    output = bytearray()

    def load(address: int, size: int) -> int:
        nonlocal scratch, heartbeat
        if ITCM_BASE <= address and address + size <= ITCM_BASE + len(image):
            offset = address - ITCM_BASE
            return int.from_bytes(image[offset : offset + size], "little")
        if address == SYS_BASE and size == 4:
            return 0x5242_4F54  # "RBOT"
        if address == SYS_BASE + 12 and size == 4:
            return heartbeat
        if STATUS_BASE <= address <= STATUS_BASE + 16 and size == 4:
            return {
                STATUS_BASE + 0: 0xFF00_0103,
                STATUS_BASE + 4: 0x8000_0320,
                STATUS_BASE + 8: 0x0000_0000,
                STATUS_BASE + 12: 0x0000_0000,
                STATUS_BASE + 16: 0x0000_00FF,
            }[address]
        if address == SELFTEST_BASE and size == 4:
            return scratch
        if address == UART_BASE + 8:
            raise StopIteration
        raise RuntimeError(f"仿真读取未映射地址 0x{address:08X}")

    def store(address: int, size: int, value: int) -> None:
        nonlocal scratch, uart_divisor, heartbeat, command
        mask = (1 << (size * 8)) - 1
        value &= mask
        if address == UART_BASE + 4 and size == 4:
            uart_divisor = value
            return
        if address == UART_BASE + 8:
            output.append(value & 0xFF)
            return
        if SELFTEST_BASE <= address < SELFTEST_BASE + 4:
            shift = 8 * (address - SELFTEST_BASE)
            byte_mask = mask << shift
            scratch = (scratch & ~byte_mask) | ((value << shift) & byte_mask)
            return
        if address == SYS_BASE + 12 and size == 4:
            heartbeat = value
            return
        if address == COMMAND_BASE and size == 4:
            command = value
            return
        raise RuntimeError(f"仿真写入未映射地址 0x{address:08X}")

    for _step in range(100_000):
        try:
            insn = load(pc, 4)
        except StopIteration as exc:
            raise RuntimeError("PC 意外进入 UART 地址") from exc
        opcode = insn & 0x7F
        rd = (insn >> 7) & 0x1F
        funct3 = (insn >> 12) & 7
        rs1 = (insn >> 15) & 0x1F
        rs2 = (insn >> 20) & 0x1F
        next_pc = (pc + 4) & 0xFFFF_FFFF
        write_value = None

        try:
            if opcode == 0x37:  # LUI
                write_value = insn & 0xFFFFF000
            elif opcode == 0x13:  # OP-IMM
                imm = _sign_extend(insn >> 20, 12)
                if funct3 == 0:
                    write_value = regs[rs1] + imm
                elif funct3 == 3:
                    write_value = int((regs[rs1] & 0xFFFF_FFFF) < (imm & 0xFFFF_FFFF))
                elif funct3 == 5 and ((insn >> 25) & 0x7F) == 0:
                    write_value = (regs[rs1] & 0xFFFF_FFFF) >> ((insn >> 20) & 0x1F)
                elif funct3 == 7:
                    write_value = regs[rs1] & imm
                else:
                    raise RuntimeError(f"不支持 OP-IMM funct3={funct3}")
            elif opcode == 0x03:  # LOAD
                address = (regs[rs1] + _sign_extend(insn >> 20, 12)) & 0xFFFF_FFFF
                if funct3 == 2:
                    write_value = load(address, 4)
                elif funct3 == 4:
                    write_value = load(address, 1)
                else:
                    raise RuntimeError(f"不支持 LOAD funct3={funct3}")
            elif opcode == 0x23:  # STORE
                imm = ((insn >> 25) << 5) | ((insn >> 7) & 0x1F)
                address = (regs[rs1] + _sign_extend(imm, 12)) & 0xFFFF_FFFF
                if funct3 == 0:
                    store(address, 1, regs[rs2])
                elif funct3 == 2:
                    store(address, 4, regs[rs2])
                else:
                    raise RuntimeError(f"不支持 STORE funct3={funct3}")
            elif opcode == 0x63:  # BRANCH
                imm = (
                    (((insn >> 31) & 1) << 12)
                    | (((insn >> 7) & 1) << 11)
                    | (((insn >> 25) & 0x3F) << 5)
                    | (((insn >> 8) & 0xF) << 1)
                )
                lhs = regs[rs1] & 0xFFFF_FFFF
                rhs = regs[rs2] & 0xFFFF_FFFF
                taken = False
                if funct3 == 0:
                    taken = lhs == rhs
                elif funct3 == 1:
                    taken = lhs != rhs
                elif funct3 == 4:
                    taken = _sign_extend(lhs, 32) < _sign_extend(rhs, 32)
                else:
                    raise RuntimeError(f"不支持 BRANCH funct3={funct3}")
                if taken:
                    next_pc = (pc + _sign_extend(imm, 13)) & 0xFFFF_FFFF
            elif opcode == 0x6F:  # JAL
                imm = (
                    (((insn >> 31) & 1) << 20)
                    | (((insn >> 12) & 0xFF) << 12)
                    | (((insn >> 20) & 1) << 11)
                    | (((insn >> 21) & 0x3FF) << 1)
                )
                write_value = next_pc
                next_pc = (pc + _sign_extend(imm, 21)) & 0xFFFF_FFFF
            elif opcode == 0x67:  # JALR
                write_value = next_pc
                next_pc = (regs[rs1] + _sign_extend(insn >> 20, 12)) & 0xFFFF_FFFE
            elif opcode == 0x73 and funct3 == 2:  # CSRRS，仅验证 cycle
                csr = (insn >> 20) & 0xFFF
                if csr != 0xC00:
                    raise RuntimeError(f"不支持 CSR 0x{csr:03X}")
                write_value = cycle
            else:
                raise RuntimeError(f"0x{pc:08X} 存在不支持的指令 0x{insn:08X}")
        except StopIteration:
            break

        if rd and write_value is not None:
            regs[rd] = write_value & 0xFFFF_FFFF
        regs[0] = 0
        pc = next_pc
        cycle += 1
    else:
        raise RuntimeError("固件启动仿真超时")

    text = output.decode("ascii")
    required = [
        "CHOICE3 ROBOT BOOT",
        "WB_SCRATCH=0xC0DEC0DE;RESULT=PASS",
        "READY:P/I/S/V/A/D/G/X/F/B/L/R/Q/E/0/?",
    ]
    missing = [token for token in required if token not in text]
    if missing:
        raise RuntimeError(f"启动输出缺少：{missing}\n{text}")
    if uart_divisor != UART_DIVISOR_50M_115200:
        raise RuntimeError(f"UART 分频错误：{uart_divisor}")
    return text


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, default=Path(__file__).resolve().parent / "build")
    args = parser.parse_args()

    words, listing = build_program().finalize()
    if len(words) * 4 > 8 * 1024:
        raise SystemExit("固件超过 8 KiB ITCM")
    if words[:3] != [0x030002B7, 0x1B200313, 0x0062A223]:
        raise SystemExit(f"启动机器码自检失败：{[f'{word:08X}' for word in words[:3]]}")
    boot_output = simulate_boot(words)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    hex_text = "".join(f"{word:08x}\n" for word in words)
    hex_path = args.out_dir / "picorv32_selftest.hex"
    hex_path.write_text(hex_text, encoding="ascii", newline="\n")

    lst_lines = [f"{item.address:08X}: {item.word:08X}  {item.note}" for item in listing]
    (args.out_dir / "picorv32_selftest.lst").write_text("\n".join(lst_lines) + "\n", encoding="utf-8")

    manifest = {
        "isa": "RV32I machine code (runs on configured RV32IMC core)",
        "itcm_base": f"0x{ITCM_BASE:08X}",
        "bytes": len(words) * 4,
        "words": len(words),
        "sha256": hashlib.sha256(hex_text.encode("ascii")).hexdigest(),
        "uart": {"clock_hz": 50_000_000, "baud": 115_200, "divisor": UART_DIVISOR_50M_115200},
        "selftest_magic": f"0x{SCRATCH_MAGIC:08X}",
        "commands": ["P", "I", "S", "V", "A", "D", "G", "X", "F", "B", "L", "R", "Q", "E", "0", "?"],
        "simulated_boot_output": boot_output,
    }
    (args.out_dir / "picorv32_selftest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(manifest, ensure_ascii=False, indent=2))
    print(f"HEX: {hex_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
