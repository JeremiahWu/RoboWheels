"""Register-level GPIO for Orange Pi 3 LTS (Allwinner H6).

Replaces OPi.GPIO for the motor/encoder hot path. OPi.GPIO 0.5.2 is
sysfs-based: every output()/input() re-opens the value file (~100-200 us),
far too slow for 200 Hz software PWM on 4 pins + 4x quadrature sampling
(measured loop ~300-600 Hz -> aliased duty, car did not move).

This module mmaps /dev/mem and drives the PIO registers directly
(~3 us per pin op). Register model copied from wiringOP v2.46 source
(drivers/wiringOP, proven on this board via the gpio utility):

  pin "PxNN": bank = letter - 'A' (A=0 .. L=11), index = NN
  bank 11 (PL) -> R_PIO base 0x07022000, others -> 0x0300B000 + bank*0x24
  CFG register offset = (index >> 3) * 4,  shift = (index & 7) * 4
      0b000 = input, 0b001 = output (mask 0x7)
  DAT register offset = 0x10, bit = index (1 = high)

Verified 2026-08-27: register read of PL_DAT matched the sysfs value
file. Do not change addresses without re-verifying.
"""
import mmap
import os

_PIO_BASE = 0x0300B000
_R_PIO_BASE = 0x07022000
_BANK_STRIDE = 0x24
_DAT_OFFSET = 0x10
_CFG_MASK = 0x7
_FUNC_IN = 0x0
_FUNC_OUT = 0x1
_PAGE = 0x1000


class Pin:
    """One precomputed pin. Methods are ~us, safe for tight loops."""

    __slots__ = ('name', 'regs', 'dat_off', 'bit', 'cfg_off', 'cfg_shift')

    def out(self, high):
        off = self.dat_off
        v = int.from_bytes(self.regs[off:off + 4], 'little')
        if high:
            v |= 1 << self.bit
        else:
            v &= ~(1 << self.bit)
        self.regs[off:off + 4] = v.to_bytes(4, 'little')

    def inp(self):
        off = self.dat_off
        v = int.from_bytes(self.regs[off:off + 4], 'little')
        return (v >> self.bit) & 1


class FastGPIO:
    """Owns the /dev/mem mappings; Pin objects borrow them."""

    def __init__(self):
        self._fd = os.open('/dev/mem', os.O_RDWR | os.O_SYNC)
        self._maps = {}

    def _map(self, base):
        page = base & ~(_PAGE - 1)
        if page not in self._maps:
            self._maps[page] = mmap.mmap(
                self._fd, _PAGE, access=mmap.ACCESS_WRITE, offset=page)
        return self._maps[page], base - page

    def _decode(self, pin_name):
        bank = ord(pin_name[1]) - ord('A')
        index = int(pin_name[2:])
        base = _R_PIO_BASE if bank == 11 else _PIO_BASE + bank * _BANK_STRIDE
        return base, index

    def pin(self, pin_name):
        """Create a Pin object (direction is not changed here)."""
        base, index = self._decode(pin_name)
        regs, off = self._map(base)
        p = Pin()
        p.name = pin_name
        p.regs = regs
        p.dat_off = off + _DAT_OFFSET
        p.bit = index
        p.cfg_off = off + (index >> 3) * 4
        p.cfg_shift = (index & 7) * 4
        return p

    def setup(self, pin_name, direction):
        """direction: 'in' or 'out'. For 'out' the pad is driven LOW
        before the function is switched, so IN pins never float high."""
        base, index = self._decode(pin_name)
        regs, off = self._map(base)
        cfg_off = off + (index >> 3) * 4
        shift = (index & 7) * 4
        if direction == 'out':
            # drive LOW first, then switch function to output
            dat_off = off + _DAT_OFFSET
            v = int.from_bytes(regs[dat_off:dat_off + 4], 'little')
            v &= ~(1 << index)
            regs[dat_off:dat_off + 4] = v.to_bytes(4, 'little')
            v = int.from_bytes(regs[cfg_off:cfg_off + 4], 'little')
            v = (v & ~(_CFG_MASK << shift)) | (_FUNC_OUT << shift)
            regs[cfg_off:cfg_off + 4] = v.to_bytes(4, 'little')
        else:
            v = int.from_bytes(regs[cfg_off:cfg_off + 4], 'little')
            v &= ~(_CFG_MASK << shift)
            regs[cfg_off:cfg_off + 4] = v.to_bytes(4, 'little')

    def close(self):
        for m in self._maps.values():
            m.close()
        self._maps.clear()
        os.close(self._fd)
