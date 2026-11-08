"""RDK X5 AT8236 hardware PWM. Zero means disabled, not enabled-at-zero."""
from pathlib import Path

class MotorPWM:
    PINS = {27: ('34160000.pwm', 1), 29: ('34140000.pwm', 0),
            31: ('34140000.pwm', 1), 33: ('34170000.pwm', 1)}

    def __init__(self, pins, frequency=200):
        if frequency <= 0:
            raise ValueError('PWM frequency must be positive')
        self.paths = {}; self.values = {}
        period = round(1e9 / frequency)
        self.period = period
        try:
            for pin in pins:
                device, channel = self.PINS[pin]
                chips = [p for p in Path('/sys/class/pwm').glob('pwmchip*')
                         if (p / 'device').resolve().name == device]
                if len(chips) != 1:
                    raise RuntimeError('Cannot locate PWM controller for BOARD %s' % pin)
                chip = chips[0]; target = chip / ('pwm%d' % channel)
                if not target.exists(): (chip / 'export').write_text(str(channel))
                if int((target / 'period').read_text()) == 0:
                    (target / 'period').write_text(str(period))
                (target / 'enable').write_text('0')
                (target / 'duty_cycle').write_text('0')
                self.paths[pin] = target
                (target / 'period').write_text(str(period))
                (target / 'polarity').write_text('normal')
                self.values[pin] = 0
        except BaseException:
            self.stop()
            raise

    def set(self, pin, percent):
        if not 0 <= percent <= 100: raise ValueError('Duty outside 0..100')
        if self.values.get(pin) == percent: return
        p = self.paths[pin]
        if percent == 0:
            (p / 'enable').write_text('0')
            (p / 'duty_cycle').write_text('0')
        else:
            (p / 'duty_cycle').write_text(str(round(self.period * percent / 100)))
            (p / 'enable').write_text('1')
        self.values[pin] = percent

    def drive(self, in1, in2, signed_duty, forward_input=2):
        if signed_duty == 0:
            self.set(in1, 0); self.set(in2, 0); return
        active = in2 if forward_input == 2 else in1
        if signed_duty < 0: active = in1 if active == in2 else in2
        inactive = in2 if active == in1 else in1
        self.set(inactive, 0)
        self.set(active, abs(signed_duty))

    def stop(self):
        errors = []
        for pin, p in self.paths.items():
            try:
                (p / 'enable').write_text('0')
                (p / 'duty_cycle').write_text('0')
                self.values[pin] = 0
            except OSError as exc: errors.append(str(exc))
        if errors: raise RuntimeError('PWM stop failures: ' + '; '.join(errors))
