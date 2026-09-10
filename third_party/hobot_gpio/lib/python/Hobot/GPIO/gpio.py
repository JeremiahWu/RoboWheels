################################################################################
# Copyright (c) 2024,D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

from Hobot.GPIO import gpio_event as event
from Hobot.GPIO import gpio_pin_data
import os
import time
import copy
import warnings

# sysfs gpio
SYSFS_GPIO = "/sys/class/gpio"
SYSFS_PLATFORM_PATH = '/sys/devices/platform/'

# MODE_LIST
BOARD = 'BOARD'
BCM = 'BCM'
CVM = 'CVM'
SOC = 'SOC'
TEGRA_SOC = 'TEGRA_SOC'
MODE_LIST = [BOARD, BCM, CVM, SOC, TEGRA_SOC]

# LEVEL_LIST
HIGH = 1
LOW = 0
LEVEL_LIST = [HIGH, LOW]

# EDGE_LIST
RISING = 'RISING'
FALLING = 'FALLING'
BOTH = 'BOTH'
EDGE_LIST = [RISING, FALLING, BOTH]

# status list
UNKNOWN = None
OUT = "OUT"
IN = "IN"
HARD_PWM = "HARD_PWM"
DIRECTION_LIST = [OUT, IN]

gpio_warning = True
model, all_pin_data = gpio_pin_data.get_all_pin_data()
pin_mode = None
pin_info = {}


class PinPro(object):
    def __init__(self, pin_name):
        self.gpio_id = None
        self.gpio_export = None
        self.gpio_unexport = None
        self.gpio_name = None
        self.gpio_value = None
        self.gpio_direction = None
        self.gpio_edge = None

        self.pwm_id = None
        self.pwm_export = None
        self.pwm_unexport = None
        self.pwm_name = None
        self.pwm_enable = None
        self.pwm_period = None
        self.pwm_duty_cycle = None

        self.pin_status = None
        try:
            self.gpio_id = str(all_pin_data[pin_mode][pin_name].gpio_id)
            self.gpio_export = os.path.join(SYSFS_GPIO, 'export')
            self.gpio_unexport = os.path.join(SYSFS_GPIO, 'unexport')
            self.gpio_name = os.path.join(SYSFS_GPIO, 'gpio' + self.gpio_id)
            self.gpio_value = os.path.join(self.gpio_name, "value")
            self.gpio_direction = os.path.join(self.gpio_name, "direction")
            self.gpio_edge = os.path.join(self.gpio_name, "edge")
        except Exception as exc:
            pass
        try:
            self.pwm_id = str(all_pin_data[pin_mode][pin_name].pwm_id)
            self.pwm_export = os.path.join(SYSFS_PLATFORM_PATH, all_pin_data[pin_mode][pin_name].pwm_chip_dir, 'export')
            self.pwm_unexport = os.path.join(SYSFS_PLATFORM_PATH, all_pin_data[pin_mode][pin_name].pwm_chip_dir, 'unexport')
            self.pwm_name = os.path.join(SYSFS_PLATFORM_PATH, all_pin_data[pin_mode][pin_name].pwm_chip_dir, 'pwm' + self.pwm_id)
            self.pwm_enable = os.path.join(self.pwm_name, 'enable')
            self.pwm_period = os.path.join(self.pwm_name, 'period')
            self.pwm_duty_cycle = os.path.join(self.pwm_name, 'duty_cycle')
        except Exception as exc:
            pass


def setwarnings(state):
    global gpio_warning
    gpio_warning = bool(state)


def setmode(mode):
    global pin_mode

    if mode not in MODE_LIST:
        raise ValueError("Invalid mode, must be selected in 'BOARD' 'BCM' 'CVM' 'SOC' 'TEGRA_SOC'")
    if pin_mode and pin_mode != mode:
        raise RuntimeError("A different mode has been set")
    pin_mode = mode


def getmode():
    global pin_mode
    return pin_mode


# GPIO function
def _export_gpio(pin_name, direction):
    if not os.path.exists(pin_info[pin_name].gpio_name):
        f_export = open(pin_info[pin_name].gpio_export, "w")
        f_export.write(pin_info[pin_name].gpio_id)
        f_export.close()
    else:
        if gpio_warning:
            warnings.warn("This channel(" + str(pin_name) + ") "
                          "has been exported before this operation ",
                          RuntimeWarning)

    while not os.access(pin_info[pin_name].gpio_value, os.R_OK | os.W_OK):
        time.sleep(0.01)

    f_direction = open(pin_info[pin_name].gpio_direction, 'w')
    f_direction.write(direction.lower())
    f_direction.close()

    pin_info[pin_name].pin_status = direction


def _unexport_gpio(pin_name):
    if os.path.exists(pin_info[pin_name].gpio_name):
        f_unexport = open(pin_info[pin_name].gpio_unexport, "w")
        f_unexport.write(pin_info[pin_name].gpio_id)
        f_unexport.close()
    else:
        if gpio_warning:
            warnings.warn("This channel(" + str(pin_name) + ") "
                          "did not exit before this operation ",
                          RuntimeWarning)

    while os.access(pin_info[pin_name].gpio_value, os.R_OK | os.W_OK):
        time.sleep(0.01)

    pin_info[pin_name].pin_status = None


def output(channels, values):
    if type(channels) == list or type(channels) == tuple:
        pin_names = copy.deepcopy(channels)
        pin_values = copy.deepcopy(values)
    elif type(channels) == str or type(channels) == int:
        pin_names = []
        pin_names.append(channels)
        pin_values = []
        pin_values.append(values)
    else:
        raise TypeError("The channel parameter is of the wrong type ")

    for i, pin_name in enumerate(pin_names):
        if not pin_info.__contains__(pin_name):
            raise RuntimeError("This channel is not setup")
        if pin_info[pin_name].pin_status != OUT:
            raise RuntimeError("This channel direction is not an output")
        if pin_values[i] not in LEVEL_LIST:
            raise ValueError("The value setting of this channel is invalid")

    for i, pin_name in enumerate(pin_names):
        f_value = open(pin_info[pin_name].gpio_value, 'w')
        f_value.write(str(pin_values[i]))
        f_value.close()


def input(channel):
    pin_name = channel
    if not pin_info.__contains__(pin_name):
        raise RuntimeError("This channel is not setup")
    if pin_info[pin_name].pin_status != IN:
        raise RuntimeError("This channel direction is not an input")

    f_value = open(pin_info[pin_name].gpio_value, 'r')
    value = f_value.read().rstrip()
    f_value.close()
    return int(value)

def setup(channels, direction, pull_up_down=None, initial=None):
    if not pin_mode:
        raise RuntimeError("No channel mode set")
    if direction not in DIRECTION_LIST:
        raise ValueError("Setup channel direction is invalid")
    if direction == OUT and pull_up_down:
        raise RuntimeError("Pull-up and pull-down are not "
                           "supported in output mode")

    if type(channels) == list or type(channels) == tuple:
        pin_names = copy.deepcopy(channels)
    elif type(channels) == str or type(channels) == int:
        pin_names = []
        pin_names.append(channels)
    else:
        raise TypeError("The channel parameter is of the wrong type")

    for pin_name in pin_names:
        if pin_info.__contains__(pin_name):
            _cleanup_one(pin_name)
        try:
            pin_info[pin_name] = PinPro(pin_name)
        except Exception as exc:
            raise ValueError("This channel was not found in this mode")

        _export_gpio(pin_name, direction)
        if direction == OUT and initial:
            output(pin_name, initial)


def _cleanup_one(pin_name):
    global pin_info
    event._event_cleanup(pin_info, pin_name)
    _unexport_gpio(pin_name)
    del pin_info[pin_name]

def cleanup(channels=None):
    global gpio_warning
    global pin_mode
    global pin_info
    gpio_clean_list = []
    pwm_clean_list = []

    if not channels:
        if gpio_warning:
            warnings.warn("No channels have been set up, "
                          "The next operation will clear all channels",
                          RuntimeWarning)
        gpio_clean_list = [x for x, y in pin_info.items()
                           if pin_info[x].pin_status in DIRECTION_LIST]
        pwm_clean_list = [x for x, y in pin_info.items()
                          if pin_info[x].pin_status is HARD_PWM]
    else:
        if type(channels) == list or type(channels) == tuple:
            pin_names = copy.deepcopy(channels)
        elif type(channels) == str or type(channels) == int:
            pin_names = []
            pin_names.append(channels)
        else:
            raise TypeError("The channel parameter is of the wrong type ")

        for pin_name in pin_names:
            if not pin_info[pin_name].pin_status:
                pin_mode = None
                continue
            if pin_info[pin_name].pin_status in DIRECTION_LIST:
                gpio_clean_list.append(pin_name)
            elif pin_info[pin_name].pin_status is HARD_PWM:
                pwm_clean_list.append(pin_name)

    for x in gpio_clean_list:
        try:
            event._event_cleanup(pin_info, pin_name)
        except Exception as exc:
            pass
        _unexport_gpio(x)
        del pin_info[x]
    for x in pwm_clean_list:
        _unexport_pwm(x)
        del pin_info[x]

    pin_mode = None


def gpio_function(channel):
    pin_name = channel
    res = None
    if not pin_info.__contains__(pin_name):
        res = 'UNKNOWN'
    else:
        res = pin_info[pin_name].pin_status
    return res


# EVENT function
def event_detected(channel):
    pin_name = channel
    if pin_info[pin_name].pin_status != IN:
        raise RuntimeError("Channel direction must be set to input")

    return event._is_event_occurred(pin_name)


def add_event_callback(channel, callback):
    pin_name = channel
    if not callable(callback):
        raise TypeError("Parameter must be callable")

    if pin_info[pin_name].pin_status != IN:
        raise RuntimeError("Channel direction must be set to input")

    if not event._is_event_added(pin_name):
        raise RuntimeError("No add_event_detect before this operation")

    event._add_callback(pin_name, lambda: callback(pin_name))


def add_event_detect(channel, edge, callback=None, bouncetime=None):
    pin_name = channel

    if (not callable(callback)) and callback is not None:
        raise TypeError("Callback Parameter must be callable")

    if pin_info[pin_name].pin_status != IN:
        raise RuntimeError("Channel direction must be set to input")

    if edge not in EDGE_LIST:
        raise ValueError("The edge must be set to BOTH, RISING, FALLING")

    if bouncetime is not None:
        if bouncetime < 0:
            raise ValueError("bouncetime must be greater than 0")

    result = event._add_event_detect(pin_info, pin_name, edge, bouncetime)

    if result == 0:
        if callback is not None:
            event._add_callback(pin_name, lambda: callback(pin_name))

    elif result == -1:
        raise RuntimeError("Conflicting edge detection event already exists")

    elif result == -2:
        raise RuntimeError("Epoll initialization failed")

    elif result == -3:
        raise RuntimeError("IOError occurs when gpio epoll blocks")

    elif result == -4:
        raise RuntimeError("A problem occurred while _poll_thread "
                           "was running")


def remove_event_detect(channel):
    pin_name = channel
    event._del_event(pin_info, pin_name)


def wait_for_edge(channel, edge, bouncetime=None, timeout=None):
    global pin_mode
    global pin_info
    pin_name = channel

    if not pin_mode:
        raise RuntimeError("No channel mode set")
    if not pin_info.__contains__(pin_name):
        raise RuntimeError("This channel is not setup")
    if pin_info[pin_name].pin_status != IN:
        raise RuntimeError("This channel must be setup as an input")
    if edge not in EDGE_LIST:
        raise ValueError("The edge must be set to RISING, FALLING_EDGE "
                         "or BOTH")
    if bouncetime:
        if bouncetime < 0:
            raise ValueError("bouncetime must be an integer greater than 0")

    if timeout:
        if timeout < 0:
            raise ValueError("Timeout must be greater than 0")

    result = event._add_event_block(pin_info, pin_name,
                                    edge, bouncetime, timeout)

    if not result:
        return None

    elif result == -1:
        raise RuntimeError("Conflicting edge detection event already exists")

    elif result == -2:
        raise RuntimeError("Epoll initialization failed")

    elif result == -3:
        raise RuntimeError("IOError occurs when gpio epoll blocks")

    elif result == -4:
        raise RuntimeError("File object not found after wait for GPIO")

    elif result == -5:
        raise RuntimeError("Length of value string was not 1 for GPIO")

    else:
        return pin_name


# PWM function
def _export_pwm(pin_name):
    if not os.path.exists(pin_info[pin_name].pwm_name):
        f_export = open(pin_info[pin_name].pwm_export, 'w')
        f_export.write(pin_info[pin_name].pwm_id)
        f_export.close()

    while not os.access(pin_info[pin_name].pwm_enable, os.R_OK | os.W_OK):
        time.sleep(0.01)

    pin_info[pin_name].pin_status = HARD_PWM


def _unexport_pwm(pin_name):
    if not pin_info.__contains__(pin_name):
        return
    f_unexport = open(pin_info[pin_name].pwm_unexport, 'w')
    f_unexport.write(pin_info[pin_name].pwm_id)
    f_unexport.close()

    while os.access(pin_info[pin_name].pwm_enable, os.R_OK | os.W_OK):
        time.sleep(0.01)

    pin_info[pin_name].pin_status = None


def _set_pwm_period(pin_name, period_ns):
    f_period = open(pin_info[pin_name].pwm_period, 'w')
    f_period.write(str(period_ns))
    f_period.close()

def _get_pwm_duty_cycle(pin_name):
    f_duty_cycle = open(pin_info[pin_name].pwm_duty_cycle, 'r')
    cur_duty_cycle = f_duty_cycle.read().rstrip()
    f_duty_cycle.close()

    return cur_duty_cycle


def _set_pwm_duty_cycle(pin_name, duty_cycle_ns):
    if not duty_cycle_ns:
        f_duty_cycle = open(pin_info[pin_name].pwm_duty_cycle, 'r')
        cur_duty_cycle = f_duty_cycle.read().rstrip()
        f_duty_cycle.close()
        if cur_duty_cycle == '0':
            return

    f_duty_cycle = open(pin_info[pin_name].pwm_duty_cycle, 'w')
    f_duty_cycle.write(str(duty_cycle_ns))
    f_duty_cycle.close()


def _enable_pwm(pin_name):
    if _get_pwm_duty_cycle(pin_name) == '0':
        return
    f_enable = open(pin_info[pin_name].pwm_enable, 'w')
    f_enable.write('1')
    f_enable.close()


def _disable_pwm(pin_name):
    if _get_pwm_duty_cycle(pin_name) == '0':
        return
    f_enable = open(pin_info[pin_name].pwm_enable, 'w')
    f_enable.write('0')
    f_enable.close()


class PWM(object):
    def __init__(self, pin_name, frequency_hz):
        if not pin_mode:
            raise RuntimeError("No channel mode set")
        if pin_info.__contains__(pin_name):
            raise RuntimeError("This channel is in use")
        try:
            pin_info[pin_name] = PinPro(pin_name)
        except Exception as exc:
            raise ValueError("This channel name was not found in this mode")

        if pin_info[pin_name].pwm_name is None:
            raise RuntimeError("The channel is not support PWM mode")
        if frequency_hz <= 0:
            raise ValueError("Wrong frequency value")

        self.pin_name = pin_name
        self.cur_frequency_hz = -1
        self.cur_duty_cycle = 0.0

        _export_pwm(self.pin_name)
        self.is_started = False
        self._configure(frequency_hz, 0.0)

    def __del__(self):
        if not pin_info.__contains__(self.pin_name):
            return
        if pin_info[self.pin_name].pin_status != HARD_PWM:
            return
        self.stop()
        _unexport_pwm(self.pin_name)

    def start(self, duty_cycle_percent):
        if duty_cycle_percent < 0 or duty_cycle_percent > 100:
            raise ValueError("Wrong duty cycle value")

        _enable_pwm(self.pin_name)
        self._configure(self.cur_frequency_hz, duty_cycle_percent)
        self.is_started = True

    def ChangeFrequency(self, frequency_hz):
        if frequency_hz <= 0:
            raise ValueError("Wrong frequency value")

        self._configure(frequency_hz, self.cur_duty_cycle)

    def ChangeDutyCycle(self, duty_cycle_percent):
        if duty_cycle_percent < 0 or duty_cycle_percent > 100:
            raise ValueError("Wrong duty cycle value")

        self._configure(self.cur_frequency_hz, duty_cycle_percent)

    def stop(self):
        _disable_pwm(self.pin_name)
        self.is_started = False

    def _configure(self, frequency_hz, duty_cycle_percent):
        # Make sure to set the period first before setting the duty cycle
        is_fre_changed = False
        if self.cur_frequency_hz != frequency_hz:
            is_fre_changed = True

        if is_fre_changed:
            _disable_pwm(self.pin_name)
            self.is_started = False
            _set_pwm_duty_cycle(self.pin_name, 0)
            self.period_ns = int(1000000000.0 / frequency_hz)
            _set_pwm_period(self.pin_name, self.period_ns)
            self.cur_frequency_hz = frequency_hz

        self.duty_cycle_ns = int(int(1000000000.0 / self.cur_frequency_hz) *
                                 (duty_cycle_percent / 100.0))
        _set_pwm_duty_cycle(self.pin_name, self.duty_cycle_ns)
        self.cur_duty_cycle = duty_cycle_percent

        if is_fre_changed:
            _enable_pwm(self.pin_name)
            self.is_started = True
            is_fre_changed = False
