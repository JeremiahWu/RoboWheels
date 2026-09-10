# Python2: thread, Python3: _thread

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

try:
    import thread
except:
    import _thread as thread

from select import epoll, EPOLLIN, EPOLLET, EPOLLPRI
from datetime import datetime

try:
    InterruptedError = InterruptedError
except:
    InterruptedError = IOError


EVENT = "EVENT"
INTERRUPT = "INTERRUPT"
event_info = {}
thread_running = False
mutex = thread.allocate_lock()

class EventPro:
    def __init__(self, event, edge=None, bouncetime=None):
        self.event = event
        self.edge = edge
        self.bouncetime = bouncetime
        self.fd_epoll = None
        self.lastcall = 0
        self.is_occurred = False
        self.init_flag = True
        self.gpio_value = None
        self.fd_gpio_value = None
        self.callbacks = []

    def __del__(self):
        del self.callbacks
        if self.fd_gpio_value:
            self.fd_gpio_value.close()


def _is_event_added(pin_name):
    if not event_info.__contains__(pin_name):
        return "NO_EDGE"
    return event_info[pin_name].edge


def _add_event_detect(pin_info, pin_name, edge, bouncetime):
    if event_info.__contains__(pin_name):
        if event_info[pin_name].event == EVENT:
            return -1
    else:
        event_info[pin_name] = EventPro(INTERRUPT, edge=edge, bouncetime=bouncetime)

    _set_edge(pin_info[pin_name].gpio_edge, edge)

    mutex.acquire()
    event_info[pin_name].fd_gpio_value = open(pin_info[pin_name].gpio_value, 'r')
    mutex.release()

    if event_info[pin_name].fd_epoll is None:
        event_info[pin_name].fd_epoll = epoll()
        if event_info[pin_name].fd_epoll is None:
            _del_event(pin_info, pin_name)
            return -2

    try:
        event_info[pin_name].fd_epoll.register(event_info[pin_name].fd_gpio_value,
            EPOLLIN | EPOLLET | EPOLLPRI)
    except IOError:
        print("IOError occurs when gpio epoll blocks")
        _del_event(pin_info, pin_name)
        return -3

    try:
        thread.start_new_thread(_poll_thread, (pin_info, pin_name))
    except RuntimeError:
        _del_event(pin_info, pin_name)
        return -4
    return 0


def _del_event(pin_info, pin_name):
    global event_info

    if not event_info.__contains__(pin_name):
        return

    if event_info[pin_name].fd_epoll:
        try:
            event_info[pin_name].fd_epoll.unregister(event_info[pin_name].fd_gpio_value)
        except Exception as exc:
            pass

    _set_edge(pin_info[pin_name].gpio_edge, 'none')

    event_info[pin_name].fd_epoll.close()
    event_info[pin_name].fd_epoll = None
    event_info[pin_name].fd_gpio_value.close()
    event_info[pin_name].fd_gpio_value = None

    mutex.acquire()
    del event_info[pin_name]
    mutex.release()


def _add_callback(pin_name, callback):
    event_info[pin_name].callbacks.append(callback)


def _is_event_occurred(pin_name):
    retval = False
    if event_info.__contains__(pin_name):
        mutex.acquire()
        if event_info[pin_name].is_occurred:
            event_info[pin_name].is_occurred = False
            retval = True
        mutex.release()
    return retval


def _set_edge(edge_path, edge):
    f_edge = open(edge_path, 'w')
    f_edge.write(edge.lower())
    f_edge.close()


def _exe_callback(pin_name):
    for callback in event_info[pin_name].callbacks:
        callback()


def _poll_thread(pin_info, pin_name):
    global event_info, thread_running
    res = None

    thread_running = True
    while thread_running:
        try:
            res = event_info[pin_name].fd_epoll.poll(maxevents=1)

            fileno = res[0][0]
            fd = event_info[pin_name].fd_gpio_value
            if fd is None or fd.closed:
                continue

            fd = event_info[pin_name].fd_gpio_value
            fd.seek(0)
            gpio_value = fd.read().rstrip()
            if len(gpio_value) != 1:
                thread_running = False
                thread.exit()

            if event_info[pin_name].init_flag:
                mutex.acquire()
                event_info[pin_name].init_flag = False
                event_info[pin_name].gpio_value = gpio_value
                mutex.release()
                continue
            else:
                if ((event_info[pin_name].edge == "RISING" and int(gpio_value) > int(event_info[pin_name].gpio_value)) or
                    (event_info[pin_name].edge == "FALLING" and int(gpio_value) < int(event_info[pin_name].gpio_value)) or
                    (event_info[pin_name].edge == "BOTH" and int(gpio_value) != int(event_info[pin_name].gpio_value))):

                    # debounce the input event for the specified bouncetimez
                    time = datetime.now()
                    time = time.second * 1E6 + time.microsecond
                    if (event_info[pin_name].bouncetime is None or
                            (time - event_info[pin_name].lastcall >
                            event_info[pin_name].bouncetime * 1000) or
                            (event_info[pin_name].lastcall == 0) or event_info[pin_name].lastcall > time):
                        mutex.acquire()
                        event_info[pin_name].lastcall = time
                        event_info[pin_name].event_occurred = True
                        mutex.release()
                        _exe_callback(pin_name)

                mutex.acquire()
                event_info[pin_name].gpio_value = gpio_value
                mutex.release()
        except InterruptedError:
            continue
        except AttributeError as exc :
            print('AttributeError occured, ', exc)
            break
    thread.exit()


def _add_event_block(pin_info, pin_name, edge, bouncetime, timeout):
    global event_info
    finished = False
    res = None
    init_gpio_value = None

    if event_info.__contains__(pin_name):
        if event_info[pin_name].event == INTERRUPT:
            return -1
    else:
        event_info[pin_name] = EventPro(EVENT, edge=edge, bouncetime=bouncetime)

    timeout = (float(timeout) / 1000) if timeout else -1
    _set_edge(pin_info[pin_name].gpio_edge, edge)

    mutex.acquire()
    event_info[pin_name].fd_gpio_value = open(pin_info[pin_name].gpio_value, 'r')
    mutex.release()

    if event_info[pin_name].fd_epoll is None:
        event_info[pin_name].fd_epoll = epoll()
        if event_info[pin_name].fd_epoll is None:
            _del_event(pin_info, pin_name)
            return -2

    try:
        event_info[pin_name].fd_epoll.register(event_info[pin_name].fd_gpio_value,
            EPOLLIN | EPOLLET | EPOLLPRI)
    except IOError:
        print("IOError occurs when gpio epoll blocks")
        _del_event(pin_info, pin_name)
        return -3

    while not finished:
        try:
            res = event_info[pin_name].fd_epoll.poll(timeout, maxevents=1)
        except InterruptedError:
            continue

        fd = event_info[pin_name].fd_gpio_value
        fd.seek(0)
        gpio_value = fd.read().rstrip()
        if event_info[pin_name].init_flag:
            event_info[pin_name].init_flag = False
            init_gpio_value = gpio_value
            continue
        else:
            if ((edge == "RISING" and int(gpio_value) > int(init_gpio_value)) or
                (edge == "FALLING" and int(gpio_value) < int(init_gpio_value)) or
                (edge == "BOTH" and int(gpio_value) != int(init_gpio_value))):
                # debounce the input event for the specified bouncetime
                time = datetime.now()
                time = time.second * 1E6 + time.microsecond
                if (event_info[pin_name].bouncetime is None or
                        (time - event_info[pin_name].lastcall >
                            event_info[pin_name].bouncetime * 1000) or
                        (event_info[pin_name].lastcall == 0) or
                        (event_info[pin_name].lastcall > time)):
                    mutex.acquire()
                    event_info[pin_name].lastcall = time
                    mutex.release()
                    finished = True
            init_gpio_value = gpio_value
    if res:
        fileno = res[0][0]
        if fileno != fd.fileno():
            _del_event(pin_info, pin_name)
            print("File handle not found")
            return -4
        else:
            mutex.acquire()
            fd.seek(0)
            value_str = fd.read().rstrip()
            mutex.release()
            if len(value_str) != 1:
                _del_event(pin_info, pin_name)
                print("Value node length is not 1 ")
                return -5

    _del_event(pin_info, pin_name)

    return int(res != [])


def _event_cleanup(pin_info, pin_name):
    global event_info, thread_running

    thread_running = False
    if event_info.__contains__(pin_name):
        _del_event(pin_info, pin_name)
