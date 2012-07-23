#!/usr/bin/env python

import fcntl, struct, time, array

#
# There are two steps to creating a rumble effect
# 1/ describe the effect and give it to the driver using an
#    ioctl.
#   There a 3 paramaters:
#     strength:  from 0 to 0xffff - this code takes a value from 0 to
#                                   1 and scales it
#     duration: milliseconds
#     delay until start: milliseconds.
#
# 2/ write a request to play a specific effect.
#
# It is possible to have multiple effects active.  If they have
# different delays they will start at different times.
# This demo shows combining 3 non-overlapping effects to make
# a simple vibration pattern
#
# An effect is created with f.new_vibe(strength, duration, delay)
# That effect can then be started with 'play' and stopped with 'stop'.

class Vibra:
    def __init__(self, file = "/dev/input/rumble"):
        self.f = open(file, "r+")

    def new_vibe(self, strength, length, delay):
        # strength is from 0 to 1
        # length and delay are in millisecs
        # this is 'struct ff_effect' from "linux/input.h"
        effect = struct.pack('HhHHHHHxxHH',
                             0x50, -1, 0, # FF_RUMBLE, id, direction
                             0, 0,        # trigger (button interval)
                             length, delay,
                             int(strength * 0xFFFF), 0)
        a = array.array('h', effect)
        # this is EVIOCSFF _IOC(_IOC_WRITE, 'E', 0x80, sizeof(struct ff_effect))
        fcntl.ioctl(self.f, 0x402c4580, a, True)
        id = a[1]
        # this is 'struct input_event': sec, nsec, type, code, value
        ev_play = struct.pack('LLHHi', 0, 0, 0x15, id, 1)
        ev_stop = struct.pack('LLHHi', 0, 0, 0x15, id, 0)
        return (ev_play, ev_stop)

    def play(self, ev):
        p,s = ev;
        self.f.write(p)
        self.f.flush()

    def stop(self, ev):
        p,s = ev;
        self.f.write(s)
        self.f.flush()



f = Vibra("/dev/input/rumble")

# rumble for 300ms, pause for 100ms, rumble for 300ms, pause for 200ms
# then half-speed rumble for 600ms
p1 = f.new_vibe(1, 300, 0)
p2 = f.new_vibe(1, 300,400)
p3 = f.new_vibe(0.5, 600, 900)
f.play(p1)
f.play(p2)
f.play(p3)

time.sleep(1.5)
