#!/usr/bin/env python3
"""No hardware: regression tests for queued GPIO quadrature decoding."""
import struct
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_ws/src/robowheels'))
from robowheels.encoder_events import EncoderEvents

def decoder():
    x=EncoderEvents.__new__(EncoderEvents)
    x.indices={1:0,0:1,9:2,3:3};x.states=[0,0];x.counts=[0,0];x.invalid=[0,0]
    x.last_seq=0;x.dropped=0;x.line_sequences={};x.reordered=0
    return x

def pack(events):return b''.join(struct.pack('=QIIII6I',*e,*([0]*6)) for e in events)
def main():
    x=decoder()
    cycle=[(10,1,0,1,1),(20,1,1,3,1),(30,2,0,2,2),(40,2,1,4,2)]
    x.consume(pack([cycle[i] for i in (0,2,1,3)]))
    assert x.counts==[4,0] and x.invalid==[0,0] and x.dropped==0
    reverse=[(50,1,1,5,3),(60,1,0,6,3),(70,2,1,7,4),(80,2,0,8,4)]
    x.consume(pack(reverse));assert x.counts==[0,0] and x.dropped==0
    x.consume(pack([(90,1,0,10,6)]));assert x.dropped==1
    try:x.consume(b'bad');raise AssertionError('Truncated event accepted')
    except RuntimeError:pass
    print('Passed GPIO ABI sizing, timestamp/SMP ordering, forward/reverse decoding, per-line loss and malformed-event checks')
if __name__=='__main__':main()
