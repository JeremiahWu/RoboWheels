"""X5 quadrature encoders using Linux GPIO v2 queued edge events.

No extra Python dependencies. ABI follows /usr/include/linux/gpio.h.
Events have kernel timestamps and sequence numbers; queue loss is reported.
"""
import ctypes as C
import fcntl
import os
import select
import struct
import time
from pathlib import Path

class _Config(C.Structure):
    _fields_=[('flags',C.c_uint64),('num_attrs',C.c_uint32),('padding',C.c_uint32*5),('attrs',C.c_uint64*30)]
class _Request(C.Structure):
    _fields_=[('offsets',C.c_uint32*64),('consumer',C.c_char*32),('config',_Config),('num_lines',C.c_uint32),('event_buffer_size',C.c_uint32),('padding',C.c_uint32*5),('fd',C.c_int32)]
assert C.sizeof(_Config)==272 and C.sizeof(_Request)==592

def _ioc(direction, number, size):
    return (direction<<30)|(size<<16)|(0xB4<<8)|number

class EncoderEvents:
    OFFSETS={11:1,13:0,15:9,16:3}
    EVENT=struct.Struct('=QIIII6I')
    # Canonical sign matches MotorDriver._QUAD_TRANS (opposite raw monitor).
    LUT=(0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0)
    def __init__(self,pins):
        self.fd=None;self.last_seq=0;self.dropped=0;self.counts=[0,0];self.invalid=[0,0]
        self.line_sequences={};self.pending=[];self.reordered=0
        offsets=[self.OFFSETS[p] for p in pins]
        self.indices={offset:i for i,offset in enumerate(offsets)}
        chip=None
        for path in Path('/dev').glob('gpiochip*'):
            candidate=os.open(path,os.O_RDONLY|os.O_CLOEXEC)
            info=bytearray(68)
            try:fcntl.ioctl(candidate,_ioc(2,1,68),info,True)
            except BaseException:os.close(candidate);raise
            label=bytes(info[32:64]).split(b'\0')[0]
            if label==b'34120000.gpio':chip=candidate;break
            os.close(candidate)
        if chip is None:raise RuntimeError('X5 encoder GPIO controller not found')
        try:
            req=_Request();req.consumer=b'robowheels-encoders';req.num_lines=4;req.event_buffer_size=16384
            req.config.flags=(1<<2)|(1<<4)|(1<<5)  # input + both edges
            for i,offset in enumerate(offsets):req.offsets[i]=offset
            data=bytearray(bytes(req));fcntl.ioctl(chip,_ioc(3,7,C.sizeof(req)),data,True)
            self.fd=_Request.from_buffer(data).fd
            os.set_blocking(self.fd,False)
            values=bytearray(struct.pack('=QQ',0,15));fcntl.ioctl(self.fd,_ioc(3,14,16),values,True)
            bits,_=struct.unpack('=QQ',values)
            self.states=[((bits>>(2*i))&1)*2+((bits>>(2*i+1))&1) for i in (0,1)]
            self.poller=select.poll();self.poller.register(self.fd,select.POLLIN)
        except BaseException:
            self.close();raise
        finally:os.close(chip)
    def decode(self,events):
        for event in events:
            _,kind,offset,seq,line_seq,*_=event
            # Global seqno allocation and FIFO enqueue can race across CPUs.
            # Loss detection must use each line's own sequence, not subtract
            # adjacent global numbers (which can produce a false 2**32 gap).
            previous=self.line_sequences.get(offset,0)
            gap=(line_seq-previous)&0xffffffff
            if 1 < gap < 0x80000000:self.dropped+=gap-1
            elif gap==0 or gap>=0x80000000:self.reordered+=1
            if 0 < gap < 0x80000000:self.line_sequences[offset]=line_seq
            self.last_seq=seq
            index=self.indices[offset];wheel=index//2;bit=1<<(1-index%2)
            old=self.states[wheel]
            if kind not in (1,2):raise RuntimeError('Invalid GPIO edge ID')
            new=(old|bit) if kind==1 else (old&~bit)
            if new==old:self.invalid[wheel]+=1
            else:self.counts[wheel]+=self.LUT[old*4+new]
            self.states[wheel]=new
    def consume(self,data):
        if len(data)%self.EVENT.size:raise RuntimeError('Truncated GPIO event')
        self.decode(sorted(self.EVENT.iter_unpack(data),key=lambda e:(e[0],e[2],e[4])))
    def read(self,timeout=.005):
        if self.poller.poll(round(timeout*1000)):
            while True:
                try:data=os.read(self.fd,self.EVENT.size*512)
                except BlockingIOError:break
                if not data:raise RuntimeError('GPIO event stream closed')
                if len(data)%self.EVENT.size:raise RuntimeError('Truncated GPIO event')
                self.pending.extend(self.EVENT.iter_unpack(data))
        # Keep a small reordering window across read batches. Event timestamps
        # are CLOCK_MONOTONIC nanoseconds, matching time.monotonic_ns().
        self.pending.sort(key=lambda e:(e[0],e[2],e[4]))
        cutoff=time.monotonic_ns()-5_000_000
        split=0
        while split<len(self.pending) and self.pending[split][0]<=cutoff:split+=1
        self.decode(self.pending[:split]);del self.pending[:split]
        return list(self.counts),list(self.invalid),self.dropped
    def close(self):
        if self.fd is not None:os.close(self.fd);self.fd=None
