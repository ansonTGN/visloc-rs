set pagination off
set confirm off
set verbose off
set print pretty off
set print elements 0
set disable-randomization on
set height 0
set width 0

file /root/visloc-basalt-clean-m7cr-20260823/build/core-relwithdebinfo/basalt_vio
set args --dataset-path /mnt/e/datasets/euroc_mav/machine_hall/MH_01_easy --cam-calib /root/visloc-basalt-clean-m7cr-20260823/data/euroc_ds_calib.json --dataset-type euroc --config-path /root/visloc-basalt-clean-m7cr-20260823/data/euroc_config.json --marg-data /marg_data_m7he_local_hb --show-gui 0 --save-trajectory euroc --num-threads 1 --max-frames 7
start

python
import gdb
import struct

inf = gdb.selected_inferior()
base = None
for line in open('/proc/%d/maps' % inf.pid):
    fields = line.split()
    if len(fields) >= 6 and fields[2] == '00000000' and fields[-1].endswith('/libbasalt.so'):
        base = int(fields[0].split('-')[0], 16)
        break
if base is None:
    raise RuntimeError('libbasalt.so base unavailable')

def mem(addr, n):
    return bytes(inf.read_memory(int(addr), int(n)))

def u64(addr):
    return struct.unpack('<Q', mem(addr, 8))[0]

def i64(addr):
    return struct.unpack('<q', mem(addr, 8))[0]

def dump_matrix(label, obj):
    try:
        data, rows, cols = u64(obj), i64(obj + 8), i64(obj + 16)
        print('M7IM15_PROBE %s obj=0x%x data=0x%x shape=%dx%d' % (label, obj, data, rows, cols))
        if data and 0 < rows <= 100 and 0 < cols <= 100:
            words = struct.unpack('<%dI' % (rows * cols), mem(data, 4 * rows * cols))
            print('M7IM15_PROBE %s_bits=%s' % (label, ','.join('%08x' % word for word in words[:min(8, rows * cols)])))
    except Exception as exc:
        print('M7IM15_PROBE %s_error=%s' % (label, exc))

class DenseEntry(gdb.Breakpoint):
    def __init__(self):
        super().__init__('*0x%x' % (base + 0x2cfab0), internal=False)
    def stop(self):
        print('M7IM15_PROBE_DENSE_HIT pc=%s rdi=%s rsi=%s rdx=%s' % tuple(gdb.parse_and_eval('$' + reg) for reg in ('pc', 'rdi', 'rsi', 'rdx')))
        dump_matrix('H_arg', int(gdb.parse_and_eval('$rsi')))
        dump_matrix('b_arg', int(gdb.parse_and_eval('$rdx')))
        gdb.execute('info args')
        gdb.execute('bt 8')
        gdb.execute('quit')
        return False

DenseEntry()
end
continue
