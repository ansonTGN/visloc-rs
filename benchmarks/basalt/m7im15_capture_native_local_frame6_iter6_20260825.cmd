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
import json
import os
import struct

OUT = '/mnt/c/Users/rsasa/Workspace/visloc-rs/target/m7im15_native_local_capture_frame6_iter6_20260825.json'
inf = gdb.selected_inferior()
base = None
for line in open('/proc/%d/maps' % inf.pid):
    fields = line.split()
    if len(fields) >= 6 and fields[2] == '00000000' and fields[-1].endswith('/libbasalt.so'):
        base = int(fields[0].split('-')[0], 16)
        break
if base is None:
    raise RuntimeError('libbasalt.so base unavailable')

def ev(expr):
    return gdb.parse_and_eval(expr)
def mem(addr, n):
    return bytes(inf.read_memory(int(addr), int(n)))
def u64(addr):
    return struct.unpack('<Q', mem(addr, 8))[0]
def i64(addr):
    return struct.unpack('<q', mem(addr, 8))[0]
def matrix_desc(obj):
    data, rows, cols = u64(obj), i64(obj + 8), i64(obj + 16)
    if data == 0 or rows <= 0 or rows > 1000 or cols <= 0 or cols > 1000:
        raise RuntimeError('bad matrix descriptor 0x%x data=0x%x shape=%dx%d' % (obj, data, rows, cols))
    return data, rows, cols
def vector_desc(obj):
    data, rows = u64(obj), i64(obj + 8)
    if data == 0 or rows <= 0 or rows > 1000:
        raise RuntimeError('bad vector descriptor 0x%x data=0x%x rows=%d' % (obj, data, rows))
    return data, rows, 1
def f32_bits(data, count):
    return ['%08x' % value for value in struct.unpack('<%dI' % count, mem(data, 4 * count))]
def write_out():
    payload = {
        'schema': 'basalt.m7im15.native_local_capture.v1',
        'binary': 'clean m7cr pinned basalt_vio',
        'commit': '0f3b2b52c807f70ff4e2973ce253c73329eea7bc',
        'num_threads': 1,
        'max_frames': 7,
        'breakpoints': {
            'get_dense_entry': '0x2cfab0',
            'imu_add_dense_entry': '0x2ce4f0',
            'after_local_gemm_and_gemv': '0x2ce8a2',
        },
        'capture_contract': {
            'Jp': 'ImuBlock<float>::Jp before add_dense_H_b, Eigen column-major 15x30',
            'r': 'ImuBlock<float>::r before add_dense_H_b, Eigen column-major vector 15',
            'H': 'temporary Jp.transpose()*Jp at after-local-GEMM/GEMV boundary, Eigen column-major 30x30',
            'b': 'temporary Jp.transpose()*r at after-local-GEMM/GEMV boundary, Eigen column-major vector 30',
        },
        'records': records,
    }
    with open(OUT, 'w', encoding='utf-8') as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)

records = []
dense_call = 0
dense_info = {}
pending = {}
seen_by_start = {}

class GetDense(gdb.Breakpoint):
    def __init__(self):
        super().__init__('*0x%x' % (base + 0x2cfab0), internal=False)
    def stop(self):
        global dense_call
        dense_call += 1
        parent = int(ev('$rdi'))
        try:
            idx_off = int(ev('&((basalt::LinearizationAbsQR<float,6>*)0)->landmark_block_idx'))
            begin = u64(parent + idx_off)
            end = u64(parent + idx_off + 8)
            landmarks = (end - begin) // 8
        except Exception:
            landmarks = None
        dense_info[int(gdb.selected_thread().global_num)] = {
            'dense_call': dense_call,
            'landmarks': landmarks,
        }
        print('M7IM15_NATIVE_GET_DENSE call=%d landmarks=%s' % (dense_call, landmarks))
        return False

class ImuEntry(gdb.Breakpoint):
    def __init__(self):
        super().__init__('*0x%x' % (base + 0x2ce4f0), internal=False)
    def stop(self):
        thread = int(gdb.selected_thread().global_num)
        this = int(ev('$rdi'))
        info = dense_info.get(thread, {'dense_call': None, 'landmarks': None})
        imu_ptr = u64(this + 0x38)
        try:
            start_ns = int(ev('((basalt::ImuBlock<float>*)%d)->imu_meas->start_t_ns_' % this))
            dt_ns = int(ev('((basalt::ImuBlock<float>*)%d)->imu_meas->delta_state_.t_ns' % this))
        except Exception as exc:
            print('M7IM15_NATIVE_TIME_ERROR', exc)
            start_ns, dt_ns = None, None
        jp_data, jp_rows, jp_cols = matrix_desc(this + 0x10)
        r_data, r_rows, r_cols = vector_desc(this + 0x28)
        ordinal = seen_by_start.get(start_ns, 0)
        seen_by_start[start_ns] = ordinal + 1
        pending[thread] = {
            'this': this,
            'imu_ptr': imu_ptr,
            'start_ns': start_ns,
            'dt_ns': dt_ns,
            'dense_call': info.get('dense_call'),
            'landmarks': info.get('landmarks'),
            'link_ordinal_for_start': ordinal,
            'jp': f32_bits(jp_data, jp_rows * jp_cols),
            'jp_shape': [jp_rows, jp_cols],
            'r': f32_bits(r_data, r_rows * r_cols),
            'r_shape': [r_rows, r_cols],
        }
        return False

class AfterLocalProduct(gdb.Breakpoint):
    def __init__(self):
        super().__init__('*0x%x' % (base + 0x2ce8a2), internal=False)
    def stop(self):
        thread = int(gdb.selected_thread().global_num)
        p = pending.pop(thread, None)
        if p is None:
            print('M7IM15_NATIVE_PRODUCT_WITHOUT_ENTRY')
            return False
        rbp = int(ev('$rbp'))
        h_data = u64(rbp - 0x120)
        h_rows = i64(rbp - 0x118)
        h_cols = i64(rbp - 0x110)
        b_data = u64(rbp - 0x130)
        b_rows = i64(rbp - 0x128)
        if h_rows != 30 or h_cols != 30 or b_rows != 30:
            raise RuntimeError('unexpected local product shape H=%dx%d b=%d' % (h_rows, h_cols, b_rows))
        p.update({
            'H': f32_bits(h_data, h_rows * h_cols),
            'H_shape': [h_rows, h_cols],
            'b': f32_bits(b_data, b_rows),
            'b_shape': [b_rows, 1],
            'capture_order': len(records),
        })
        records.append(p)
        write_out()
        print('M7IM15_NATIVE_LOCAL order=%d dense=%s landmarks=%s start=%s ordinal=%d' %
              (p['capture_order'], p['dense_call'], p['landmarks'], p['start_ns'], p['link_ordinal_for_start']))
        return False

GetDense()
ImuEntry()
AfterLocalProduct()
end
continue
end
continue
