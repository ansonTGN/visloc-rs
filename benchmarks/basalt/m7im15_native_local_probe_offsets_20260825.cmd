set pagination off
set confirm off
set verbose off
set print pretty off
set print elements 0
set disable-randomization on
file /root/visloc-basalt-clean-m7cr-20260823/build/core-relwithdebinfo/basalt_vio
set args --dataset-path /mnt/e/datasets/euroc_mav/machine_hall/MH_01_easy --cam-calib /root/visloc-basalt-clean-m7cr-20260823/data/euroc_ds_calib.json --dataset-type euroc --config-path /root/visloc-basalt-clean-m7cr-20260823/data/euroc_config.json --marg-data /marg_data_m7he_local_hb --show-gui 0 --save-trajectory euroc --num-threads 1 --max-frames 7
start
python
import gdb
import os
import struct
inf = gdb.selected_inferior()
base = None
with open('/proc/%d/maps' % inf.pid) as stream:
    for line in stream:
        fields = line.split()
        if len(fields) >= 6 and fields[2] == '00000000' and fields[-1].endswith('/libbasalt.so'):
            base = int(fields[0].split('-')[0], 16)
            break
def ev(expr):
    return gdb.parse_and_eval(expr)
class FirstImu(gdb.Breakpoint):
    def __init__(self):
        super().__init__('*0x%x' % (base + 0x2ce4f0), internal=False)
    def stop(self):
        this = int(ev('$rdi'))
        print('M7IM15_FIRST_THIS', hex(this))
        for expr in [
            '((basalt::ImuBlock<float>*)%d)->imu_meas->get_start_t_ns()' % this,
            '((basalt::ImuBlock<float>*)%d)->imu_meas->get_dt_ns()' % this,
            '((basalt::ImuBlock<float>*)%d)->imu_meas->start_t_ns_' % this,
            '((basalt::ImuBlock<float>*)%d)->imu_meas->delta_state_.t_ns' % this,
            '((basalt::ImuBlock<float>*)%d)->frame_ids[0]' % this,
            '((basalt::ImuBlock<float>*)%d)->frame_ids[1]' % this,
        ]:
            try:
                print('M7IM15_EXPR', expr, ev(expr))
            except Exception as exc:
                print('M7IM15_EXPR_ERROR', expr, exc)
        gdb.execute('quit')
        return False
if base is not None:
    FirstImu()
for typ, fields in [
    ('basalt::ImuBlock<float>', ['imu_meas', 'imu_lin_data', 'aom', 'Jp', 'r']),
    ('basalt::IntegratedImuMeasurement<float>', ['dt_ns', 'start_t_ns']),
]:
    print('M7IM15_TYPE', typ)
    for field in fields:
        try:
            print('M7IM15_OFFSET', typ, field, gdb.parse_and_eval("&((%s*)0)->%s" % (typ, field)))
        except Exception as exc:
            print('M7IM15_OFFSET_ERROR', typ, field, exc)
    try:
        print('M7IM15_SIZE', typ, gdb.parse_and_eval('sizeof(%s)' % typ))
    except Exception as exc:
        print('M7IM15_SIZE_ERROR', typ, exc)
end
continue
