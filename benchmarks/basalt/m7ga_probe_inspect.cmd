set pagination off
set confirm off
set verbose off
set breakpoint pending on
set print pretty off
set print object off
set print elements 0
set print frame-arguments none
set print demangle off
set disable-randomization on
set height 0
set width 0

file /root/visloc-basalt-clean-m7cr-20260823/build/core-relwithdebinfo/basalt_vio
set args --dataset-path /mnt/e/datasets/euroc_mav/machine_hall/MH_01_easy --cam-calib /root/visloc-basalt-clean-m7cr-20260823/data/euroc_ds_calib.json --dataset-type euroc --show-gui 0 --config-path /root/visloc-basalt-clean-m7cr-20260823/data/euroc_config.json --marg-data /marg_data_m7ct --save-trajectory euroc --num-threads 4 --max-frames 5

start
python
import gdb

inf = gdb.selected_inferior()
base = None
with open('/proc/%d/maps' % inf.pid, 'r') as f:
    for line in f:
        fields = line.split()
        if len(fields) >= 6 and fields[2] == '00000000' and fields[-1].endswith('/libbasalt.so'):
            base = int(fields[0].split('-')[0], 16)
            break
if base is None:
    raise RuntimeError('libbasalt mapping unavailable')

class Inspect(gdb.Breakpoint):
    def __init__(self):
        super().__init__('*0x%x' % (base + 0x275e58), internal=False)
    def stop(self):
        print('M7GA_HIT base=0x%x pc=%s thread=%s' % (base, gdb.parse_and_eval('$pc'), gdb.selected_thread().global_num))
        gdb.execute('bt 8')
        gdb.execute('info args')
        gdb.execute('info locals')
        print('M7GA_REGS r14=%s rbp=%s r10=%s rdx=%s rsi=%s rcx=%s' % tuple(gdb.parse_and_eval('$'+r) for r in ('r14','rbp','r10','rdx','rsi','rcx')))
        gdb.execute('x/32gx $rdi')
        gdb.execute('set $m7this = (unsigned long)this')
        gdb.execute('printf "M7GA_THIS=%p\\n", $m7this')
        gdb.execute('x/96gx $m7this')
        for expr in ('this->storage.data()', 'this->storage.rows()', 'this->storage.cols()', 'this->padding_idx', 'this->padding_size', 'this->lm_idx', 'this->res_idx', 'this->num_cols', 'this->num_rows', 'this->pose_lin_vec.size()', 'i'):
            try:
                print('M7GA_EXPR %s = %s' % (expr, gdb.parse_and_eval(expr)))
            except Exception as exc:
                print('M7GA_EXPR_ERROR %s: %s' % (expr, exc))
        gdb.execute('x/64wx $rsi')
        gdb.execute('x/32wx $rbp-0x120')
        gdb.execute('quit')
        return False

Inspect()
end
continue
