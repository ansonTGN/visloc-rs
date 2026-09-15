set pagination off
set confirm off
set disable-randomization on
set auto-solib-add off
set debuginfod enabled off
file /root/visloc-basalt-clean-m7cr-20260823/build/core-relwithdebinfo/basalt_vio
set args --dataset-path /mnt/e/datasets/euroc_mav/machine_hall/MH_01_easy --cam-calib /root/visloc-basalt-clean-m7cr-20260823/data/euroc_ds_calib.json --dataset-type euroc --config-path /root/visloc-basalt-clean-m7cr-20260823/data/euroc_config.json --marg-data /marg_data_m7hd_tail_gdb --show-gui 0 --save-trajectory euroc --num-threads 1 --max-frames 2
start
x/8i 0x7ffff7290350
break *0x7ffff7290350
commands
 silent
 printf "M7HD TAIL pc=%p rax=%p rdx=%p rdi=%p rsi=%p r8=%ld r9=%ld r10=%p r11=%p r12=%p r13=%p r14=%p r15=%p\n", $pc,$rax,$rdx,$rdi,$rsi,$r8,$r9,$r10,$r11,$r12,$r13,$r14,$r15
 x/64wx $rdx
 x/64wx $rax
 info registers xmm0 xmm1 xmm2 xmm3 xmm4 xmm5 xmm6 xmm7 xmm8 xmm9 xmm10 xmm11 xmm12 xmm13 xmm14 xmm15
 quit
end
continue
