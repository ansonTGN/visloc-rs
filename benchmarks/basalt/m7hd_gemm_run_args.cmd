set pagination off
set confirm off
set disable-randomization on
set auto-solib-add off
set debuginfod enabled off
file /root/visloc-basalt-clean-m7cr-20260823/build/core-relwithdebinfo/basalt_vio
set args --dataset-path /mnt/e/datasets/euroc_mav/machine_hall/MH_01_easy --cam-calib /root/visloc-basalt-clean-m7cr-20260823/data/euroc_ds_calib.json --dataset-type euroc --config-path /root/visloc-basalt-clean-m7cr-20260823/data/euroc_config.json --marg-data /marg_data_m7hd_gemm_run_args --show-gui 0 --save-trajectory euroc --num-threads 1 --max-frames 5
start
set $n_run = 0
set $n_kernel = 0
set $n_g = 0
x/8i 0x7ffff728ea30
break *0x7ffff728ea30
commands
 silent
 set $n_kernel = $n_kernel + 1
 if $n_kernel <= 12
  printf "M7HD KERNEL n=%d rdi=%p rsi=%p rdx=%p rcx=%ld r8=%ld r9=%ld stack10=%ld stack18=%ld stack20=%ld stack28=%ld\n", $n_kernel, $rdi, $rsi, $rdx, $rcx, $r8, $r9, *(long*)($rsp+0x10), *(long*)($rsp+0x18), *(long*)($rsp+0x20), *(long*)($rsp+0x28)
 end
 continue
end
break *0x7ffff72c1710
commands
 silent
 set $n_run = $n_run + 1
 if $n_run <= 12
  printf "M7HD RUN n=%d rdi=%p rsi=%p rdx=%p rcx=%ld r8=%ld r9=%ld stack10=%ld stack18=%ld stack20=%ld stack28=%ld\n", $n_run, $rdi, $rsi, $rdx, $rcx, $r8, $r9, *(long*)($rsp+0x10), *(long*)($rsp+0x18), *(long*)($rsp+0x20), *(long*)($rsp+0x28)
 end
 continue
end
break *0x7ffff72c2a68
commands
 silent
 set $n_g = $n_g + 1
 if $n_g >= 2
  quit
 end
 continue
end
continue
