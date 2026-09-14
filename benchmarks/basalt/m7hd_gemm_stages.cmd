set pagination off
set confirm off
set disable-randomization on
set auto-solib-add off
set debuginfod enabled off
file /root/visloc-basalt-clean-m7cr-20260823/build/core-relwithdebinfo/basalt_vio
set args --dataset-path /mnt/e/datasets/euroc_mav/machine_hall/MH_01_easy --cam-calib /root/visloc-basalt-clean-m7cr-20260823/data/euroc_ds_calib.json --dataset-type euroc --config-path /root/visloc-basalt-clean-m7cr-20260823/data/euroc_config.json --marg-data /marg_data_m7hd_gemm_stages --show-gui 0 --save-trajectory euroc --num-threads 1 --max-frames 5
start
set $n_fl = 0
set $n_fr = 0
set $n_a = 0
set $n_g = 0
break *0x7ffff72c27c0
commands
 silent
 set $n_fl = $n_fl + 1
 if $n_fl <= 2
  printf "M7HD GEMM stage=after_F_left n=%d pc=%p rbp=%p rbx=%p r12=%p r13=%p r14=%p\n", $n_fl, $pc, $rbp, $rbx, $r12, $r13, $r14
  x/18wx ($rbp - 0xd30)
 end
 continue
end
break *0x7ffff72c2824
commands
 silent
 set $n_fr = $n_fr + 1
 if $n_fr <= 2
  printf "M7HD GEMM stage=after_F_right n=%d pc=%p rbp=%p rbx=%p r12=%p r13=%p r14=%p\n", $n_fr, $pc, $rbx, $r12, $r13, $r14, $rbp
  x/18wx ($rbp - 0xd30)
 end
 continue
end
break *0x7ffff72c2945
commands
 silent
 set $n_a = $n_a + 1
 if $n_a <= 2
  printf "M7HD GEMM stage=after_A n=%d pc=%p rbp=%p rbx=%p r12=%p r13=%p r14=%p\n", $n_a, $pc, $rbp, $rbx, $r12, $r13, $r14
  x/18wx ($rbp - 0xd30)
 end
 continue
end
break *0x7ffff72c2a68
commands
 silent
 set $n_g = $n_g + 1
 printf "M7HD GEMM stage=after_G n=%d pc=%p rbp=%p rbx=%p r12=%p r13=%p r14=%p\n", $n_g, $pc, $rbp, $rbx, $r12, $r13, $r14
 x/18wx ($rbp - 0xd30)
 if $n_g >= 2
  quit
 end
 continue
end
continue
