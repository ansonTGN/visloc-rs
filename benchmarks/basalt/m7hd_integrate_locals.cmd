set pagination off
set confirm off
set disable-randomization on
set auto-solib-add off
set print demangle on
set debuginfod enabled off
file /root/visloc-basalt-clean-m7cr-20260823/build/core-relwithdebinfo/basalt_vio
set args --dataset-path /mnt/e/datasets/euroc_mav/machine_hall/MH_01_easy --cam-calib /root/visloc-basalt-clean-m7cr-20260823/data/euroc_ds_calib.json --dataset-type euroc --config-path /root/visloc-basalt-clean-m7cr-20260823/data/euroc_config.json --marg-data /marg_data_m7hd_integrate_locals --show-gui 0 --save-trajectory euroc --num-threads 1 --max-frames 5
start
set $hit_n = 0
set $packet_n = 0
break *0x7ffff72c2a80
commands
 silent
 set $hit_n = $hit_n + 1
 if $rbx == $rbp - 0xd30
  set $packet_n = $packet_n + 1
  printf "M7HD INTEGRATE ADD pc=%p packet=%d hit=%d r12=%p rbx=%p r14=%p r13=%p r10=%p r11=%p rax=%p rdx=%p\n", $pc, $packet_n, $hit_n, $r12, $rbx, $r14, $r13, $r10, $r11, $rax, $rdx
  printf "cov_0x50_first_18\n"
  x/18wx ($r12 + 0x50)
  printf "local_d30_first_18\n"
  x/18wx ($rbp - 0xd30)
  if $packet_n >= 4
   quit
  end
 end
 continue
end
continue
