set pagination off
set confirm off
set breakpoint pending on
set disable-randomization on
set print pretty on
set print elements 100
set print demangle on
set debuginfod enabled off
file /root/visloc-basalt-clean-m7cr-20260823/build/core-relwithdebinfo/basalt_vio
set args --dataset-path /mnt/e/datasets/euroc_mav/machine_hall/MH_01_easy --cam-calib /root/visloc-basalt-clean-m7cr-20260823/data/euroc_ds_calib.json --dataset-type euroc --config-path /root/visloc-basalt-clean-m7cr-20260823/data/euroc_config.json --marg-data /marg_data_m7hd_sqrt --show-gui 0 --save-trajectory euroc --num-threads 4 --max-frames 5
start
set $entry_n = 0
set $return_n = 0
break *0x7ffff70c23f0
commands
 silent
 set $entry_n = $entry_n + 1
 printf "M7HD SQRT ENTRY n=%d pc=%p\n", $entry_n, $pc
 tbreak *($pc + 0xb78)
 commands
  silent
  set $return_n = $return_n + 1
  printf "M7HD SQRT RETURN n=%d this=%p\n", $return_n, $rbx
  printf "M7HD sqrt_cov_inv 81 float words (Eigen column-major)\n"
  x/81wx ($rbx + 0x194)
  if $return_n >= 4
   quit
  end
  continue
 end
 continue
end
continue
