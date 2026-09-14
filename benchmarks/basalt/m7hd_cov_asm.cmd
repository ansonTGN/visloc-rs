set pagination off
set confirm off
set breakpoint pending on
set disable-randomization on
set debuginfod enabled off
file /root/visloc-basalt-clean-m7cr-20260823/build/core-relwithdebinfo/basalt_vio
set args --dataset-path /mnt/e/datasets/euroc_mav/machine_hall/MH_01_easy --cam-calib /root/visloc-basalt-clean-m7cr-20260823/data/euroc_ds_calib.json --dataset-type euroc --config-path /root/visloc-basalt-clean-m7cr-20260823/data/euroc_config.json --marg-data /marg_data_m7hd_cov_asm --show-gui 0 --save-trajectory euroc --num-threads 1 --max-frames 5
start
break /root/visloc-basalt-clean-m7cr-20260823/include/basalt/linearization/imu_block.hpp:26
commands
 silent
 printf "M7HD COV ASM ENTRY pc=%p\n", $pc
 x/260i $pc
 quit
end
run
