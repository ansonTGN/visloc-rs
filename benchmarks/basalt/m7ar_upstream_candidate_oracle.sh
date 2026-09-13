#!/usr/bin/env bash
set -e
root=/root/visloc-basalt-oracle-0f3b2b52
src=$root/src/vi_estimator/sqrt_keypoint_vio.cpp
bak=$src.m7ar.bak
cp -p "$src" "$bak"
trap 'cp -p "$bak" "$src"; rm -f "$bak"' EXIT
sed -i '/p1_3d\.template head<3>(), T_0_1);/a\        if (const char* p = std::getenv("BASALT_CANDIDATE_TRACE")) { std::ofstream f(p, std::ios::out | std::ios::app); if (opt_flow_meas->t_ns == 1403636581513555456) f << std::setprecision(17) << lm_id << " " << tcido.frame_id << " " << tcido.cam_id << " " << T_0_1.translation().squaredNorm() << " " << p0_triangulated[0] << " " << p0_triangulated[1] << " " << p0_triangulated[2] << " " << p0_triangulated[3] << "\\n"; }' "$src"
cmake --build "$root/build/core-relwithdebinfo" --target basalt_vio -j4
rm -rf /tmp/m7ar_up
mkdir -p /tmp/m7ar_up
BASALT_CANDIDATE_TRACE=/tmp/m7ar_up/candidates.txt "$root/build/core-relwithdebinfo/basalt_vio" --dataset-path /mnt/e/datasets/euroc_mav/machine_hall/MH_01_easy --cam-calib "$root/data/euroc_ds_calib.json" --dataset-type euroc --config-path "$root/data/euroc_config.json" --marg-data /tmp/m7ar_up/marg --show-gui 0 --save-trajectory euroc --max-frames 36 >/tmp/m7ar_up/stdout.log 2>/tmp/m7ar_up/stderr.log
wc -l /tmp/m7ar_up/candidates.txt
head -5 /tmp/m7ar_up/candidates.txt
