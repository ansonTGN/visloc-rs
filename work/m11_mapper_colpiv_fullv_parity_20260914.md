# M11 mapper ColPiv Full-V parity — 2026-09-14

Status: **PASS_EXACT_MATCH_GRAPH**

The last native/Rust temporal-inlier mismatch is resolved. OpenGV samples eight correspondences, sends the first five to `fivept_stewenius`, and uses all eight only for pose disambiguation. Eigen's dynamic 5x9 `JacobiSVD` scales the matrix and applies its default `ColPivHouseholderQR` preconditioner. The required four nullspace vectors are the trailing four columns of the resulting full Q.

The Rust port now follows that path instead of diagonalizing `Q^T Q`. Against the native stage capture, the QR matrix differs by at most `1.78e-15`, the Householder coefficients by `6.66e-16`, and both full Q and `V.rightCols(4)` by `2.78e-15`.

The real frame-51 native MargData packet, seed 7 run now matches all 120 native pair inlier sets exactly: 15,299 raw matches and 14,139 inliers on both sides, with zero differing pairs. It also retains 397 tracks, 382 landmarks, and 3,583 final observations. The Rust trajectory, poses, map, and points hashes are unchanged from the already tolerance-validated downstream result; all 3,583 points remain within 1 mm of native.

Validation completed on E: only: focused native stage oracle passed, mapper feature tests passed, release build passed, and the real packet end-to-end run passed.
