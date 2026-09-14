// Diagnostic-only mirror of the active Rust `sophus_packet::quaternion_product`
// arithmetic. Inputs/outputs use Eigen xyzw coefficient order.
#[inline]
fn product(first: [f32; 4], second: [f32; 4]) -> [f32; 4] {
    let [ax, ay, az, aw] = first;
    let [bx, by, bz, bw] = second;
    let raw = [
        (-az).mul_add(by, ay.mul_add(bz, aw.mul_add(bx, ax * bw))),
        (-ax).mul_add(bz, az.mul_add(bx, aw.mul_add(by, ay * bw))),
        (-ay).mul_add(bx, ax.mul_add(by, aw.mul_add(bz, az * bw))),
        (-az).mul_add(bz, (-ay).mul_add(by, bw.mul_add(aw, -(bx * ax)))),
    ];
    let [x, y, z, w] = raw;
    let norm_sq = (x * x + z * z) + (y * y + w * w);
    let scale = norm_sq.sqrt();
    [x / scale, y / scale, z / scale, w / scale]
}

#[inline]
fn product_scalar(first: [f32; 4], second: [f32; 4]) -> [f32; 4] {
    let [ax, ay, az, aw] = first;
    let [bx, by, bz, bw] = second;
    let raw = [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by + ay * bw + az * bx - ax * bz,
        aw * bz + az * bw + ax * by - ay * bx,
        aw * bw - ax * bx - ay * by - az * bz,
    ];
    let [x, y, z, w] = raw;
    let norm_sq = (x * x + z * z) + (y * y + w * w);
    let scale = norm_sq.sqrt();
    [x / scale, y / scale, z / scale, w / scale]
}

fn raw(first: [f32; 4], second: [f32; 4]) -> [f32; 4] {
    let [ax, ay, az, aw] = first;
    let [bx, by, bz, bw] = second;
    [
        (-az).mul_add(by, ay.mul_add(bz, aw.mul_add(bx, ax * bw))),
        (-ax).mul_add(bz, az.mul_add(bx, aw.mul_add(by, ay * bw))),
        (-ay).mul_add(bx, ax.mul_add(by, aw.mul_add(bz, az * bw))),
        (-az).mul_add(bz, (-ay).mul_add(by, bw.mul_add(aw, -(bx * ax)))),
    ]
}

fn normalize_with(value: [f32; 4], norm_sq: f32) -> [f32; 4] {
    let scale = norm_sq.sqrt();
    [value[0] / scale, value[1] / scale, value[2] / scale, value[3] / scale]
}

fn f(bits: u32) -> f32 { f32::from_bits(bits) }
fn print(label: &str, q: [f32; 4]) {
    println!("{label}={:08x},{:08x},{:08x},{:08x}",
        q[0].to_bits(), q[1].to_bits(), q[2].to_bits(), q[3].to_bits());
}

fn main() {
    let q1 = [f(0xbd5cdea6), f(0xbf4d341f), f(0xbb2aa78b), f(0x3f186f67)];
    let dq12 = [f(0xbb1356a1), f(0xbb1ec3a1), f(0xbad83d21), f(0x3f7fff8e)];
    let q3 = [f(0xbd5e760f), f(0xbf4e5e85), f(0xbbc2d95f), f(0x3f16d68a)];
    let dq34 = [f(0xb8675a3d), f(0xbbc5a149), f(0x396ab03e), f(0x3f7ffecf)];
    print("frame1_to_frame2", product(q1, dq12));
    print("frame3_to_frame4", product(q3, dq34));
    print("frame1_to_frame2_scalar", product_scalar(q1, dq12));
    print("frame3_to_frame4_scalar", product_scalar(q3, dq34));
    let r = raw(q1, dq12);
    print("frame1_raw", r);
    let norms = [
        (r[0] * r[0] + r[1] * r[1]) + (r[2] * r[2] + r[3] * r[3]),
        (r[0] * r[0] + r[2] * r[2]) + (r[1] * r[1] + r[3] * r[3]),
        (r[0] * r[0] + (r[1] * r[1] + (r[2] * r[2] + r[3] * r[3]))),
        ((r[0] * r[0] + r[1] * r[1]) + r[2] * r[2]) + r[3] * r[3],
    ];
    for (i, n) in norms.into_iter().enumerate() {
        println!("frame1_norm{i}={:08x}", n.to_bits());
        print(&format!("frame1_n{i}"), normalize_with(r, *n));
    }
}
