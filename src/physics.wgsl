struct Cuboid {
    index: u32,
    position: vec4<f32>,
    rotation: vec4<f32>,
    velocity: vec4<f32>,
    angular_velocity: vec4<f32>,
    scale: vec4<f32>,
    corners: array<vec4<f32>, 8>,
    aabb_min: vec4<f32>,
    aabb_max: vec4<f32>,
    frozen: u32,
    face_axes: array<vec4<f32>, 3>,
    density: f32,
}
@group(0) @binding(0)
var<storage,read_write> cuboids:array<Cuboid>;

@compute @workgroup_size(64)
fn update(@builtin(global_invocation_id) global_id:vec3<u32>) {
    let index=global_id.x;
    cuboids[index]=cuboids[index]; //identity
    //TODO: integrate, update derived etc
}
