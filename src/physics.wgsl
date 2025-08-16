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

const PHYSICS_DT: f32 = 1.0 / 60.0;

// SI units
const GRAV_ACCEL: vec3<f32> = vec3(0.0, -9.81, 0.0);

const GLOBAL_AXES = array(
    vec3<f32>(1.0, 0.0, 0.0),
    vec3<f32>(0.0, 1.0, 0.0),
    vec3<f32>(0.0, 0.0, 1.0),
);
@compute @workgroup_size(64)
fn update(@builtin(global_invocation_id) global_id:vec3<u32>) {
    let index=global_id.x;
    if cuboids[index].frozen==1u {
        return;
    }
    cuboids[index].velocity+=vec4(GRAV_ACCEL*PHYSICS_DT,0.0);
    cuboids[index].position+=cuboids[index].velocity*PHYSICS_DT;
    
}
