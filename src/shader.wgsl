struct CameraUniform {
    matrix: mat4x4<f32>,
    view_pos: vec3<f32>,
    _padding1: f32
};
@group(0) @binding(0)
var<uniform> camera: CameraUniform;
@group(0) @binding(1)
var<storage,read>cuboids:array<Cuboid>;
struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) color: vec3<f32>,
    @location(2) normal: vec3<f32>
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) world_position: vec3<f32>
};
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
};

@vertex fn vs_main( model: VertexInput,@builtin(instance_index) index:u32
) -> VertexOutput {
    let cuboid=cuboids[index];
    let scale=cuboid.scale;
    let rotation=cuboid.rotation;
    let position=cuboid.position;
    let r=rotation[0];
    let x=rotation[1];
    let y=rotation[2];
    let z=rotation[3];
    let model_matrix=mat4x4<f32>(
        vec4<f32>( (r*r + x*x - y*y - z*z) * scale.x, (-2.0*r*z + 2.0*x*y) * scale.x, (2.0*r*y + 2.0*x*z) * scale.x, 0.0 ),
        vec4<f32>( (2.0*r*z + 2.0*x*y) * scale.y, (r*r + y*y - x*x - z*z) * scale.y, (-2.0*r*x + 2.0*y*z) * scale.y, 0.0 ),
        vec4<f32>( (-2.0*r*y + 2.0*x*z) * scale.z, (2.0*r*x + 2.0*y*z) * scale.z, (r*r + z*z - x*x - y*y) * scale.z, 0.0 ),
        vec4<f32>( position.x, position.y, position.z, 1.0 )
    );
    return calculate_vertex_output(model, model_matrix);
}

fn calculate_vertex_output(model: VertexInput, model_matrix: mat4x4<f32>) -> VertexOutput {
    var out: VertexOutput;
    out.color = model.color;
    out.normal = normalize((model_matrix * vec4<f32>(model.normal, 0.0)).xyz);
    let world_position: vec4<f32> = model_matrix * vec4<f32>(model.position, 1.0);
    out.world_position = world_position.xyz;
    out.clip_position = camera.matrix * world_position;
    return out;
}

@vertex
fn vs_static(
    model: VertexInput
) -> VertexOutput {
    let model_matrix = mat4x4<f32>(
        vec4<f32>(1.0, 0.0, 0.0, 0.0),
        vec4<f32>(0.0, 1.0, 0.0, 0.0),
        vec4<f32>(0.0, 0.0, 1.0, 0.0),
        vec4<f32>(0.0, 0.0, 0.0, 1.0),
    );
    return calculate_vertex_output(model, model_matrix);
}

fn srgb_to_linear(colour: f32) -> f32 {
    return pow((colour + 0.055) / 1.055, 2.4);
}
@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let light_dir = -normalize(vec3<f32>(-0.5, -1.0, 0.5));
    let light_colour = vec3<f32>(1.0, 1.0, 1.0);
    let linear_colour = vec3<f32>(
        srgb_to_linear(in.color.r),
        srgb_to_linear(in.color.g),
        srgb_to_linear(in.color.b)
    );
    //lambertian diffuse
    let diffuse = max(dot(in.normal, light_dir), 0.0);

    //blinn-phong specular
    let view_dir = normalize(camera.view_pos.xyz - in.world_position);
    let half_dir = normalize(view_dir + light_dir);
    let specular = pow(max(dot(in.normal, half_dir), 0.0), 32.0);

    //fresnel factor, schlick approximation
    let R0 = vec3<f32>(0.04);
    let fresnel_cos = max(dot(view_dir, in.normal), 0.0);
    let fresnel = R0 + (vec3<f32>(1.0) - R0) * pow(1 - fresnel_cos, 5.0);

    let ambient_term = 0.4 * linear_colour;
    let diffuse_term = diffuse * light_colour * linear_colour;
    let specular_term = specular * light_colour * fresnel; //no linear_colour bc it's just light reflected
    return vec4<f32>(ambient_term + diffuse_term + specular_term, 1.0);
}
