struct InstanceInput {
    @location(3) model_matrix_0: vec4<f32>,
    @location(4)model_matrix_1: vec4<f32>,
    @location(5)model_matrix_2: vec4<f32>,
    @location(6)model_matrix_3: vec4<f32>
}
struct CameraUniform {
    matrix: mat4x4<f32>,
    view_pos: vec3<f32>,
    _padding1: f32
};
@group(0) @binding(0)
var<uniform> camera: CameraUniform;
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

@vertex
fn vs_main(
    model: VertexInput,
    instance: InstanceInput
) -> VertexOutput {
    let model_matrix = mat4x4<f32>(
        instance.model_matrix_0,
        instance.model_matrix_1,
        instance.model_matrix_2,
        instance.model_matrix_3
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
    let light_dir = -normalize(vec3<f32>(0.0, -1.0, 0.0));
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

    let ambient_term = 0.1 * linear_colour;
    let diffuse_term = diffuse * light_colour * linear_colour;
    let specular_term = specular * light_colour * fresnel; //no linear_colour bc it's just light reflected
    return vec4<f32>(ambient_term + diffuse_term + specular_term, 1.0);
}
