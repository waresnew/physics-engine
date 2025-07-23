struct InstanceInput {
    @location(3) model_matrix_0: vec4<f32>,
    @location(4)model_matrix_1: vec4<f32>,
    @location(5)model_matrix_2: vec4<f32>,
    @location(6)model_matrix_3: vec4<f32>
}
struct CameraUniform {
    matrix: mat4x4<f32>,
};
@group(0) @binding(0)
var<uniform> camera:CameraUniform;
struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) color: vec3<f32>,
    @location(2) normal: vec3<f32>
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec3<f32>,
    @location(1) normal: vec3<f32>
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
    var out: VertexOutput;
    out.color = model.color;
    out.normal = normalize((model_matrix * vec4<f32>(model.normal, 0.0)).xyz);
    out.clip_position = camera.matrix * model_matrix * vec4<f32>(model.position, 1.0);
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let light_pos = normalize(vec3<f32>(1.0, 2.0, 1.0));
    let diffuse = max(dot(in.normal, light_pos), 0.0);
    let ambient = 0.1;
    let shade = vec3<f32>(1.0, 1.0, 1.0) * (diffuse + ambient);
    return vec4<f32>(in.color * shade, 1.0);
}
