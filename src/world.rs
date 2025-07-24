use crate::math::{Quaternion, Vec3};

pub struct World {
    pub instances: Vec<Cuboid>,
}

const INSTANCES_PER_ROW: u32 = 2;
const INSTANCE_SPACING: f32 = 2.0;
impl World {
    pub fn new() -> Self {
        let instances = (0..INSTANCES_PER_ROW)
            .flat_map(|z| {
                (0..INSTANCES_PER_ROW).map(move |x| {
                    let position = Vec3 {
                        x: x as f32 * INSTANCE_SPACING,
                        y: 5.0,
                        z: z as f32 * INSTANCE_SPACING,
                    };
                    let rotation = Quaternion::identity();
                    Cuboid { position, rotation }
                })
            })
            .collect();

        Self { instances }
    }

    pub fn update(&mut self, dt: f32) {
        for instance in &mut self.instances {
            // instance.position += Vec3 {
            //     x: 0.0,
            //     y: 0.01,
            //     z: 0.01,
            // };
            instance.rotation = instance.rotation
                * Quaternion::from_angle(
                    &Vec3 {
                        x: 1.0,
                        y: 1.0,
                        z: 0.0,
                    },
                    0.02,
                );
        }
    }
}

pub struct Cuboid {
    position: Vec3, //centre
    rotation: Quaternion,
}
#[repr(C)]
#[derive(Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct CuboidRaw {
    model: [f32; 16],
}

impl CuboidRaw {
    //4 vec4s = 1 mat4
    const ATTRIBUTES: [wgpu::VertexAttribute; 4] =
        wgpu::vertex_attr_array![3=>Float32x4,4=>Float32x4,5=>Float32x4,6=>Float32x4];
    pub fn desc() -> wgpu::VertexBufferLayout<'static> {
        use std::mem;
        wgpu::VertexBufferLayout {
            array_stride: mem::size_of::<Self>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Instance,
            attributes: &Self::ATTRIBUTES,
        }
    }
}

impl Cuboid {
    pub fn new() -> Self {
        Self {
            position: Vec3::new(),
            rotation: Quaternion::identity(),
        }
    }

    #[rustfmt::skip]
    pub fn to_raw(&self)->CuboidRaw {
        let rotation_matrix=self.rotation.normalize().to_mat3();
        CuboidRaw {
            model:[
                    rotation_matrix.array[0], rotation_matrix.array[3], rotation_matrix.array[6], 0.0,
                    rotation_matrix.array[1], rotation_matrix.array[4], rotation_matrix.array[7], 0.0,
                    rotation_matrix.array[2], rotation_matrix.array[5], rotation_matrix.array[8], 0.0,
                    self.position.x,        self.position.y,        self.position.z,        1.0,
            ]
        }
    }
}
