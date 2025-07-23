use crate::math::{Mat3, Vec3};

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
                        y: 0.0,
                        z: z as f32 * INSTANCE_SPACING,
                    };
                    let rotation = Mat3::identity();
                    Cuboid { position, rotation }
                })
            })
            .collect();

        Self { instances }
    }

    pub fn update(&mut self, dt: f32) {
        for instance in &mut self.instances {
            // instance.position += Vec3{x:0.0,y:0.01,z:0.0};
        }
    }
}

pub struct Cuboid {
    position: Vec3,
    rotation: Mat3,
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
            rotation: Mat3::identity(),
        }
    }

    #[rustfmt::skip]
    pub fn to_raw(&self)->CuboidRaw {
        CuboidRaw {
            model:[
                    self.rotation.array[0], self.rotation.array[3], self.rotation.array[6], 0.0,
                    self.rotation.array[1], self.rotation.array[4], self.rotation.array[7], 0.0,
                    self.rotation.array[2], self.rotation.array[5], self.rotation.array[8], 0.0,
                    self.position.x,        self.position.y,        self.position.z,        1.0,
            ]
        }
    }
}
