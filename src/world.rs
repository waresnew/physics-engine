use crate::math::{Mat3, Mat4, Vec3};

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
}

pub struct Cuboid {
    position: Vec3,
    rotation: Mat3,
}
#[repr(C)]
#[derive(Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct CuboidRaw {
    model: [[f32; 4]; 4],
}

impl CuboidRaw {
    //4 vec4s = 1 mat4
    const ATTRIBUTES: [wgpu::VertexAttribute; 4] =
        wgpu::vertex_attr_array![2=>Float32x4,3=>Float32x4,4=>Float32x4,5=>Float32x4];
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
        let mut temp=Mat4::new();
        temp.array=[
                [self.rotation.array[0][0], self.rotation.array[0][1], self.rotation.array[0][2], self.position.x],
                [self.rotation.array[1][0], self.rotation.array[1][1], self.rotation.array[1][2],self.position.y],
                [self.rotation.array[2][0], self.rotation.array[2][1], self.rotation.array[2][2],self.position.z],
                [0.0, 0.0, 0.0, 1.0]
        ];


        CuboidRaw {
            model:temp.transpose().array         
        }
    }
}
