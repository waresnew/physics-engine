use crate::math::{Mat3, Quaternion, Vec3};

pub struct World {
    pub instances: Vec<Cuboid>,
}

const INSTANCES_PER_ROW: u32 = 2;
const INSTANCE_SPACING: f32 = 2.0;
const GRAV_ACCEL: Vec3 = Vec3 {
    x: 0.0,
    y: -9.81,
    z: 0.0,
};

impl World {
    pub fn new() -> Self {
        let instances = (0..INSTANCES_PER_ROW)
            .flat_map(|z| {
                (0..INSTANCES_PER_ROW).map(move |x| {
                    let scale = Vec3 {
                        x: 1.0,
                        y: 1.0,
                        z: 1.0,
                    };
                    let position = Vec3 {
                        x: x as f32 * INSTANCE_SPACING * scale.x,
                        y: 5.0,
                        z: z as f32 * INSTANCE_SPACING * scale.z,
                    };
                    Cuboid::new(position, scale)
                })
            })
            .collect();

        Self { instances }
    }

    pub fn update(&mut self, dt: f32) {
        for instance in &mut self.instances {
            instance.velocity += GRAV_ACCEL * dt;
            instance.position += instance.velocity * dt;

            // instance.rotation = instance.rotation
            //     * Quaternion::from_angle(
            //         &Vec3 {
            //             x: 1.0,
            //             y: 1.0,
            //             z: 1.0,
            //         },
            //         0.02,
            //     );
        }
    }
}

pub struct Cuboid {
    position: Vec3, //centre
    rotation: Quaternion,
    velocity: Vec3,
    angular_velocity: Vec3,
    scale: Vec3,
}
impl Cuboid {
    pub fn new(position: Vec3, scale: Vec3) -> Self {
        Self {
            position,
            rotation: Quaternion::identity(),
            velocity: Vec3::new(),
            angular_velocity: Vec3::new(),
            scale,
        }
    }
    pub fn get_AABB(&self) {}
    #[rustfmt::skip]
    pub fn to_raw(&self)->CuboidRaw {
        let rotation_matrix=self.rotation.normalize().to_mat3();
        CuboidRaw {
            model:[
                    rotation_matrix.array[0]*self.scale.x, rotation_matrix.array[3]*self.scale.x, rotation_matrix.array[6]*self.scale.x, 0.0,
                    rotation_matrix.array[1]*self.scale.y, rotation_matrix.array[4]*self.scale.y, rotation_matrix.array[7]*self.scale.y, 0.0,
                    rotation_matrix.array[2]*self.scale.z, rotation_matrix.array[5]*self.scale.z, rotation_matrix.array[8]*self.scale.z, 0.0,
                    self.position.x,        self.position.y,        self.position.z,        1.0,
            ]
        }
    }
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
