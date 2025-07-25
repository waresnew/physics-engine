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
const FLOOR_AABB: AABB = AABB {
    min: Vec3 {
        x: f32::NEG_INFINITY,
        y: 0.0,
        z: f32::NEG_INFINITY,
    },
    max: Vec3 {
        x: f32::INFINITY,
        y: 0.0,
        z: f32::INFINITY,
    },
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
            let aabb = instance.get_aabb();
            if !aabb.intersects(&FLOOR_AABB) {
                instance.velocity += GRAV_ACCEL * dt;
                instance.position += instance.velocity * dt;
            }

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
    position: Vec3,       //centre
    rotation: Quaternion, //TODO: normalize quaternion after updating it so that i can assume it already is when reading it
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
    pub fn get_aabb(&self) -> AABB {
        let delta: Vec3 = self.scale / 2.0;
        let mut min_x = f32::INFINITY;
        let mut min_y = f32::INFINITY;
        let mut min_z = f32::INFINITY;
        let mut max_x = f32::NEG_INFINITY;
        let mut max_y = f32::NEG_INFINITY;
        let mut max_z = f32::NEG_INFINITY;
        for i in [-1.0, 1.0] {
            for j in [-1.0, 1.0] {
                for k in [-1.0, 1.0] {
                    let corner = self.position
                        + (Vec3 {
                            x: i * delta.x,
                            y: j * delta.y,
                            z: k * delta.z,
                        }
                        .rotate(self.rotation));

                    min_x = min_x.min(corner.x);
                    max_x = max_x.max(corner.x);
                    min_y = min_y.min(corner.y);
                    max_y = max_y.max(corner.y);
                    min_z = min_z.min(corner.z);
                    max_z = max_z.max(corner.z);
                }
            }
        }
        AABB {
            min: Vec3 {
                x: min_x,
                y: min_y,
                z: min_z,
            },
            max: Vec3 {
                x: max_x,
                y: max_y,
                z: max_z,
            },
        }
    }
    #[rustfmt::skip]
    pub fn to_raw(&self)->CuboidRaw {
        let rotation_matrix=self.rotation.to_mat3();
        CuboidRaw {
            model:[
                    rotation_matrix.array[0]*self.scale.x, rotation_matrix.array[3]*self.scale.x, rotation_matrix.array[6]*self.scale.x, 0.0,
                    rotation_matrix.array[1]*self.scale.y, rotation_matrix.array[4]*self.scale.y, rotation_matrix.array[7]*self.scale.y, 0.0,
                    rotation_matrix.array[2]*self.scale.z, rotation_matrix.array[5]*self.scale.z, rotation_matrix.array[8]*self.scale.z, 0.0,
                    self.position.x,        self.position.y,        self.position.z,                               1.0,
            ]
        }
    }
}

pub struct AABB {
    min: Vec3,
    max: Vec3,
}

impl AABB {
    pub fn new(min: Vec3, max: Vec3) -> Self {
        assert!(
            min.x <= max.x && min.y <= max.y && min.z <= max.z,
            "invalid AABB: min:{min:?} max:{max:?}",
        );
        Self { min, max }
    }

    pub fn get_dimensions(&self) -> Vec3 {
        Vec3 {
            x: self.max.x - self.min.x,
            y: self.max.y - self.min.y,
            z: self.max.z - self.min.z,
        }
    }
    pub fn intersects(&self, other: &AABB) -> bool {
        !((self.min.x > other.max.x || self.min.y > other.max.y || self.min.z > other.max.z)
            || (self.max.x < other.min.x || self.max.y < other.min.y || self.max.z < other.min.z))
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
