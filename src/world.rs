use crate::{
    math::{EPSILON, Mat3, Quaternion, Vec3},
    physics::{CollisionInfo, apply_impulse, detect_collision},
};

pub struct World {
    pub instances: [Cuboid; N + 1],
    floor: Cuboid,
    collisions: Vec<CollisionInfo>,
}

// SI units
pub const N: usize = 1; //number of cuboids minus floor
const DENSITY: f32 = 850.0;
const INSTANCE_SPACING: f32 = 2.0;
const GRAV_ACCEL: Vec3 = Vec3 {
    x: 0.0,
    y: -9.81,
    z: 0.0,
};
const GLOBAL_AXES: [Vec3; 3] = [
    Vec3 {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    },
    Vec3 {
        x: 0.0,
        y: 1.0,
        z: 0.0,
    },
    Vec3 {
        x: 0.0,
        y: 0.0,
        z: 1.0,
    },
];

impl World {
    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        let num_cols = N.isqrt();
        let mut instances: [Cuboid; N + 1] = [Cuboid::default(); N + 1];
        for i in 0..N {
            let row = i / num_cols;
            let col = i % num_cols;
            let scale = Vec3 {
                x: 1.0,
                y: 1.0,
                z: 1.0,
            };
            let position = Vec3 {
                x: row as f32 * INSTANCE_SPACING * scale.x,
                y: 10.0,
                z: col as f32 * INSTANCE_SPACING * scale.z,
            };
            instances[i] = Cuboid {
                position,
                scale,
                index: i,
                ..Default::default()
            };
        }
        let mut floor = Cuboid {
            scale: Vec3 {
                x: 200.0,
                y: 1.0,
                z: 200.0,
            },
            position: Vec3 {
                x: 0.0,
                y: -0.5,
                z: 0.0,
            },
            index: N,
            frozen: true,
            ..Default::default()
        };
        floor.update_derived();
        instances[N] = floor;

        Self {
            instances,
            floor,
            collisions: Vec::with_capacity(N * 8 / 2 + N),
        }
    }

    pub fn update(&mut self, dt: f32) {
        self.collisions.clear();
        for i in 0..N {
            let instance = &mut self.instances[i];
            instance.velocity += GRAV_ACCEL * dt;
            instance.position += instance.velocity * dt;
            if instance.angular_velocity.mag() > EPSILON * dt {
                instance.rotation = (instance.rotation
                    * Quaternion::from_angle(
                        &instance.angular_velocity.normalize(),
                        instance.angular_velocity.mag() * dt,
                    ))
                .normalize();
            }
            instance.update_derived();

            if instance.aabb.intersects(&self.floor.aabb) {
                if let Some(collision_info) = detect_collision(instance, &self.floor) {
                    if self.collisions.len() < self.collisions.capacity() {
                        self.collisions.push(collision_info);
                    } else {
                        eprintln!("self.collisions capacity exceeded");
                    }
                }
            }

            //OPTIMIZE:O(n^2)
            for j in (i + 1)..N {
                let (a, b) = self.instances.split_at_mut(j);
                let instance = &mut a[i];
                let other = &mut b[0];
                if let Some(collision_info) = detect_collision(instance, other) {
                    if self.collisions.len() < self.collisions.capacity() {
                        self.collisions.push(collision_info);
                    } else {
                        eprintln!("self.collisions capacity exceeded");
                    }
                }
            }
        }
        for collision in &mut self.collisions {
            apply_impulse(collision, &mut self.instances, dt);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_floor_corners() {
        let world = World::new();
        let count1 = world.floor.corners.iter().filter(|x| x.y == 0.0).count();
        let count2 = world.floor.corners.iter().filter(|x| x.y == -1.0).count();

        assert_eq!(count1, 4);
        assert_eq!(count2, 4);
    }
}

#[derive(Debug, Copy, Clone)]
pub struct Cuboid {
    pub index: usize,
    pub position: Vec3, //centre
    pub rotation: Quaternion,
    pub velocity: Vec3,         //ms^-1
    pub angular_velocity: Vec3, //rads^-1
    pub scale: Vec3,
    pub corners: [Vec3; 8],
    pub aabb: AABB,
    pub frozen: bool,
    pub face_axes: [Vec3; 3],
    pub centre: Vec3,
}
impl Cuboid {
    pub fn update_derived(&mut self) {
        self.calc_corners();
        self.calc_aabb();
        self.calc_face_axes();
        self.calc_centre();
    }
    fn calc_centre(&mut self) {
        let mut centre = Vec3::default();
        for v in self.corners {
            centre += v;
        }
        centre /= 8.0;
        self.centre = centre;
    }
    #[rustfmt::skip]
    pub fn get_inverse_moment_of_inertia(&self)->Mat3 {
        let i_m=self.get_inverse_mass();
        if i_m==0.0 {
            return Mat3::zero();
        }
        let m=1.0/i_m;
        if m==f32::INFINITY {
            return Mat3::zero();
        }
        let Vec3{x,y,z}=self.scale;
        let ixx = (1.0/12.0) * m * (y*y + z*z);
        let iyy = (1.0/12.0) * m * (x*x + z*z);
        let izz = (1.0/12.0) * m * (x*x + y*y);
        Mat3 {
            array: [
                1.0/ixx,0.0,0.0,
                0.0,1.0/iyy,0.0,
                0.0,0.0,1.0/izz
            ]
        }
    }
    fn calc_face_axes(&mut self) {
        self.face_axes = [
            GLOBAL_AXES[0].rotate(self.rotation).normalize(),
            GLOBAL_AXES[1].rotate(self.rotation).normalize(),
            GLOBAL_AXES[2].rotate(self.rotation).normalize(),
        ];
    }
    pub fn get_inverse_mass(&self) -> f32 {
        if self.frozen {
            0.0
        } else {
            1.0 / (self.scale.mag() * DENSITY)
        }
    }
    fn calc_corners(&mut self) {
        let delta: Vec3 = self.scale / 2.0;
        let mut index = 0;
        let mut ans = [Vec3::default(); 8];
        for i in [-1.0, 1.0] {
            for j in [-1.0, 1.0] {
                for k in [-1.0, 1.0] {
                    ans[index] = self.position
                        + (Vec3 {
                            x: i * delta.x,
                            y: j * delta.y,
                            z: k * delta.z,
                        }
                        .rotate(self.rotation));
                    index += 1;
                }
            }
        }
        self.corners = ans;
    }

    pub fn calc_aabb(&mut self) {
        let mut min_x = f32::INFINITY;
        let mut min_y = f32::INFINITY;
        let mut min_z = f32::INFINITY;
        let mut max_x = f32::NEG_INFINITY;
        let mut max_y = f32::NEG_INFINITY;
        let mut max_z = f32::NEG_INFINITY;
        for corner in self.corners {
            min_x = min_x.min(corner.x);
            max_x = max_x.max(corner.x);
            min_y = min_y.min(corner.y);
            max_y = max_y.max(corner.y);
            min_z = min_z.min(corner.z);
            max_z = max_z.max(corner.z);
        }
        self.aabb = AABB {
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
impl Default for Cuboid {
    fn default() -> Self {
        Self {
            scale: Vec3 {
                x: 1.0,
                y: 1.0,
                z: 1.0,
            },
            position: Vec3::default(),
            rotation: Quaternion::default(),
            velocity: Vec3::default(),
            angular_velocity: Vec3::default(),
            corners: [Vec3::default(); 8],
            aabb: AABB::default(),
            face_axes: [Vec3::default(); 3],
            frozen: false,
            index: 0,
            centre: Vec3::default(),
        }
    }
}
#[derive(Debug, Default, Copy, Clone)]
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
#[derive(Default, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
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
