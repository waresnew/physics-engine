use std::{
    collections::HashSet,
    io::{self, Write},
    time::{Duration, Instant},
};

use crate::{
    hash_grid::HashGrid,
    math::{EPSILON, Mat3, Quaternion, Vec3},
    physics::{CollisionInfo, detect_collision, resolve_collisions},
    scenes::{N, Scene},
};

pub struct World {
    pub instances: Vec<Cuboid>,
    floor: Cuboid,
    collisions: Vec<CollisionInfo>,
    hash_grid: HashGrid,
    last_log: Instant,
    last_tick: Instant,
    vis_collisions: HashSet<(usize, usize)>,
}

pub const PHYSICS_DT: f32 = 1.0 / 180.0; //180 for accuracy, 60 for speed
// SI units
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
        let mut instances = Vec::with_capacity(N + 1);
        Scene::Cube.populate_scene(&mut instances);

        let mut floor = Cuboid {
            scale: Vec3 {
                x: 1000.0,
                y: 1.0,
                z: 1000.0,
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
        instances.push(floor);

        let hash_grid = HashGrid::new(&instances, N);
        Self {
            instances,
            floor,
            collisions: Vec::with_capacity(N * 8 / 2 + N),
            hash_grid,
            last_log: Instant::now(),
            last_tick: Instant::now(),
            vis_collisions: HashSet::with_capacity(N * 8 / 2 + N),
        }
    }

    pub fn update(&mut self) {
        let dt = PHYSICS_DT;
        for i in 0..N {
            let instance = &mut self.instances[i];
            if !instance.frozen {
                instance.velocity += GRAV_ACCEL * dt;
                instance.position += instance.velocity * dt;
                if instance.angular_velocity.mag() > EPSILON * dt {
                    instance.rotation = (Quaternion::from_angle(
                        &instance.angular_velocity.normalize().unwrap(),
                        instance.angular_velocity.mag() * dt,
                    ) * instance.rotation)
                        .normalize();
                }

                instance.update_derived();
            }
        }
        let grid_init_time = Instant::now();
        self.hash_grid.clear();
        self.hash_grid.init(&self.instances);
        let floor_time = Instant::now();

        for i in 0..N {
            let instance = &self.instances[i];
            if instance.aabb.intersects(&self.floor.aabb) {
                if let Some(collision_info) = detect_collision(instance, &self.floor) {
                    if self.collisions.len() < self.collisions.capacity() {
                        self.collisions.push(collision_info);
                    } else {
                        eprintln!("self.collisions capacity exceeded");
                    }
                }
            }
        }

        let broad_time = Instant::now();
        let mut narrow_time = 0;
        let mut check_count = 0;
        for bucket in &self.hash_grid.buckets {
            if bucket.len() <= 1 {
                continue;
            }
            for i in 0..bucket.len() {
                let instance = &self.instances[bucket[i]];
                for j in i + 1..bucket.len() {
                    let other = &self.instances[bucket[j]];
                    check_count += 1;
                    if instance.index == other.index {
                        continue;
                    }
                    let pair = (
                        instance.index.min(other.index),
                        instance.index.max(other.index),
                    );
                    if self.vis_collisions.contains(&pair) {
                        continue;
                    }
                    let pre = Instant::now();
                    if instance.aabb.intersects(&other.aabb) {
                        if let Some(collision_info) = detect_collision(instance, other) {
                            if self.collisions.len() < self.collisions.capacity() {
                                self.collisions.push(collision_info);
                                self.vis_collisions.insert(pair);
                            } else {
                                eprintln!("self.collisions capacity exceeded");
                            }
                        }
                    }
                    narrow_time += pre.elapsed().as_nanos();
                }
            }
        }
        let broad_phase_time = (Instant::now() - broad_time).as_nanos() - narrow_time;
        let impulse_time = Instant::now();
        resolve_collisions(&self.collisions, &mut self.instances, dt);
        if self.last_log.elapsed() > Duration::from_secs(1) {
            print!(
                "\rTPS: {}, grid init: {} ms, floor: {} ms, check count: {}, broad: {} ms, narrow {} ms, impulse: {} ms",
                self.last_tick.elapsed().as_millis(),
                (floor_time - grid_init_time).as_millis(),
                (broad_time - floor_time).as_millis(),
                check_count,
                broad_phase_time / 1000000,
                narrow_time / 1000000,
                impulse_time.elapsed().as_millis()
            );
            io::stdout().flush().unwrap();
            self.last_log = Instant::now();
        }
        self.collisions.clear();
        self.vis_collisions.clear();
        self.last_tick = Instant::now();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cuboid_model_matrix() {
        let cuboid = Cuboid {
            position: Vec3 {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            rotation: Quaternion::from_angle(
                &Vec3 {
                    x: 0.0,
                    y: 1.0,
                    z: 0.0,
                },
                std::f32::consts::FRAC_PI_2, // 90 degrees CCW around y axis
            ),
            ..Default::default()
        };

        let raw = cuboid.to_raw();

        #[rustfmt::skip]
        let expected_model = [
            0.0, 0.0, -1.0, 0.0, //column major
            0.0, 1.0, 0.0, 0.0,
            1.0, 0.0, 0.0, 0.0,
            1.0, 2.0, 3.0, 1.0,
        ];
        dbg!(raw.model);
        for (expected, actual) in expected_model.iter().zip(raw.model.iter()) {
            assert!((expected - actual).abs() < EPSILON);
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct Cuboid {
    pub index: usize,
    pub position: Vec3, //centre
    pub rotation: Quaternion,
    pub velocity: Vec3,         //ms^-1
    pub angular_velocity: Vec3, //rads^-1
    pub scale: Vec3,            //local
    pub corners: [Vec3; 8],
    pub aabb: AABB,
    pub frozen: bool,
    pub face_axes: [Vec3; 3],
    pub density: f32,
}
impl Cuboid {
    pub fn update_derived(&mut self) {
        self.calc_corners();
        self.calc_aabb();
        self.calc_face_axes();
    }
    pub fn get_all_face_axes(&self) -> [Vec3; 6] {
        [
            self.face_axes[0],
            self.face_axes[1],
            self.face_axes[2],
            -self.face_axes[0],
            -self.face_axes[1],
            -self.face_axes[2],
        ]
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
        let local = Mat3 {
            array: [
                1.0/ixx,0.0,0.0,
                0.0,1.0/iyy,0.0,
                0.0,0.0,1.0/izz
            ]
        };
        let rotation_mat=self.rotation.to_mat3();
        let transpose=rotation_mat.transpose();
        rotation_mat*local*transpose //OPTIMIZE: cache in update_derived()

    }
    fn calc_face_axes(&mut self) {
        self.face_axes = [
            GLOBAL_AXES[0].rotate(self.rotation).normalize().unwrap(),
            GLOBAL_AXES[1].rotate(self.rotation).normalize().unwrap(),
            GLOBAL_AXES[2].rotate(self.rotation).normalize().unwrap(),
        ];
    }
    pub fn get_inverse_mass(&self) -> f32 {
        if self.frozen {
            0.0
        } else {
            1.0 / (self.scale.mag() * self.density)
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
                    rotation_matrix.array[0]*self.scale.x, rotation_matrix.array[1]*self.scale.x, rotation_matrix.array[2]*self.scale.x, 0.0,
                    rotation_matrix.array[3]*self.scale.y, rotation_matrix.array[4]*self.scale.y, rotation_matrix.array[5]*self.scale.y, 0.0,
                    rotation_matrix.array[6]*self.scale.z, rotation_matrix.array[7]*self.scale.z, rotation_matrix.array[8]*self.scale.z, 0.0,
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
            density: 1.0,
        }
    }
}
#[derive(Debug, Default, Copy, Clone)]
pub struct AABB {
    pub min: Vec3,
    pub max: Vec3,
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
