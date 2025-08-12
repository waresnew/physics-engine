use crate::{
    math::{Quaternion, Vec3},
    world::Cuboid,
};

pub const N: usize = 9; //number of cuboids minus floor
const INSTANCE_SPACING: f32 = 2.0;

//TODO: take in cmd args from cargo run and use that to set N and Scene, then i can easily
//reproduce scenes with the same command
pub enum Scene {
    Grid,
    SlantedTower,
    //TODO: non unit scale (thin/long/fat prisms), big meteor cube falling into a bunch of smaller ones that are already
    //on the floor, small cube falling onto big cube, pile of non unit scale prisms on ground whiel
    //i drop more cubes on them, dropping cubes on pyramid or leaning tower structure (those are
    //already on the ground) and watch
    //them collapse
}

impl Scene {
    pub fn populate_scene(&self, instances: &mut Vec<Cuboid>) {
        match self {
            Scene::Grid => {
                let num_cols = N.isqrt();
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

                    let mut instance = Cuboid {
                        position,
                        scale,
                        index: i,
                        rotation: Quaternion::from_angle(
                            &Vec3 {
                                x: 1.0,
                                y: 0.0,
                                z: 1.0,
                            },
                            i as f32 * 0.5 / N as f32,
                        ),
                        ..Default::default()
                    };
                    instances.push(instance);
                    instance.update_derived();
                }
            }
            Scene::SlantedTower => {
                for i in 0..N {
                    let scale = Vec3 {
                        x: 1.0,
                        y: 1.0,
                        z: 1.0,
                    };
                    let position = Vec3 {
                        x: i as f32 * 0.1,
                        y: 10.0 + i as f32 * INSTANCE_SPACING,
                        z: 0.0,
                    };

                    let mut instance = Cuboid {
                        position,
                        scale,
                        rotation: Quaternion::from_angle(
                            &Vec3 {
                                x: 1.0,
                                y: 0.0,
                                z: 0.0,
                            },
                            i as f32 * 0.5 / N as f32,
                        ),
                        index: i,
                        ..Default::default()
                    };
                    instances.push(instance);
                    instance.update_derived();
                }
            }
        }
    }
}
