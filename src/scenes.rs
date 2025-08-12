use crate::{
    math::{Quaternion, Vec3},
    world::Cuboid,
};

pub const N: usize = 90; //number of cuboids minus floor

//TODO: take in cmd args from cargo run and use that to set N and Scene, then i can easily
//reproduce scenes with the same command

//TODO: non unit scale (thin/long/fat prisms),  pile of non unit scale prisms on ground whiel i drop more cubes on them, dropping cubes on packed cuboids or leaning tower structure (those are already on the ground) and watch them collapse
pub enum Scene {
    Grid,
    SlantedTower,
    Meteor,
    InvertedMeteor,
}

impl Scene {
    fn gen_grid(instances: &mut Vec<Cuboid>, instance_spacing: f32, scale: Vec3) {
        let num_cols = N.isqrt();
        for i in 0..N {
            let row = i / num_cols;
            let col = i % num_cols;
            let position = Vec3 {
                x: row as f32 * instance_spacing * scale.x,
                y: 10.0,
                z: col as f32 * instance_spacing * scale.z,
            };

            let mut instance = Cuboid {
                position,
                scale,
                index: i,
                rotation: Quaternion::from_angle(
                    &Vec3 {
                        x: 0.0,
                        y: 0.0,
                        z: 1.0,
                    },
                    i as f32 * 2.0 / N as f32,
                ),
                ..Default::default()
            };
            instances.push(instance);
            instance.update_derived();
        }
    }
    fn gen_meteor(
        instances: &mut Vec<Cuboid>,
        little_scale: Vec3,
        big_scale: Vec3,
        inverted: bool,
    ) {
        const INSTANCE_SPACING: f32 = 1.5;
        let num_cols = (N - 1).isqrt();
        for i in 0..(N - 1) {
            let row = i / num_cols;
            let col = i % num_cols;
            let position = Vec3 {
                x: row as f32 * INSTANCE_SPACING * little_scale.x,
                y: if !inverted {
                    little_scale.y / 2.0
                } else {
                    60.0
                },
                z: col as f32 * INSTANCE_SPACING * little_scale.z,
            };

            let mut instance = Cuboid {
                position,
                scale: little_scale,
                index: i,
                rotation: Quaternion::from_angle(
                    &Vec3 {
                        x: 0.0,
                        y: 1.0,
                        z: 0.0,
                    },
                    i as f32 * 0.5 / (N - 1) as f32,
                ),
                ..Default::default()
            };
            instances.push(instance);
            instance.update_derived();
        }
        let mut meteor = Cuboid {
            position: Vec3 {
                x: little_scale.x * INSTANCE_SPACING * num_cols as f32 / 2.0,
                y: if !inverted { 60.0 } else { big_scale.y / 2.0 },
                z: little_scale.z * INSTANCE_SPACING * num_cols as f32 / 2.0,
            },
            scale: big_scale,
            index: N - 1,
            ..Default::default()
        };
        instances.push(meteor);
        meteor.update_derived();
    }
    pub fn populate_scene(&self, instances: &mut Vec<Cuboid>) {
        match self {
            Scene::Grid => {
                Self::gen_grid(
                    instances,
                    2.0,
                    Vec3 {
                        x: 1.0,
                        y: 1.0,
                        z: 1.0,
                    },
                );
            }
            Scene::SlantedTower => {
                const INSTANCE_SPACING: f32 = 2.0;
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
            Scene::Meteor => {
                Self::gen_meteor(
                    instances,
                    Vec3 {
                        x: 0.25,
                        y: 0.25,
                        z: 0.25,
                    },
                    Vec3 {
                        x: 5.0,
                        y: 5.0,
                        z: 5.0,
                    },
                    false,
                );
            }
            Scene::InvertedMeteor => {
                Self::gen_meteor(
                    instances,
                    Vec3 {
                        x: 0.25,
                        y: 0.25,
                        z: 0.25,
                    },
                    Vec3 {
                        x: 5.0,
                        y: 5.0,
                        z: 5.0,
                    },
                    true,
                );
            }
        }
    }
}
