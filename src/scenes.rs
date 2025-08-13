use std::f32::consts::PI;

use crate::{
    math::{Quaternion, Vec3},
    world::Cuboid,
};

pub const N: usize = 8 * 8 * 8; //number of cuboids minus floor

//TODO: take in cmd args from cargo run and use that to set N and Scene, then i can easily
//reproduce scenes with the same command

pub enum Scene {
    Grid,
    SlantedTower, //stacking is visible at low N like 9
    Meteor,
    InvertedMeteor,
    Catapult,
    Sticks,
    Platforms,
    Cube,
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
                            i as f32 * 0.5 / 9.0,
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
            Scene::Catapult => {
                assert_eq!(N % 4, 0, "N for catapult must be multiple of 4");
                const INSTANCE_SPACING: f32 = 2.0;
                let mut i = 0;
                while i < N / 4 * 4 {
                    let x_coord = i as f32 * INSTANCE_SPACING;
                    let mut fulcrum = Cuboid {
                        position: Vec3 {
                            x: x_coord,
                            y: 0.5,
                            z: 0.0,
                        },
                        scale: Vec3 {
                            x: 1.0,
                            y: 1.0,
                            z: 0.25,
                        },
                        frozen: true,
                        index: i,
                        ..Default::default()
                    };
                    fulcrum.update_derived();
                    instances.push(fulcrum);
                    i += 1;
                    let mut lever = Cuboid {
                        position: Vec3 {
                            x: x_coord,
                            y: 2.0,
                            z: -0.25,
                        },
                        scale: Vec3 {
                            x: 1.0,
                            y: 0.5,
                            z: 4.0,
                        },
                        rotation: Quaternion::from_angle(
                            &Vec3 {
                                x: 1.0,
                                y: 0.0,
                                z: 0.0,
                            },
                            PI / 6.0,
                        ),
                        index: i,
                        ..Default::default()
                    };
                    i += 1;
                    lever.update_derived();
                    instances.push(lever);
                    let mut projectile = Cuboid {
                        position: Vec3 {
                            x: x_coord,
                            y: 2.5,
                            z: -0.25,
                        },
                        scale: Vec3 {
                            x: 0.25,
                            y: 0.25,
                            z: 0.25,
                        },
                        index: i,
                        ..Default::default()
                    };
                    i += 1;
                    projectile.update_derived();
                    instances.push(projectile);

                    let mut heavy = Cuboid {
                        position: Vec3 {
                            x: x_coord,
                            y: 12.0,
                            z: 0.9,
                        },
                        scale: Vec3 {
                            x: 1.0,
                            y: 1.0,
                            z: 1.0,
                        },
                        density: 3.0,
                        index: i,
                        ..Default::default()
                    };
                    i += 1;
                    heavy.update_derived();
                    instances.push(heavy);
                }
            }
            Scene::Sticks => {
                Self::gen_grid(
                    instances,
                    4.0,
                    Vec3 {
                        x: 1.0,
                        y: 4.0,
                        z: 1.0,
                    },
                );
            }
            Scene::Platforms => {
                Self::gen_grid(
                    instances,
                    1.5,
                    Vec3 {
                        x: 4.0,
                        y: 1.0,
                        z: 4.0,
                    },
                );
            }
            Scene::Cube => {
                let len = (N as f32).cbrt() as usize;
                const INSTANCE_SPACING: f32 = 2.0;
                for i in 0..len {
                    for j in 0..len {
                        for k in 0..len {
                            let index = (i * len + j) * len + k;
                            let mut instance = Cuboid {
                                position: Vec3 {
                                    x: i as f32 * INSTANCE_SPACING * 1.0,
                                    y: 10.0 + j as f32 * INSTANCE_SPACING * 1.0,
                                    z: k as f32 * INSTANCE_SPACING * 1.0,
                                },
                                scale: Vec3 {
                                    x: 1.0,
                                    y: 1.0,
                                    z: 1.0,
                                },
                                index,

                                rotation: Quaternion::from_angle(
                                    &Vec3 {
                                        x: 0.0,
                                        y: 0.0,
                                        z: 1.0,
                                    },
                                    index as f32 * 2.0 / N as f32,
                                ),
                                ..Default::default()
                            };
                            instance.update_derived();
                            instances.push(instance);
                        }
                    }
                }
            }
        }
    }
}
