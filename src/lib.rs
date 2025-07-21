pub mod math;
pub mod window;

#[repr(C)]
#[derive(Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Vertex {
    position: [f32; 3],
    colour: [f32; 3],
}

impl Vertex {
    const ATTRIBUTES: [wgpu::VertexAttribute; 2] =
        wgpu::vertex_attr_array![0=>Float32x3,1=>Float32x3];
    pub fn desc() -> wgpu::VertexBufferLayout<'static> {
        use std::mem;
        wgpu::VertexBufferLayout {
            array_stride: mem::size_of::<Self>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRIBUTES,
        }
    }
}

pub const CUBE_VERTICES: &[Vertex] = &[
    // z axis is outward, x is right, y is up
    // front face
    Vertex {
        position: [0.5, -0.5, 0.5],
        colour: [1.0, 0.0, 0.0],
    },
    Vertex {
        position: [0.5, 0.5, 0.5],
        colour: [1.0, 0.0, 0.0],
    },
    Vertex {
        position: [-0.5, 0.5, 0.5],
        colour: [1.0, 0.0, 0.0],
    },
    Vertex {
        position: [-0.5, -0.5, 0.5],
        colour: [1.0, 0.0, 0.0],
    },
    // back face
    Vertex {
        position: [-0.5, -0.5, -0.5],
        colour: [1.0, 0.0, 0.0],
    },
    Vertex {
        position: [-0.5, 0.5, -0.5],
        colour: [1.0, 0.0, 0.0],
    },
    Vertex {
        position: [0.5, 0.5, -0.5],
        colour: [1.0, 0.0, 0.0],
    },
    Vertex {
        position: [0.5, -0.5, -0.5],
        colour: [1.0, 0.0, 0.0],
    },
];

#[rustfmt::skip]
pub const CUBE_INDICES: &[u16] = &[
    // each triangle's vertices are defined CCW order if i were to look at it head on
    // front face
    0,1,2,
    2,3,0,
    //back face
    4,5,6,
    6,7,4,
    //top face
    1,6,5,
    5,2,1,
    //bottom face
    7,0,3,
    3,4,7,
    //left face
    3,2,5,
    5,4,3,
    //right face
    7,6,1,
    1,0,7
];
