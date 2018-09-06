use ncollide2d::shape::{Cuboid, ShapeHandle};

use nalgebra as na;
use nphysics2d::math::{Isometry, Vector};
use nphysics2d::object::Material;
use nphysics2d::volumetric::Volumetric;
use nphysics2d::world::World;

fn main() {
    let mut world = World::new();
    world.set_gravity(Vector::y() * -9.81);

    let cuboid = ShapeHandle::new(Cuboid::new(Vector::new(1.0, 2.0)));
    let local_inertia = cuboid.inertia(1.0);
    let local_center_of_mass = cuboid.center_of_mass();
    let rigid_body_handle = world.add_rigid_body(
        Isometry::new(Vector::x() * 2.0, na::zero()),
        local_inertia,
        local_center_of_mass,
    );

    // Create a collider attached to a previously-added rigid-body with handle `rigid_body_handle`.
    let material = Material::new(0.5, 0.3); // Custom material.
    let _collider_handle = world.add_collider(
        0.04,
        cuboid,
        rigid_body_handle,
        Isometry::identity(),
        material,
    );

    loop {
        world.step();
    }
}
