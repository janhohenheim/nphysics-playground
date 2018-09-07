use ncollide2d::shape::{Cuboid, ShapeHandle};

use nalgebra as na;
use nphysics2d::math::{Isometry, Vector};
use nphysics2d::object::Material;
use nphysics2d::volumetric::Volumetric;
use nphysics2d::world::World;

use std::{thread, time};

fn main() {
    let mut world = World::new();
    world.set_gravity(Vector::new(0.0, 0.0));

    let cuboid = ShapeHandle::new(Cuboid::new(Vector::new(1.0, 2.0)));
    let local_inertia = cuboid.inertia(0.1);
    let local_center_of_mass = cuboid.center_of_mass();
    let rigid_body_handle = world.add_rigid_body(
        Isometry::new(Vector::new(2.0, 10.0), na::zero()),
        local_inertia,
        local_center_of_mass,
    );

    // Create a collider attached to a previously-added rigid-body with handle `rigid_body_handle`.
    let material = Material::default(); // Custom material.
    let _collider_handle = world.add_collider(
        0.04,
        cuboid,
        rigid_body_handle,
        Isometry::identity(),
        material,
    );

    loop {
        world.step();
        let collider = world.collider(_collider_handle).expect("The unexpected");
        println!("{:#?}", collider.position());

        thread::sleep(time::Duration::from_millis(1000));
    }
}
