use ncollide2d::shape::{ConvexPolygon, ShapeHandle};

//use nalgebra as na;
use nphysics2d::math::{Isometry, Point, Vector};
use nphysics2d::object::Material;
use nphysics2d::volumetric::Volumetric;
use nphysics2d::world::World;

use std::{thread, time};

fn main() {
    let mut world = World::new();
    world.set_gravity(Vector::y() * 9.81);

    let polyline = ShapeHandle::new(
        ConvexPolygon::try_new(vec![
            Point::new(1.0, 2.0),
            Point::new(2.0, 2.0),
            Point::new(2.0, 3.0),
            Point::new(1.0, 3.0),
        ]).unwrap(),
    );
    let local_inertia = polyline.inertia(0.1);
    let local_center_of_mass = polyline.center_of_mass();
    let rigid_body_handle = world.add_rigid_body(
        Isometry::new(Vector::new(2.0, 10.0), 3.0),
        local_inertia,
        local_center_of_mass,
    );

    // Create a collider attached to a previously-added rigid-body with handle `rigid_body_handle`.
    let material = Material::default(); // Custom material.
    let collider_handle = world.add_collider(
        0.04,
        polyline,
        rigid_body_handle,
        Isometry::identity(),
        material,
    );

    loop {
        world.step();
        let collider = world
            .collider(collider_handle)
            .expect("Collider handle was invalid");
        println!("{}", collider.position());
        let shape: &ConvexPolygon<_> = collider
            .shape()
            .as_shape()
            .expect("Failed to cast shapehandle to a ConvexPolygon");
        for vertex in shape.points() {
            println!("{}", vertex);
        }
        thread::sleep(time::Duration::from_millis(1000 / 60));
    }
}
