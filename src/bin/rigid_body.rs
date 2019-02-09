use ncollide2d::shape::{ConvexPolygon, ShapeHandle};
use nphysics2d::algebra::ForceType;
use nphysics2d::material::{BasicMaterial, MaterialHandle};
use nphysics2d::math::{Force, Isometry, Point, Vector};
use nphysics2d::object::{Body, ColliderDesc, RigidBodyDesc};
use nphysics2d::volumetric::Volumetric;
use nphysics2d::world::World;
use std::{thread, time};

fn main() {
    let mut world = World::new();

    let polygon = ShapeHandle::new(
        ConvexPolygon::try_new(vec![
            Point::new(1.0, 2.0),
            Point::new(2.0, 2.0),
            Point::new(2.0, 3.0),
            Point::new(1.0, 3.0),
        ])
        .unwrap(),
    );
    let local_inertia = polygon.inertia(0.1);
    let local_center_of_mass = polygon.center_of_mass();
    let rigid_body_handle = RigidBodyDesc::new()
        .position(Isometry::new(Vector::new(2.0, 10.0), 3.0))
        .local_inertia(local_inertia)
        .local_center_of_mass(local_center_of_mass)
        .mass(100.0)
        .build(&mut world)
        .part_handle();

    let material = MaterialHandle::new(BasicMaterial::default());

    let collider_handle = ColliderDesc::new(polygon.clone())
        .margin(0.04)
        .position(Isometry::identity())
        .material(material.clone())
        .build_with_parent(rigid_body_handle, &mut world)
        .unwrap()
        .handle();

    loop {
        world.step();
        let collider = world
            .collider(collider_handle)
            .expect("Collider handle was invalid");

        println!("{}", collider.position());

        let body_handle = world.collider_body_handle(collider_handle).unwrap();
        let body = world.rigid_body_mut(body_handle).unwrap();
        let force = Force::from_slice(&[5.00, 2.00, 0.5]);
        body.apply_force(body.part_handle().1, &force, ForceType::Force, true);

        thread::sleep(time::Duration::from_millis(1000 / 60));
    }
}
