use criterion::{black_box, criterion_group, criterion_main, Criterion};
use digital_twin::prelude::*;
use digital_twin_glue::prelude::*;
use nalgebra::Vector3;
use uom::si::f64::Length;
use uom::si::length::meter;

fn bench_step(c: &mut Criterion) {
    let config = get_default_config();
    let state = get_initial_state(&config);
    let mesh_generator = TheMeshGenerator::default();

    let target_vec = Vector3::new(1000.0, 1000.0, 1000.0);
    let waypoint = Some(target_vec.map(|c| Length::new::<meter>(c)));

    let rocket = TheRocket::new(config.clone(), waypoint);
    let mut twin = DigitalTwin::new(config, state, Box::new(mesh_generator), Box::new(rocket));

    // Warm up the twin
    twin.step(0.01);
    
    let mut group = c.benchmark_group("digital_twin");
    group.bench_function("step_0_01", |b| {
        b.iter(|| {
            twin.step(black_box(0.01));
            // Just mutate the time back manually to avoid state explosion during bench
            twin.state.time = uom::si::f64::Time::new::<uom::si::time::second>(0.0);
        })
    });
    group.finish();
}

criterion_group!(benches, bench_step);
criterion_main!(benches);
