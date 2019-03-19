## Sample a scene, wait until it's stablized, return the object poses.
```
$ python scripts/sample_final_bullet_scene --config-file configs/tless_scenes.yaml
```

<!-- bulletwrapper
=============

**bulletwrapper is a set of extensions to pybullet (https://pybullet.org/). It includes interface classes and utility functions.**

Interface
---------

bulletwrapper provides openai-gym style interface of pybullet.

```python
sim = BulletSimulator()
sim.reset()

while True:
    sim.step()
```

bulletwrapper uses various hooks for adding dynamic components and cameras.

```python
r2d2_adder = R2D2AdderHook()
final_image_capturer = FinalImageCapturerHook()

sim = BulletSimulator(
        hooks=[
            r2d2_adder,
            final_image_capturer,
        ],

        max_sim_time = 5.,
    )

sim.reset()

while True:
    try:
        observations = sim.step()
    except StopSimulation:
        break;
```



 -->
