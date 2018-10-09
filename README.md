bulletwrapper
=============

**bulletwrapper is a set of extensions of pybullet (https://pybullet.org/). It includes interface classes and utility functions.**

Interface
---------

bulletwrapper provides openai-gym style interface of pybullet. E.g.,

```python
sim = BulletSimulator()
sim.reset()

while True:
    sim.step()
```

bulletwrapper uses various hooks for adding dynamic components and cameras.

```python
r2d2_adder = R2D2AdderHook()
final_image_capturer = FinalImageCapturer()

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



