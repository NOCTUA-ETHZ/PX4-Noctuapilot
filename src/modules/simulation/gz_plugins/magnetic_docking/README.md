# Magnetic docking Gazebo plugin

`MagneticDockingSystem` applies equal and opposite forces between two
configurable points on two Gazebo links. It is a world plugin and contains no
PX4 flight-control, contact, latching, or release logic.

The force magnitude uses the finite-at-contact closed-form approximation from
Zurek (2022) for two identical, coaxial, axially magnetized cylindrical
magnets:

```text
xc = x + 0.8 R
F(x) = pi Br^2 R^4 / (4 mu0)
       * [1/xc^2 + 1/(xc + 2L)^2 - 2/(xc + L)^2]
```

Here `x` is the face-to-face gap, `R` is magnet radius, `L` is magnet length,
`Br` is remanent flux density, and `mu0` is the permeability of free space.
Unlike a point-dipole law, this approximation remains finite at contact while
retaining the expected rapid decay at larger distances.

The published law assumes coaxial cylinders with parallel magnetization. In
the simulation, force is directed along the line joining the configured face
centres and scaled by the dot product of their axes. This gives a simple
capture-volume approximation for small lateral or angular errors, but is not
an exact off-axis magnetic solution. Axial damping is a numerical stabilizer;
it is weighted by `F(x)/F(0)` so that it vanishes with the magnetic force.
Forces are applied at the magnet offsets, allowing Gazebo to produce moments
from off-centre forces. No latch or artificial alignment torque is added.

## SDF configuration

```xml
<plugin filename="libMagneticDockingSystem.so"
        name="custom::MagneticDockingSystem">
  <model_a>fixed_magnet</model_a>
  <link_a>magnet_link</link_a>
  <offset_a>0 0 -0.055</offset_a>
  <axis_a>0 0 1</axis_a>
  <model_b>test_body</model_b>
  <link_b>body_link</link_b>
  <offset_b>0 0 0.065</offset_b>
  <axis_b>0 0 1</axis_b>
  <capture_distance>0.15</capture_distance>
  <magnet_radius>0.015</magnet_radius>
  <magnet_length>0.005</magnet_length>
  <remanence>0.8</remanence>
  <force_scale>1.0</force_scale>
  <damping>3.0</damping>
  <enabled>true</enabled>
  <control_topic>/magnetic_docking/enable</control_topic>
</plugin>
```

`model_a`, `link_a`, `model_b`, and `link_b` are required. Offsets default to
zero and axes default to local +Z. The representative physical parameters are
30 mm diameter, 5 mm length, and 0.8 T remanence. `force_scale` should normally
remain 1.0, but can later calibrate the approximation to measured EPM data.
`capture_distance` is only a computational cutoff. The magnet is enabled by
default.

For the representative values, the calculated attraction is approximately
42.3 N at contact, 26.3 N at 2 mm, 6.1 N at 10 mm, and 0.004 N at 150 mm.

## Switching the magnet

While Gazebo is running, disable the force with:

```sh
gz topic -t /magnetic_docking/enable -m gz.msgs.Boolean -p 'data: false'
```

Enable it again with:

```sh
gz topic -t /magnetic_docking/enable -m gz.msgs.Boolean -p 'data: true'
```

The topic name and initial state can be changed using `<control_topic>` and
`<enabled>` in the plugin's SDF configuration. The most recently received
Boolean value remains active until another value is published or the world is
restarted.

## Build and run the standalone test

From the PX4 repository root:

```sh
make px4_sitl_default
cmake --build build/px4_sitl_default --target px4_gz_plugins
GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build/px4_sitl_default/src/modules/simulation/gz_plugins${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}" \
  gz sim -r src/modules/simulation/gz_plugins/magnetic_docking/test/magnetic_docking.sdf
```

The test uses zero gravity. A red dynamic body starts inside the capture range
of a blue static magnet and should be pulled towards it. Pause and reset the
world in the Gazebo GUI to repeat the test.

## X500 and overhead platform scenario

The `x500_magnet` model extends the basic X500 with a 30 mm diameter magnet
on top. The `magnetic_docking` world contains a 2 m square static platform
centred 5 m above the ground, with a matching magnet mounted on its underside.

Run the PX4 SITL scenario with:

```sh
make px4_sitl gz_x500_magnet_magnetic_docking
```

The X500 magnet's upper face is 0.065 m above `base_link`. The platform
magnet's lower face is at 4.945 m. The plugin evaluates the force while the two
faces are less than 0.15 m apart.

## Model choice and sources

The point-dipole model was rejected for near-contact docking because it treats
the magnets as dimensionless and is valid only when their separation is large
relative to their size. Vokoun et al. explicitly identify shape effects as
necessary near contact and present an accurate cylindrical-magnet solution,
but that solution requires Bessel or elliptic-integral evaluation. Zurek's
closed-form correction was chosen as a practical middle ground: it uses magnet
geometry and remanence, remains finite at contact, is inexpensive enough for
every simulation step, and was evaluated using NdFeB, ferrite, and SmCo magnets
over contact forces from 0.2 N to 250 N.

- S. Zurek, “Performance of closed-form equations for force between
  cylindrical magnets over wide range of volume, aspect ratio, and force,”
  *Journal of Electrical Engineering*, 73(6), 405–412, 2022.
  https://doi.org/10.2478/jee-2022-0055
- D. Vokoun, M. Beleggia, L. Heller, and P. Sittner, “Magnetostatic
  interactions and forces between cylindrical permanent magnets,” *Journal of
  Magnetism and Magnetic Materials*, 321(22), 3758–3763, 2009.
  https://doi.org/10.1016/j.jmmm.2009.07.030
- J. S. Agashe and D. P. Arnold, “A study of scaling and geometry effects on
  the forces between cuboidal and cylindrical magnets using analytical force
  solutions,” *Journal of Physics D: Applied Physics*, 41, 105001, 2008.
  https://doi.org/10.1088/0022-3727/41/10/105001
