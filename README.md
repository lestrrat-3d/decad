# decad

<p align="center">
  <img src="docs/images/hero.gif" alt="DECAD letters assemble on a navy plate, a light sweeps across them, the logo holds, and the letters lift away" width="900">
</p>

A **headless CAD system** for Go: the 3D modeling layer above the
[sketch](https://github.com/lestrrat-3d/sketch) 2D constraint engine and the
[r3](https://github.com/lestrrat-3d/r3) coordinate-math layer.

> **Work in progress.** The API and supported capabilities may change.

## Why this exists

When a designer wanted a parametric part, they worked in a CAD app. They drew
sketches, gave them dimensions, and added features such as extrusions and cuts.
If a dimension changed, the app rebuilt the part from those steps. The designer
could inspect the result and adjust the model in the same place. That workflow
still works.

Coding agents can now make convincing 3D shapes. Some are good enough for a
one-off render or print. But if you ask for a bevel gear built to given
dimensions, a shape that looks like a gear is only the start. Its teeth have
to mesh with its mate. A fresh request to the agent may also produce different
geometry. Variation may be fine for a one-off model; it makes a dimensioned
part hard to reproduce.

With [decad](https://github.com/lestrrat-3d/decad), a Go program builds and
checks a CAD model without a CAD app. It defines dimensioned sketches and
ordered modeling operations, then builds 3D bodies from them. decad can
tessellate those bodies for rendering. The program can measure volume and
centroid, check whether bodies interfere or have enough clearance, and ask
whether a wall is too thin for a cutting tool. An agent can change a dimension
or feature, run the program again, and inspect the new body and verification
report. With the program and inputs held fixed, **the same model can be rebuilt**
without a fresh request to the agent.

An agent can also use decad to verify a construction before writing an add-in
that builds it in Autodesk Fusion or another CAD app. That route may be more
practical when rebuilding the model in decad is too slow for the intended use.
decad's performance may improve, but matching those apps' speed is not its
primary goal. The agent checks the part in the app too, because it may
interpret the steps differently.

For certified shape and motion paths, decad can detect a collision and compute
the resulting rigid-body motion. The [current support guide](docs/collision-v1-support.md)
names those paths, and the [box collision example](examples/dynamics_box_collision_example_test.go)
runs an impact and reports the rebound.

## What it builds

Each animation shows intermediate decad bodies rendered from the triangle
meshes decad tessellates.

<table>
<tr>
<td>
<img src="docs/images/features/extrude.gif" alt="An L-shaped profile rises into an angle bracket" width="320"><br>
<strong>Extrude</strong> sweeps a solved 2D profile straight into a solid.
</td>
<td>
<img src="docs/images/features/revolve.gif" alt="A circular profile turns around an axis into a ring" width="320"><br>
<strong>Revolve</strong> spins a profile about an axis, so a curved generator gives a curved solid.
</td>
</tr>
<tr>
<td>
<img src="docs/images/features/sweep.gif" alt="One square section rises, turns, extends, and turns again" width="320"><br>
<strong>Sweep</strong> carries one profile along a path with two rounded bends and a straight run.
</td>
<td>
<img src="docs/images/features/loft.gif" alt="A transition duct grows between two offset rectangles" width="320"><br>
<strong>Loft</strong> rules a wall between two profiles on different planes.
</td>
</tr>
<tr>
<td>
<img src="docs/images/features/freeform.gif" alt="Two curved spline edges form a leaf-shaped section before it rises into a solid" width="320"><br>
<strong>Free-form profiles</strong> shape a section with fit splines; Extrude carries those curves into the solid walls.
</td>
<td>
<img src="docs/images/features/fillet.gif" alt="A plate's four upright edges gradually round over" width="320"><br>
<strong>Fillet</strong> rounds selected edges into tangent arcs.
</td>
</tr>
<tr>
<td>
<img src="docs/images/features/chamfer.gif" alt="A plate's four upright edges become flat bevels" width="320"><br>
<strong>Chamfer</strong> cuts selected edges back to a straight bevel.
</td>
<td>
<img src="docs/images/features/cap-chamfer.gif" alt="A plate's top rim gains a bevel" width="320"><br>
<strong>Cap-loop chamfer</strong> bevels a whole rim, the lead-in a bore or a lid needs.
</td>
</tr>
<tr>
<td>
<img src="docs/images/features/shell.gif" alt="A shallow cavity widens into an open tray" width="320"><br>
<strong>Shell</strong> hollows a solid into a wall of one thickness.
</td>
<td>
<img src="docs/images/features/boolean.gif" alt="Three fixed cylinders cut a central bore and two bolt holes into a box" width="320"><br>
<strong>Union, Cut and Intersect</strong> combine two bodies explicitly, never folded into a feature.
</td>
</tr>
<tr>
<td>
<img src="docs/images/features/verify.png" alt="A pin standing inside a larger bore with clearance all around" width="320"><br>
<strong>Verify</strong> reports whether bodies interfere and measures their gap when clearance is requested.
</td>
<td>
<img src="docs/images/features/surface.gif" alt="A curved open dish grows as its profile revolves" width="320"><br>
<strong>Surface result</strong> keeps a feature's swept walls and omits the faces that exist only to close the solid,
leaving a sheet body.
</td>
</tr>
</table>

Regenerate the gallery GIFs with `cd _gallery && go run . features` (requires
`ffmpeg`). The still images can be regenerated with `go run .`. From `_gallery`,
regenerate the animated hero with `go run . hero > hero-assemble.sh && sh hero-assemble.sh`.
Its letters assemble, a light crosses them, the finished logo holds for ten
seconds, and the letters lift away before the loop repeats.

The landing-page clip animates several gallery parts, a curved duct, and the wordmark. Render it with
`cd _gallery && go run . clip > assemble.sh && sh assemble.sh`: the program
writes each shot's PNG frames under `_gallery/out/` and prints the two ffmpeg
commands, which the script runs to write `out/decad-landing.mp4` and
`out/decad-landing.gif`.

## Motion and collisions

decad also checks parts that move. `Document.VerifyMotion` moves bodies along one
revolute, prismatic or `Between` path. `Document.VerifyLinkage` drives a chain of
joints, including a closed loop such as a four-bar or a slider-crank.
`Document.VerifyJointBox` checks every combination of joint values in a box.
Each check splits the motion into intervals, or the box into cells, and reports
each one as clear, colliding, or undecided. For a clear interval, decad proves
that no two bodies overlap at any point in it, not only at sampled poses, and
reports a lower bound on the gap. A colliding interval has a proven, measured
overlap at one of its ends. An undecided interval makes the report `Suspect`, and
decad does not count it as clear.

The `dynamics` package simulates rigid bodies under gravity, impact and
friction. A `dynamics.Timeline` advances a scene one fixed step at a time. A step
that its proofs cannot settle returns `Undecided` with a typed reason, and the
timeline stops there. Every frame below is a pose the check or the timeline
computed. The two linkage clips drive each mechanism out to the last pose the
check proves clear, just short of its first proven collision, and back. The
link and the stop it would hit flash coral while the clip pauses there.

<table>
<tr>
<td>
<img src="docs/images/motion/arm.gif" alt="A two-link arm pinned to a post on a base plate folds up to a stop block, the forearm and the block flash coral, and the arm folds back" width="320"><br>
<strong>VerifyLinkage</strong> proves the folding arm clear up to s = 91/256, with the forearm all but
on the stop: the exact contact is at s = asin(17/32)/90° ≈ 0.3566 and the first proven collision at
92/256. The clip drives to 91/256 and back. At every pose the check evaluates, each pin sits on
its bore's centre and measures 0.5 mm clear of it.
</td>
<td>
<img src="docs/images/motion/rocker.gif" alt="A crank drives a pinned four-bar linkage until the follower reaches a stop block, the follower and the block flash coral, and the crank turns back" width="320"><br>
<strong>Closed loops</strong> are checked too: VerifyLinkage proves the crank-rocker clear up to
s = 118/256 (crank angle 165.94°), just before the follower's flank reaches the stop at a crank
angle of 167.62°; its first proven collision is at 120/256. The clip drives to 118/256 and back.
All four pins sit on their bores' centres and measure 0.5 mm clear of them at every pose the
check evaluates.
</td>
</tr>
<tr>
<td>
<img src="docs/images/motion/tumble.gif" alt="Spinning boxes, a hexagonal prism, a wedge and a tetrahedron drop into a tray and settle" width="320"><br>
<strong>Tumble</strong> runs all 768 steps of its 3 s timeline, and every body ends at rest on a face,
with at least three vertices within 10 µm of the tray floor.
</td>
<td>
<img src="docs/images/motion/parts-bin.gif" alt="Five modeled parts drop into a tray and settle while a cylinder rolls across the floor" width="320"><br>
<strong>Parts bin</strong> drops a shelled cup, a chamfered block, a loft, a swept hexagon and a
revolved bottle to rest, while a cylinder rolls 60 mm without slipping.
</td>
</tr>
</table>

Regenerate these GIFs with `cd _gallery && go run . motion` (requires `ffmpeg`).
The same scenes render as full-size PNG frames under `_gallery/out/` with
`go run . linkage`, `go run . linkage -scene rocker`, and
`go run . dynamics -scene tumble` or `-scene parts-bin`. A third dynamics scene,
`-scene stack-and-drop`, drops spheres and a cylinder beside a resting box
pyramid.

## Layering

```
decad    3D bodies, features, verification   (this module)
  |
sketch   parametric 2D constraint solving    github.com/lestrrat-3d/sketch
  |
r3       vectors, frames, rigid transforms   github.com/lestrrat-3d/r3
  |
units    typed quantities (Value, Kind)      github.com/lestrrat-3d/units
```

The arrows point **down and never back up**. decad imports `sketch`, `r3` and
[`units`](https://github.com/lestrrat-3d/units); none of them knows decad
exists.

This is the layer both of them deliberately left room for. `r3` excludes shapes
by charter — *"if it lives in ℝ³ it belongs here; if it **is** a shape, it does
not"* — and `sketch` excludes anything that must be computed **from** a solid,
consuming 3D-derived geometry only as first-class reference geometry it is
*given*. decad is what sits on the other side of that seam.

A 2D question — does this profile close, is this sketch fully constrained — is
`sketch`'s to answer, and decad consumes the answer rather than re-deriving it.

## License

This project is **source-available**, and is licensed under the
[PolyForm Noncommercial License 1.0.0](LICENSE).

* **Noncommercial use is free.** Individuals, hobby and personal projects,
  research, education, nonprofits, and government may use, modify, and
  redistribute it at no cost, subject to the license terms.
* **Commercial / business use requires a separate license.** Any use by or for
  a business, or for commercial advantage, is not permitted under the
  noncommercial license. To obtain a commercial license, reach out on Bluesky
  at [@lestrrat.bsky.social](https://bsky.app/profile/lestrrat.bsky.social).

### Contributions

This repository does **not** accept external pull requests.
