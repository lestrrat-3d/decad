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

With [decad](https://github.com/lestrrat-3d/decad), a Go program defines
dimensioned sketches and ordered modeling operations, then builds 3D bodies
without a CAD app. decad can tessellate those bodies for rendering. The program
can measure volume and centroid, check whether bodies interfere or have enough
clearance, and ask whether a wall is too thin for a cutting tool. An agent can
change a dimension or feature, run the program again, and inspect the new body
and verification report. With the program and inputs held fixed, **the same
model can be rebuilt** without a fresh request to the agent.

That Go program can be the CAD workflow in its own right. decad's performance
still needs work, but matching the speed of Autodesk Fusion or other CAD apps is
not its primary goal. If a workflow needs the speed of one of those apps, an
agent can develop and verify the construction in decad before implementing it
as an add-in there. The agent checks the part in the app too, because it may
interpret the steps differently.

For certified shape and motion paths, decad can detect a collision and compute
the resulting rigid-body motion. The [current support guide](docs/collision-v1-support.md)
names those paths, and the [box collision example](examples/dynamics_box_collision_example_test.go)
runs an impact and reports the rebound.

## What it builds

Every part below is a decad body, rendered from the triangle mesh decad itself
tessellates.

| | |
|---|---|
| <img src="docs/images/features/extrude.png" alt="An L-shaped angle bracket, one sketched section swept straight upward" width="320"><br>**Extrude** sweeps a solved 2D profile straight into a solid. | <img src="docs/images/features/revolve.png" alt="A flat ring, the solid swept by a circle offset from the axis" width="320"><br>**Revolve** spins a profile about an axis, so a curved generator gives a curved solid. |
| <img src="docs/images/features/sweep.png" alt="A square duct following a 3D path with bends in two orthogonal planes" width="320"><br>**Sweep** moves one solved profile along a tangent 3D path through multiple bend planes. | <img src="docs/images/features/loft.png" alt="A transition duct narrowing from a large rectangle to a smaller offset one" width="320"><br>**Loft** rules a wall between two profiles on different planes. |
| <img src="docs/images/features/freeform.png" alt="A blade section whose curved front wall is a spline, extruded into a solid" width="320"><br>**Free-form profiles** carry spline walls, measured exactly rather than approximated. | <img src="docs/images/features/fillet.png" alt="A rectangular plate whose four upright edges are rounded" width="320"><br>**Fillet** rounds selected edges into tangent arcs. |
| <img src="docs/images/features/chamfer.png" alt="A rectangular plate whose four upright edges are cut back to flat bevels" width="320"><br>**Chamfer** cuts selected edges back to a straight bevel. | <img src="docs/images/features/cap-chamfer.png" alt="A rectangular plate whose whole top rim is bevelled" width="320"><br>**Cap-loop chamfer** bevels a whole rim, the lead-in a bore or a lid needs. |
| <img src="docs/images/features/shell.png" alt="An open tray: a block with its top face removed and its walls left one thickness" width="320"><br>**Shell** hollows a solid into a wall of one thickness. | <img src="docs/images/features/boolean.png" alt="A flange plate with one large central bore and two smaller bolt holes drilled through it" width="320"><br>**Union, Cut and Intersect** combine two bodies explicitly, never folded into a feature. |
| <img src="docs/images/features/verify.png" alt="A round pin standing inside a larger bore, clearance visible all the way round" width="320"><br>**Verify** proves the gap between two bodies, so a fit is checked before anything is cut. | <img src="docs/images/features/surface.png" alt="A curved open dish of no thickness, its inner side shaded apart from its outer one" width="320"><br>**Surface result** keeps a feature's swept walls and omits the faces that exist only to close the solid, leaving a sheet body. |

Regenerate the still images with `cd _gallery && go run .`. From `_gallery`,
regenerate the animated hero with `go run . hero > hero-assemble.sh && sh hero-assemble.sh`.
Its letters assemble, a light crosses them, the finished logo holds for ten
seconds, and the letters lift away before the loop repeats.

The landing-page clip animates these parts and the wordmark. Render it with
`cd _gallery && go run . clip > assemble.sh && sh assemble.sh`: the program
writes each shot's PNG frames under `_gallery/out/` and prints the two ffmpeg
commands, which the script runs to write `out/decad-landing.mp4` and
`out/decad-landing.gif`.

The `stack-and-drop` dynamics scene drops spheres and a cylinder beside a
resting box pyramid, every frame a certified pose of a `dynamics.Timeline`.
Render it with `cd _gallery && go run . dynamics -scene stack-and-drop`; the
frames go under `_gallery/out/`. The `tumble` scene drops spinning boxes, a
hexagonal prism, a wedge and a stitched tetrahedron into a tray, where each
lands on a corner and comes to rest face down; `-scene tumble` renders it.
The `parts-bin` scene drops a shelled cup, a chamfered block, a loft, a swept
hexagon and a revolved bottle into the same tray, where each comes to rest,
while a cylinder rolls across its floor; `-scene parts-bin` renders it.

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
