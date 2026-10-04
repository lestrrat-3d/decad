# decad

<p align="center">
  <img src="docs/images/hero.png" alt="Dimensional DECAD lettering rendered from decad solids against a pale blue-gray background" width="900">
</p>

A **headless CAD engine** for Go: the 3D modeling layer above the
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

An agent can also write a CAD add-in that creates dimensioned sketches and
ordered features. The add-in can rebuild the part from the same inputs. The
agent still has to run it in the CAD app to learn whether a sweep failed,
whether the body is watertight, or whether two components collide. A change to
the construction means another run in the app.

With decad, the agent writes and runs that construction in Go before building
the CAD add-in. decad builds 3D bodies from solved sketches and ordered
modeling operations, and can tessellate them for rendering. The agent can
measure volume and centroid, check whether bodies interfere or have enough
clearance, and ask whether a wall is too thin for a cutting tool. It can change
a dimension or feature, run the program again, and inspect the new body and
verification report. With the program and inputs held fixed, the same model
can be rebuilt without a fresh request to the agent.

Those modeling operations correspond to steps a CAD add-in can use. Once the
decad construction meets the checks the agent has asked for, the agent can
carry the steps into the CAD app to make an editable, parametric part. The CAD
app may interpret those steps differently, so the agent checks the part it
builds.

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

Regenerate every image on this page with `cd _gallery && go run .`.

The landing-page clip animates these parts and the wordmark. Render it with
`cd _gallery && go run . clip > assemble.sh && sh assemble.sh`: the program
writes each shot's PNG frames under `_gallery/out/` and prints the two ffmpeg
commands, which the script runs to write `out/decad-landing.mp4` and
`out/decad-landing.gif`.

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
