# Boss root fillet overflow design

A `Boss` root fillet whose blend runs off its face (boss-design §6). The blend ends where it meets another face of the
body, as Fusion 360 ends it. Status: design only. Nothing here is built.

Navigation (not authority): target result → §2; the gate before any build → §2.4; which configurations build → §4;
why each admission is sound → §5; what must exist first → §6; size and options → §7, §8.

## 1. Problem

`Boss` with `RootFillet` calls `Fillet` on the one closed circle of the result in the face's plane (boss-design R2).
The retained edge blend builds that circle as R2 (retained-edge-blend §4.2) only when the whole blend lies inside the
plane face (retained-edge-blend §6.2, R2 row). A boss standing where the face ends has no such circle.

Probe on `main` at `cccee25a4b`, 2026-11-16. The fixture is a disc of radius 21 and thickness 5 from `z = 0`, with four
windows between arms 4 wide (half width 2 about radial lines at 45°, 135°, 225°, 315°). Each window's inner wall is an
arc of radius 7 about the axis, its outer wall an arc of radius 18, and every window corner is a sketch fillet of
radius 2. The boss stands on the top face at the origin: height 1, `RootFillet` 0.5. This is the swisslever balance
wheel of decad-gallery at the gallery's hub size.

| Boss Ø | Root `R` | Union result near the hub | `Boss` with `RootFillet` 0.5 |
|---|---|---|---|
| 12 | 6 | whole root circle on the face | builds (R2) |
| 13 | 6.5 | whole root circle; the blend's outer tangent circle `R + ρ = 7` lies on the windows' inner arcs | "the held bounds cannot show that the blend leaves the plane face" (SR5) |
| 14 | 7 | boss wall on the windows' inner walls; 92 faces with `FacetedCurve` slivers along those arcs | "the boss's root circle is not an edge of the result" |
| 15 | 7.5 | four clean concave root arcs, each ending on two window corner rounds | "the boss's root circle is not an edge of the result" |

The union is a mesh Boolean result (`facetedrecord.Payload`) at every size: A1 (general-boolean §3) admits hole-free
operands only, and the disc has four holes.

## 2. Target result: cliff capping

### 2.1 Terms

| Term | Meaning |
|---|---|
| `A` | the boss's planar face, plane `z = 0` in the boss frame, material below |
| `B` | the boss wall, the cylinder of radius `R` about the boss axis, normal to `A` |
| `ρ` | the fillet radius |
| `W` | the full fillet ring: the solid swept about the axis by `{(s, z): 0 ≤ s ≤ ρ, 0 ≤ z ≤ ρ, (s − ρ)² + (z − ρ)² ≥ ρ²}`, `s = r − R` |
| `D` | the ring's footprint on `A`'s plane: the annulus `R ≤ r ≤ R + ρ` |
| `F` | the closed planar region of `A` in the body before the boss |
| cliff face `G` | a face of the body before the boss that meets `A` along a convex edge crossing `D` |
| `S` | the result of the boss's union and pilot cut, before the fillet, footing included (§6.2) |

### 2.2 Rule CO

**The filleted result is `S ∪ reg(W ∩ P)`, where `P` is the prism `{x + h·n : x ∈ F ∩ D, 0 ≤ h ≤ ρ}` over `F ∩ D`, `n`
the face's outward normal, and `reg` the closure of the interior.**

- `reg` drops what `W ∩ P` keeps of lower dimension: the generator segment at a C3 end and the contact line of C1.
- When `D` lies inside `F`, `W ∩ P = W` and the rule is R2's ring (retained-edge-blend §4.2).
- When every cliff face `G` is a surface swept along `n` (a plane or a cylinder parallel to the boss axis), its
  extension above `A` is the side wall of `P`. So `reg(W ∩ P)` is the ring trimmed by every extended cliff face.
- An oblique cliff face (a chamfered or drafted window) extends along its own slope, not along `n`. Rule CO is then not
  the capped result, and such a cliff is refused (§4.2, X3).

### 2.3 Why this may be Fusion's result

A kernel has three outcomes where a rolling-ball blend's contact line on `A` runs over the convex edge `A ∩ G`:

| Outcome | Result | Name elsewhere |
|---|---|---|
| keep surface | the blend surface rolled on `A` and `B`, trimmed by `G` extended: rule CO | cliff-edge capping |
| keep edge | the edge `A ∩ G` stays; past it the ball rolls on `B` and along that edge, a pipe about the edge | keep edge |
| failure | the feature does not compute | "fillet could not be completed" |

- No ball touches both `B` and `G` from the material side, because `A` and `G` meet convexly. So the kernel never
  rolls the ball onto `G` as a face.
- Rule CO is the keep-surface outcome. Nothing in this repository shows which outcome Fusion takes.
- C1 puts the contact line exactly on an edge. C3 makes the cap tangent to `B` along a line. Both are known failure
  cases of blend kernels. C3 is the configuration most at risk.
- Selection follows the port: `fusion.fillet` of the Fusion add-ins (`addins/Planetary gearbox/commands/gearbox/fusion.py`)
  passes `isTangentChain = False` and sets `isRollingBallCorner`. decad's `Fillet` default is also no tangent chain. The
  root arcs alone are blended. A window's top edges, tangent to a root arc at Ø14, are not.

### 2.4 Gate G0 — Fusion decides the rule

**Before any increment of §7 is scheduled, build the fixture in Fusion at Ø13, Ø14 and Ø15 through the port's own
helpers, and record the faces each fillet produces.**

- The boss is extruded from 1 mm below the face and joined, as the port does (§6.2).
- The fillet uses `fusion.fillet` on the root edges only.
- Record per diameter: success or failure, the face count, the surface type of every face the fillet adds or changes,
  and the volume.
- A diameter where Fusion keeps the edge or fails is not built by rule CO. Its increments are dropped or redesigned.

### 2.5 Faces and edges of the result

| Face | Surface | Change |
|---|---|---|
| fillet | `Torus`, `Center` on the axis at height `ρ`, `Major = R + ρ`, `Minor = ρ` (R2's) | trimmed by the cliff faces |
| `A` | its plane | loses `F ∩ D` |
| `B` | its cylinder | loses the band up to height `ρ` over the root arcs |
| each cliff face `G` | its own plane or cylinder | gains a cap: the part of its extension between `A`'s plane and the torus |

| Edge | Curve |
|---|---|
| fillet ∩ `A`, fillet ∩ `B` | arcs of the circles `r = R + ρ, z = 0` and `r = R, z = ρ` (`Arc3`) |
| `G` ∩ `A` | the existing cliff edge, shortened |
| fillet ∩ `G` | a torus section by a plane or by a cylinder parallel to the axis: no curve of the vocabulary, a chain of exact points (retained-edge-blend §4.4 states the plane case) |

The fillet face is published as a `Torus` tag only with no charge on `A` or `B` (retained-edge-blend §6.1, displaced
planes). A charged fillet stays `Faceted` (§5.4).

## 3. Configurations

Each fixture row of §1 is one of C0 to C3. C4 is the remaining case rule CO covers.

| # | Configuration | Fixture | What is new against R2 and R4 |
|---|---|---|---|
| C0 | `D` inside `F` with clearance | Ø12 | nothing (R2) |
| C1 | `D ⊂ F`, and the outer tangent circle `r = R + ρ` lies on `∂F` along arcs: a cliff cylinder coaxial with `B` of radius `R + ρ` | Ø13 | the fillet borders the cliff cylinder directly along those arcs, with a 90° convex edge; `A` keeps pieces that end in zero-angle points where `∂F` leaves the circle |
| C2 | each root arc ends on a cliff face that crosses `B` transversally | Ø15 | R4 ends on planes only, refuses several arcs on one wall face, and has no cap on a cylinder |
| C3 | each root arc ends where a cliff cylinder is tangent to `B`, and `B` runs on below `A` past the end as one face | Ø14 | the root arc's end is a zero-angle point of `A`; the union must first state `B` as one face (§6.1) |
| C4 | the root circle stays whole, and `∂F` crosses `D` transversally: a boss near a plate's edge, or a window whose inner radius lies strictly between `R` and `R + ρ` | none | the cap is an arch from `A`'s plane back to `A`'s plane, and some rows never reach `G` |

C1 and C3 hold only where two recorded curves are coincident or tangent. In the fixture they are so only to rounding:
two of the four inner window arcs record radius 7 and two `7.0000000000000009`, and the corner rounds record radii from
`1.999999999999996` to `2.0000000000000031`. A C1 case whose cliff radius sits a few ulps inside `R + ρ` is a C4 case
with a cap about `1e-15` wide; §5.2 decides which one holds.

## 4. Accepted and refused configurations

### 4.1 Accepted

A `Boss` with `RootFillet` builds by rule CO when every row holds. The first failing row is the refusal.

| # | Condition |
|---|---|
| A0 | Gate G0 recorded the keep-surface outcome for the configuration's class. |
| A1 | The boss is round, upright, undrafted, without ribs, on a planar face (boss-design R2, Q6, G4, C3, D4 unchanged). |
| A2 | `A` and `B` are retained tags. A stated departure, including a shared span's width charge on `B` (§5.4), is carried into the bounds or refused. |
| A3 | The root is one closed circle (C0, C1) or a set of disjoint concave arcs of the circle `r = R` on `A`, each spanning less than `π` (C2, C3). |
| A4 | Every cliff face whose edge with `A` meets `D` is a retained `Plane` or `Cylinder` parallel to the boss axis, convex to `A` along that edge. |
| A5 | Each arc end is C2 or C3, decided as §5.2 states. A C2 end crosses `B` at an angle whose charged sine exceeds `minSine` (retained-edge-blend §4.5). |
| A6 | Each C1 arc is decided coincident as §5.2 states. Away from C1 arcs, every boundary point of `A` inside `D` lies on a cliff edge of A4 that runs from a C2 or C3 end out of `D`. |
| A7 | No face of `S` other than `A`, `B` and the cliff faces meets `P`. No cliff face holds another edge inside `P`'s reach (retained-edge-blend SR5's end-face rule). |
| A8 | The cap on each cliff face is a simple region of that face, and the held audit of the rewritten mesh passes (retained-edge-blend SR7). |

### 4.2 Refused

| # | Configuration | Error |
|---|---|---|
| X1 | a root arc that ends on a reflex edge of `A` (a wall rising from `A`): the ball would roll onto that wall, a vertex blend | `ErrUnsupported` |
| X2 | a cliff face that is a cone, sphere, torus, free-form or faceted surface | `ErrUnsupported`, SR1's text |
| X3 | a cliff face not parallel to the boss axis (oblique plane, oblique cylinder): its extension is not `P`'s wall | `ErrUnsupported` |
| X4 | a C1 or C3 coincidence that §5.2 cannot certify, or a C2 crossing at or below `minSine` | `ErrUnsupported` |
| X5 | a feature of `S` inside `P` other than the boss: a second boss, rib or wall within the ring | `ErrUnsupported` (A7) |
| X6 | `ρ ≥` boss height, `ρ` too large for `A` or `B` (R2's existing gates) | existing errors |
| X7 | a drafted, shaped, tilted, ribbed or cylinder-face boss | existing `ErrUnsupported` |
| X8 | C4, the whole root circle with `∂F` crossing `D` | `ErrUnsupported` |
| X9 | a root arc that ends at a corner of `∂F` (two cliff edges meet at the end) | `ErrUnsupported` |
| X10 | both ends of one root arc on one cliff face (R4's existing refusal) | `ErrUnsupported` |
| X11 | a root arc of `π` or more, as a boss overhanging a plate's edge by less than half its width makes | `ErrUnsupported` |
| X12 | a root arc that ends on a cliff edge lying on the root circle itself (a cliff cylinder coaxial with `B` of radius `R`, outside C3's merged wall) | `ErrUnsupported` |
| X13 | a configuration whose class gate G0 found Fusion keeping the edge or failing | `ErrUnsupported` |

## 5. Soundness

### 5.1 The rule

- `S`, `W` and `P` are regular closed sets, and `reg` keeps the trim regular. So `S ∪ reg(W ∩ P)` is a regular closed
  solid. `reg(W ∩ P)` touches `S` along `A ∩ D` and `B` up to height `ρ`, so the result is one lump wherever `S` is.
- `reg(W ∩ P)` lies in `0 ≤ z ≤ ρ` above `F`, where `S` has only the boss (A7). So the fillet adds material and
  removes none, as a concave fillet must.
- C0 reduces to R2 exactly, so every result built today is unchanged.

### 5.2 Coincidence and tangency

CLAUDE.md's rule governs every admission: a decad-side residual may only falsify.

| Decision | Proof | Never |
|---|---|---|
| C1, or C3's coincidence of the inner arc with `B`, between tags equal in rationals with zero departure | the exact equality itself | — |
| C1 between near-equal carriers | `sketch`'s coincident-carrier reading from the records (`carriersIdentical`, prism-boolean §4.1), through general-boolean A3's shared span, charged by the span's width | a radius residual below a tolerance |
| C3's tangency at the arc end | the joint between the window loop's inner arc and its corner round, proved exactly tangent by `capband.JoinIsG1` or admitted near-tangent with its sliver charged by `capband.NearTangentCorner` (draft-design §9.1.1, loop-fillet §5.5), plus the shared span of §6.1 that makes the inner arc part of `B`; the arc end is the span's end vertex; a joint neither admits refuses (X4) | an angle residual below a tolerance; a new `sketch` query (no API reports tangent contact apart from a crossing; `prismcells.RegionContact` returns a bool) |
| C2's transversal crossing | the charged sine of the crossing angle exceeds `minSine` | a sine reading without its charge |

A mesh Boolean result keeps retained tags, not the records `sketch` reads. So near-equal C1 and C3 need the records the
union came from: a record route (§6.1) or a history read like `Body.Section`'s (section-design). With tags alone, only
the exact-equality row decides, and a near-equal pair refuses (X4).

### 5.3 The fillet's ends

- C2: the torus rows end on `G`. For a plane `G` the row end solves `n_G · x = h_G` in the azimuth (R4). For a cylinder
  `G` parallel to the axis it solves a circle-circle intersection in plan, a quadratic. Each row is a circle of the torus
  at a tube angle the blend picks, not a recorded entity, so the row ends are not a 2D answer about records. R4 computes
  its plane row ends decad-side for the same reason. The increment that builds I2 must state that reason in
  retained-edge-blend, or move the computation into a `sketch` scene. The root on the arc's side is chosen by the sign of
  the exact discriminant and the arc's direction, and a discriminant whose charged sign is undecided refuses (X4).
- C3: only the row on `B` has a double root. In the fixture `cos φ = (77 + r²)/(18r)` at plan radius `r`, so the
  azimuth `φ` of the end grows linearly in `ρ − z` near the end point on `B` at height `ρ`. The torus section is a
  regular curve, and the row ends lie along it. The construction picks each row's root by continuity from the rows
  above `B`, or parameterizes the end curve by `z`. The cap on `G` is the curved triangle bounded by `A`'s plane, `G`'s
  generator where it touches `B`, and the torus section. The torus section meets that generator at height `ρ` only
  where `G` touches `B` exactly. Under a rounding-level miss, §5.2's tangency proof decides, and `capband.NearTangentCorner`'s sliver charge enters the cap's bounds.

### 5.4 Charges

A shared span's width charge (general-boolean A3) is a stated departure of `B`. retained-edge-blend charges displaced
planes through `planeCharge` and refuses a displaced cylinder (SR1). Rule CO on a charged `B` needs a cylinder charge of
the same kind, carried into the strip's bounds. Its fillet face is then not published as a `Torus` tag (retained-edge-blend
§6.1, displaced planes) and stays `Faceted`. In the fixture two of four inner arcs carry such a charge, so at Ø13 and
Ø14 the fillet face is `Faceted` unless the torus tag gains a field for a positional bound.

## 6. Prerequisites

### 6.1 The Ø14 union states `B` as one face

The union at Ø14 puts the boss wall on the inner window walls over the footing's depth (boss-design B3). The mesh path
tessellates the two walls with different chords, so the result carries `FacetedCurve` slivers along the arcs and no
root edge exists to fillet. Rule CO needs `B` as one face below and above `A`, recorded from the shared arcs.

Two routes can give that:

- A record route: A1 extended to a holed lower operand whose holes share arcs with the upper operand's outline (A1 with
  A3's shared spans and A5's holes), returning a `brepPayload`. The fillet then runs on the brep routes, which have no
  root fillet of this kind, so §5.3's construction is built a second time there.
- The mesh path with a shared-span reading from the operands' records, so coincident walls take one tessellation, and
  the history read of §5.2.

### 6.2 The footing

The boss pillar starts `embed` below `A` (boss-design B3) everywhere in its section. Where the section leaves `F`, the
footing hangs into the void below `A`: at Ø15 the result holds four 1 mm footings inside the windows, 12.7 mm³ more than
disc plus pillar. The Fusion port copies that footing: the planetary gearbox cover's boss is extruded from 1 mm below
the face and joined (`addins/Planetary gearbox/commands/gearbox/parts.py`, the cover boss). So `S` keeps the footing,
and C2's topology has `G` crossing `B` below `A` as well. Rule CO is unaffected, because `P` lies above `A`.

## 7. Increments

| # | Increment | Unlocks | Estimate |
|---|---|---|---|
| G0 | Fusion run of §2.4 | decides which increments are worth building | about an hour in Fusion |
| I2 | C2: R4 with cylinder end faces, `B` running on past a cylinder end face below `A` (the footing), several arcs on one wall, caps on cylinders | Ø15 and every transversal root end | 2 to 3 weeks |
| I3 | C1 with §5.2's coincidence proof through the union's records | Ø13 | 1 to 2 weeks, after I4's record source |
| I4 | §6.1 one-face union at Ø14 | the Ø14 union | 2 to 4 weeks, plus 2 to 3 weeks for §5.3 again on the brep routes (record route) or an unestimated history read (mesh route) |
| I5 | C3 tangent ends | Ø14 | 1.5 to 2.5 weeks, after I2 and I4 |
| I6 | §5.4's cylinder charge on `B` | Ø13 and Ø14 with near-equal carriers | about 1 week |

The Ø14 gallery case needs G0, I2, I4, I5 and I6: about 8.5 to 13.5 weeks on I4's record route, or 6.5 to 10.5 weeks plus the
unestimated history read on its mesh route. The estimates scale from the existing R4 code
(`internal/retainedblend`: `arc*.go`, `cyl_band.go`, `chain_open.go`, about 3,000 lines with tests) and from A1's brep
build.

## 8. Options

| Option | Gains | Costs |
|---|---|---|
| O1: build G0, I2 to I6 | Ø14 and the other configurations build as in Fusion, if G0 confirms | 8.5 to 13.5 weeks (record route), or 6.5 to 10.5 weeks plus an unestimated history read (mesh route) |
| O2: build G0 and I2; the gallery moves the windows' inner radius in, for example to 6.5, keeping Ø14 | the root ends cross the corner rounds transversally (C2), with no coincident walls, no charge and no tangency; the probe's union there has four clean `Arc3` root arcs | 2 to 3 weeks of decad work; the arms' inner window edge moves in 0.5 mm, and the footing hangs into the windows as in the port |
| O3: the gallery moves the windows' inner radius out to at least `R + ρ` plus a margin | builds today: inner radius 7.6 with Ø14 and fillet 0.5 builds (C0); 7.5 refuses (C1) | no decad change; the inner window edge moves out 0.6 mm |
| O4: the gallery shrinks the boss to Ø12.5 or less | builds today | rejected by the user |

## 9. Rejected shortcuts

- **A fillet ring tool clipped by `P` and unioned.** `W ∩ P` meets `S` on `A`'s plane, on `B` and on every cliff face,
  faces the mesh Boolean refuses as tangent or coincident contact (retained-edge-blend §1). Moving the tool into `S` to
  avoid that puts the ring's sides on the cliff walls below `A`, the same coincident contact as §6.1.
- **Filleting before the windows are cut.** For vertical cliffs this order gives `S ∪ reg(W ∩ P)`, and Ø12 builds that
  way. At Ø14 the window cuts meet the coincident walls of §6.1 (92 faces after the first pair, the second refuses), at
  Ø13 the third cut refuses on its mesh bound, and one cut of all four windows exceeds the analytic scene's segment cap.
- **Admitting C1 or C3 when the radii or angles agree to a tolerance.** A small residual proves nothing (CLAUDE.md).
- **Reading O3 or O4 as a decad fix.** Both change the gallery's geometry and leave every configuration of §3 other
  than C0 refused.
