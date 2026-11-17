# Boss root fillet overflow design

A `Boss` root fillet whose blend runs off its face (boss-design §6). The blend ends where it meets another face of the
body, as Fusion 360 ends it. Status: approved for building in the increments of §7. P2 is built: the Ø14 union is an
A1 brep with `B` on one carrier (general-boolean §3 "A1 as a brep", holed operands). P4 is built: C2 root arcs ending
on window or slot corner rounds. P3 is built as tangent-notch T3. P5 is built for closed rims (§5.4). P6 is built:
C3 tangent root ends (§5.5), so the Ø14 gallery boss builds. C3 is accepted on the expected outcome of §2.4 until gate
G0 runs. P7 is not built.

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
- Fusion's `FilletFeatureInput` exposes no overflow choice: its members are `edgeSetInputs`, `isRollingBallCorner`
  and `targetBaseFeature`. The kernel picks the outcome itself.
- C1 puts the contact line exactly on an edge. C3 makes the cap tangent to `B` along a line. Both are known failure
  cases of blend kernels. C3 is the configuration most at risk.
- Selection follows the port: `fusion.fillet` of the Fusion add-ins (`addins/Planetary gearbox/commands/gearbox/fusion.py`)
  passes `isTangentChain = False` and sets `isRollingBallCorner`. decad's `Fillet` default is also no tangent chain. The
  root arcs alone are blended. A window's top edges, tangent to a root arc at Ø14, are not.

### 2.4 Expected Fusion behaviour and gate G0

**Build to the expected outcome below. Run gate G0 when Fusion is available, and refuse any class it contradicts.**

| Class | Expected outcome | Confidence |
|---|---|---|
| C2 (Ø15) | keep surface: the fillet trimmed by the extended corner rounds | medium: the usual cliff-edge treatment of blend kernels; no Fusion source confirms it |
| C1 (Ø13) | the fillet borders the inner window walls along a 90° convex edge; `A` keeps cusped pieces | medium |
| C3 (Ø14) | keep surface, the cap on each corner round narrowing to the tangent point | low: a cap tangent to `B` along a line is a known failure case; Fusion may refuse the fillet |

The open uncertainty is C3. If Fusion refuses the Ø14 fillet, decad's Ø14 result has no Fusion counterpart, and the
port must change its geometry whatever decad does.

Gate G0 builds the fixture in Fusion at Ø13, Ø14 and Ø15 through the port's own helpers and records the faces each
fillet produces:

- The boss is extruded from 1 mm below the face and joined, as the port does (§6.2).
- The fillet uses `fusion.fillet` on the root edges only.
- Record per diameter: success or failure, the face count, the surface type of every face the fillet adds or changes,
  and the volume.
- A diameter where Fusion keeps the edge or fails is not built by rule CO: its class moves to X13.

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
| C3 | each root arc ends where a cliff cylinder is tangent to `B`, and `B` runs on below `A` past the end on one carrier (§6.1) | Ø14 | the root arc's end is a zero-angle point of `A`; the union must first state `B` on one carrier (§6.1) |
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
| A0 | The configuration's class is not in X13 (§2.4). |
| A1 | The boss is round, upright, undrafted, without ribs, on a planar face (boss-design R2, Q6, G4, C3, D4 unchanged). |
| A2 | `A` and `B` are retained tags. A stated departure, including a shared span's width charge on `B` (§5.4), is carried into the bounds or refused. |
| A3 | The root is one closed circle (C0, C1) or a set of disjoint concave arcs of the circle `r = R` on `A`, each spanning less than `π` (C2, C3). |
| A4 | Every cliff face whose edge with `A` meets `D` is a retained `Plane` or `Cylinder` parallel to the boss axis, convex to `A` along that edge. |
| A5 | Each arc end is C2 or C3, decided as §5.2 states. A C2 end crosses `B` at an angle whose charged sine exceeds `minSine` (retained-edge-blend §4.5). |
| A6 | Each C1 arc is decided coincident as §5.2 states. Away from C1 arcs, every boundary point of `A` inside `D` lies on a cliff edge of A4 that runs from a C2 or C3 end out of `D`. |
| A7 | No face of `S` other than `A`, `B`, the cliff faces and a C3 end's side face `K` (proved G1 to `B` and `G`, meeting `P` only along the generator through the arc end, §5.5.1) meets `P`. No cliff face holds another edge inside `P`'s reach (retained-edge-blend SR5's end-face rule). |
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
| X13 | a class gate G0 found Fusion keeping the edge or failing; empty until G0 runs | `ErrUnsupported` |

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
| C3's tangency at the arc end | the joint between the window loop's inner arc and its corner round, proved exactly tangent by `capband.JoinIsG1` or admitted near-tangent by `capband.NearTangentCorner` (draft-design §9.1.1, loop-fillet §5.5) with its turn `θ_up` charged into the end (§5.5.3), plus `JoinIsG1` from the full-height wall `K` to `B` at the same recorded point (§5.5.2); a joint neither admits refuses (X4) | an angle residual below a tolerance; a new `sketch` query (no API reports tangent contact apart from a crossing; `prismcells.RegionContact` returns a bool) |
| C2's transversal crossing | the charged sine of the crossing angle exceeds `minSine` | a sine reading without its charge |

A mesh Boolean result keeps retained tags, not the records `sketch` reads. So near-equal C1 and C3 need the records the
union came from: a record route (§6.1) or a history read like `Body.Section`'s (section-design). With tags alone, only
the exact-equality row decides, and a near-equal pair refuses (X4).

### 5.3 The fillet's ends

- C2: the torus rows end on `G`. For a plane `G` the row end solves `n_G · x = h_G` in the azimuth (R4). For a cylinder
  `G` parallel to the axis it solves a circle-circle intersection in plan, a quadratic. Each row is a circle of the torus
  at a tube angle the blend picks, not a recorded entity, so the row ends are not a 2D answer about records. R4 computes
  its plane row ends decad-side for the same reason; retained-edge-blend §4.4 ("An end on a cylinder") states it for
  the cylinder ends. The root on the arc's side is chosen by the side rule: the arc runs on material outside the corner round, so in
  the arc's direction it leaves the round's disk at the centre's azimuth plus `α` (end 0) and enters it at the centre's
  azimuth minus `α` (end 1), where `cos α = (R² + d² − r_G²)/(2·R·d)`. The distance to the held vertex never chooses.
  A discriminant whose charged sign is undecided refuses (X4). A plane end keeps the root nearest the held vertex, and
  the other root must lie farther from it than the vertex's bound, the slack and the roots' rounding, else the end
  refuses ("an arc end lies between two meetings of its tags").
- C3: only the row on `B` has a double root. In the fixture `cos φ = (77 + r²)/(18r)` at plan radius `r`, so the
  azimuth `φ` of the end grows linearly in `ρ − z` near the end point on `B` at height `ρ`. The torus section is a
  regular curve, and the row ends lie along it. The cap on `G` is the curved triangle bounded by `A`'s plane, `G`'s
  generator where it touches `B`, and the torus section. §5.5 states the construction.

### 5.4 Charges

A shared span's width charge (general-boolean A3) is a stated departure of `B`. The stacked view states it, with the
record's section displacement, as the `AxialDelta` of every swept circular wall's `Cylinder` tag (brep-modify §2.3).
The retained edge blend charges it as it charges a displaced plane (retained-edge-blend §6.1, "Displaced planes and
walls"): the closed rim (R2) adds `4δ/minSine` to its slack and a radius term to its area bound, and its blend face
stays `Faceted`. Every other route refuses a charged wall, an open arc (R4) included, and the result publishes no
charged wall. In the fixture two of four inner arcs carry such a charge, so at Ø13 and Ø14 the fillet face is
`Faceted` unless the torus tag gains a field for a positional bound.

At Ø14 each root arc ends at a four-face corner (`A`, the boss wall above the arm, the full wall piece that continues
the window wall, the corner round). §5.5 reads that corner from the record's tangency proof and charges the arc's span
and ends (§5.5.6). A closed rim of the same union, such as a round hole's rim, takes R2's charge
(`apitest/stacked_brep_view_charged_wall_test.go`).

A blend result publishes no charged wall (retained-edge-blend §6.1), so every swept circular wall of the Ø14 union,
the far ones included, leaves the fillet untagged with `FacetedCurve` edges. A later `Chamfer` or `Fillet` that
selects such an edge refuses; the gallery's balance wheel edge chamfers on the rim are such a selection.

### 5.5 The C3 construction (P6)

One construction serves the convex boss wall and the concave notch wall (tangent-notch-design T4). It is R4's cylinder
end (retained-edge-blend §4.4, "An end on a cylinder") with three changes: the corner is four faces, the crossing at
the arc end is a proven tangency instead of a charged sine, and the wall ends at its own seam instead of running on.
Only a C3 end takes it. A C2 end keeps R4's reading, and a C2 end on a charged wall keeps refusing. An arc with one
C3 end and one C2 end refuses ("an arc with one tangent and one crossing cylinder end is not built"): a C2 end needs
the band, a C3 end the whole-face cut.

#### 5.5.1 The four-face corner

| Face | Boss (Ø14) | Notch (fork) |
|---|---|---|
| `A` | arm top, a record planar face | floor, a record planar face |
| `B` | boss wall above the arm, `wall(k)` over `[A, top]` | notch wall above the floor |
| `G` | window corner round, below `A`, `Reversed` | gap round, below the floor, `Reversed` |
| `K` | the window's inner wall over the full height, on `B`'s carrier, tangent to `B` and `G` | the fork's corner round over the full height, tangent to `B` and `G` |

- The arc end `V` is a vertex of exactly these four faces. `B` and `K` meet along the generator through `V` above
  `A`; `G` and `K` meet along it below `A`. `G` meets `A` along a convex edge leaving `V`.
- `readArc` reads such an end only when the caller hands it a proof for the corner (§5.5.2). Without one, the
  four-face corner keeps `cornerOf`'s refusal. So a mesh result, which has no records, never builds C3 (§5.2, last
  paragraph).
- `G` is the face of the four that shares an edge with `A` at `V`; `K` the one that shares no edge with `A` there.
  Reject-only cross-checks: `G` and `K` share a held edge from `V` below `A`, and `B` and `K` one above it.
- `K` must be one record wall across `A`'s level (`K.Z0 < A < K.Z1`). §6.1's per-slab option (a face per slab on one
  carrier) would put five faces at `V` and refuses there; P6 needs P2's one-face option, which P2 built.
- `K` is not rewritten. Its facets that hold the seam edge crossing height `t_B` are split at the wall's cut vertex
  there (the existing `work.seam` split), so the cap meets `K` along the generator from `A` to `t_B` without a
  T-junction. `K` meets `P` only along that generator; A7 admits it (§4.1), and `checkStripClear`'s exact test of
  `K`'s facets against the strip and the cap still refuses any other contact.
- The notch's root arc is the whole semicircle, span exactly `π`. R4's height-cut path admits it; only band mode
  needs a span below `π − minSine`. A3 and X11 are `Boss`'s own gates (`bossRootEdges`).

#### 5.5.2 The tangency proof

`brepmodify` proves the corner from the stacked or slab-cut record before it runs the view blend
(`stackedViewBlend`), and hands the proofs to the blend as a `retainedblend.TangentEnds`. A proof is keyed by the
origin-role triple of `(B, G, K)` under the record's producer (`wall(k)`), never by position, so it still matches the
faces each arc's rewrite rebuilds.

| Check | Proof | Refuses |
|---|---|---|
| `K` and `G` meet at one recorded point `V_r` | the walls' recorded segments share an endpoint, equal floats; one walk ends there and the other starts there | a pair that shares no endpoint, or both |
| `K → G` (or `G → K`) is G1 | `capband.JoinIsG1` on the two record walls; else `capband.NearTangentCorner(prev, cur, 0, 0, 0)`, which admits `sin²θ ≤ 2⁻⁶⁴` | a joint neither admits ("the arc end's cliff joint is neither exactly nor near tangent") |
| `K → B` (or `B → K`) is G1 at the same `V_r` | `capband.JoinIsG1` only: `B` and `K` are one carrier (boss) or a recorded tangent joint (notch) | any other joint |
| the levels | `G.Z1 = B.Z0 = A`'s level and `K.Z0 < A < K.Z1`, exact floats in one record frame (all three walls' frames equal) | anything else |

- The floor loop's own joint `B → G` is a cusp (the tangents are opposite), and both capband tests refuse it. The
  proof never reads it (tangent-notch-design §5).
- Two G1 joints at `V_r` with the walk sense agreeing put `B`, `G` and `K` on one tangent line at `V_r` with one
  outward normal. So `G` is externally tangent to `B` when exactly one of `B`, `G` is `Reversed` (`d' = R + r_G`), and
  internally tangent otherwise (`d' = |R − r_G|`).
- The proof carries `V` (`V_r` placed by the record's `Xform` and frame), used only as a reject-only cross-check, and
  `θ_up`, an upper bound on the `K → G` turn: zero for `JoinIsG1`, else `asin` of `√(sin²θ)` rounded up.
  `capband.JoinTurnSinSq` returns that `sin²θ` from the same rationals `NearTangentCorner` reads.

#### 5.5.3 The ideal end

The construction uses the ideal configuration: `B`'s tag (axis `O`, radius `R`) and `G*`, the circle of `G`'s tag
radius `r_G` whose centre is `c* = O + d'·û`, `û` the unit direction from `O` to `G`'s tag centre `c` in the rim's
plane. `G*` touches `B` at the tangent point `T = O + R·û`. The end's `cylEnd` stores `c := c*` and `d := d'`, so every
R4 reader of `cylEnd.c` and `rG` (`chainWalker`, `alongG`, `outwardG`, `gArc`, `capFacets`, `cylEndChecks`) reads `G*`.

- `e_c = δ_B + δ_G + (r_G + δ_G)·θ_up` is the charge part, `δ` each tag's `AxialDelta` (zero when it states none).
  `e_r = |d − d'|↑ + e_d` is the rounding part, `e_d = 16·ulp·S_far`. `e = e_c + e_r`.
- `e` bounds how far `G*` and `B`'s tag lie from the true walls: each tag lies within its `δ` of its wall, `c*` lies
  `|d − d'|` from `c`, and the true `G` turns at most `θ_up` from the circle tangent to `K` at `V_r`, which moves its
  centre by at most `(r_G + δ_G)·θ_up`.
- The tag check refuses, reject-only: `|d − d'| > e_c + e_d` ("an arc end's tags disagree with its proven tangency").
- The held vertex at `V` must lie within its bound, the slack and `e·(1 + 2R/d') + e_d` of `T`, else SR6. The placed
  `V_r` must lie within the same of `T`, else SR6.
- Every R4 test that reads `G`'s circle with the slack (the chain walk, `readCylChain`'s uncertainty test, the reach
  test, `cylSetback`'s clearance and its `4·margin` vertex test) adds
  `e` to its allowance. On the C3 path `e_V`'s readers are the band (not used) and the held-vertex check (replaced
  above), so `e_V` is not computed.
- `lineEnd.exact` of a C3 end is `T`. Every reader of it (`onGen`, `alongG`, `theta0`/`span`, `arcKey`/`findArc`,
  `gArc`, `capArea`, the plane's zone, the reach test) reads `T`; the ones that compare a held point with it by the
  slack (`onGen`, the `findArc` key) add `e·(1 + 2R/d') + e_d`.
- "Within `e`" means a circle's centre offset plus radius difference is at most `e`, `wall_charge.go`'s convention.
- An internal end requires `r_G < R`, so `T = O + R·û` is the touching point; otherwise it refuses.
- Row ends: the row of radius `ρ` at `s < 1` meets `G*` at `θ_c ± α(ρ)`, `cos α = (ρ² + d'² − r_G²)/(2ρd')`, on R4's side
  rule (end 0 at `+α`, end 1 at `−α`).
- The wall's row is identified by `s = 1`, never by `ρ`: it takes `ρ = R`, `α = 0` and `ψ = 0` (external) or `ψ = π`
  (internal) without calling `acos`, in `rowAngles`, `endSection.at`, `curv`, `shift`, `qHi`, `capArea` and `arcAreas`.
  The profile's float `ρ(1)` is not used there.
- `G` touches `B` at one point of the rim's circle, so there is no second root; the `Π` band and its on-set test
  against the second root's generator do not apply.
- Every admitted true row meets the true `G`: the record puts `V_r` on both walls, so `d_t − r_t ≤ R_t ≤ d_t + r_t`, and
  the rows lie on the side of `B` where `G` reaches. A near tangent (`θ > 0`) makes the true walls cross at `V_r`; the
  side rule then picks the true root at `V_r`'s side, which §5.5.6's `σ` covers.

#### 5.5.4 The end curve's rows

`cylEndSection` keeps R4's certification for every piece of the profile that ends before the wall's row, with these
changes for a tangent end.

- `H(h) = 2·asin(min(1, √(h/2)))`, rounded up, is the modulus of continuity of `acos`: `|acos x̃ − acos x| ≤ H(|x̃ − x|)`
  for `x, x̃ ∈ [−1, 1]`, with equality at `x = 1`.
- `q = sin∠OPC`: under R4's fold check `ρ_lo² + d'² > r_G²`, which stays, `cos∠OPC` grows with `ρ`, so `q` has a
  single peak over the profile and its least value over a piece lies at one of the piece's ends. A piece `[s0, s1]` with `s1 < 1` reads its angle allowance and margin from
  `q_lo = min(q(s0), q(s1))` rounded down, not once from the profile's ends, where `q(1) = 0`.
- A piece with `s1 = 1` takes the cone `[0, π/2]` and no angle allowance. `u = −κ·r_G·ψ(ρ)` and `v = side·z` are both
  monotone in `s` (`ψ` is monotone in `ρ` on `[|d' − r_G|, d' + r_G]`, `ρ` and `z` are monotone in `s`, and `−κ`
  orients `u`), so the curve between the piece's ends lies in their box, the parallelogram `certify` reads for
  `lo = 0`, `hi = π/2`. The piece is split until the box corners lie within `τ` less the margin.
- The margin of a tangent end's piece, with R4's `S = S_far`, `r_lo`, `z_hi` and `ε` (G's tilt):
  `m = 64·ulp·S·(S/r_lo) + 2ε·(z_hi + r_G) + min(X/q_lo, r_G·H(err_ψ) + ρ_hi·H(err_α))`, with
  `X = 64·ulp·S·(S/r_lo) + 2ε·(z_hi + r_G)` and `q_lo` as above. The first branch reads `(1 + 2⁻¹⁰)·X/q_lo` and is
  taken only when `X ≤ 2⁻¹²·q_lo²·min(ρ_lo, r_G)`, as `σ₁` is (§5.5.6), which keeps the crossing's curvature remainder
  inside the factor; otherwise, and on the last piece, the piece takes the second branch.
  With `D = 2ε·(z_hi + r_G)` (G's tilt as a move of its section) and `e_d` both read as moves of `d'` and `r_G`:
  `err_ψ = 16·ulp·(d'² + r_G² + ρ_hi²)/(2d'·r_G) + (1 + 2⁻¹⁰)·(D + e_d)·2·(1/r_G + 1/d')` bounds the error of
  `cos ψ`'s argument (`|∂/∂d'| + |∂/∂r_G| ≤ 2(1/r_G + 1/d')`), and
  `err_α = 16·ulp·(ρ_hi² + d'² + r_G²)/(2ρ_lo·d') + (1 + 2⁻¹⁰)·(D + e_d)·2r_G/(ρ_lo·d')` that of `cos α`'s. The `α`
  term of `m` adds `ρ_hi·asin↑(D/(d' − D))` for the turn of `θ_c` under a sideways shift of `G`'s section, inside the
  second branch. The end
  refuses unless `D + e_d ≤ 2⁻²⁰·min(d', r_G)`, which keeps the second-order remainder inside the `2⁻¹⁰` factor.
- `curv`'s widening `2·e.angle` on the last piece is `2·H(err_ψ)`.
- `sectionGrid`'s up-front test (`margin < τ`) and `certify`'s gap test (`margin + curv < τ`) read the piece's own
  margin; the up-front test reads the first-branch margin of the profile's first piece.
- The strict range checks `ρ > |d' − r_G|` and `ρ < d' + r_G` apply to `s < 1` only.
- A tangent end has no cusp: `u` grows linearly in `ρ_f − z` (`ρ_f` the fillet radius) near the apex (§5.3), so the
  cap is a triangle with a nonzero angle at `(T, t_B)`. `cuspParam` and `widenRows` do not run. `capSimple2` runs on the cap, as it does where
  the wall extends; a cap it refuses refuses.

#### 5.5.5 The wall, the plane and the cap

- `rewriteArc`'s refusal "a root arc ends on a cylinder its wall does not run on past" is lifted for a proven C3 end
  only. A proven C3 end does not run `readCylEnd`; its reading keeps `readCylEnd`'s gates that apply to a tangent end
  (a `Cylinder` with `RetainedTag`, `TagDeparture` 0, `Reversed`, tilt within `parallelTol`, `d > 64·slack`) with the
  displacement clause replaced by `wallRecord` (a usable `faceDelta` and `recordTag`), as `readRimSide` requires of
  `B`. §5.5.3 replaces the crossing sine, `rootBound`, `cylRoots` and the `e_V` vertex check. A proven corner whose
  `G` is not a cylinder refuses.
- `B` lies on one side of `A` and its chain at each end climbs the seam with `K` (`kindB = trims`). It takes R4's
  whole-face height cut (`clipArcWall`), not the band. The cut vertex on the seam edge that crosses `t_B` is entered in
  `work.seam` for that held edge, so `K`'s facet across it is split.
- `A` is R4's cylinder end on `G*`: the chain along `A ∩ G` from `V` (`chainWalker`), `P_A` on `G*` (`cylSetback`),
  the region lost along `G*`'s arc (`gArc`).
- `G`'s checks (`cylEndChecks`) read `G`'s loop at `V` between `A`'s chain and the `G`–`K` seam below `A`, which
  takes the place of `B`'s chain; the convex turn and the reach test are otherwise unchanged.
- The cap (`capFacets`) is R4's with the wall's run replaced by the `B`–`K` seam vertices strictly between `V` and the
  cut vertex, from the cut down to `V`. These are held vertices shared with `K`, so the cap runs those edges against
  `K`'s facets. The cut vertex is the wall's row end.

#### 5.5.6 Bounds and the arc charge

- The torus is published as its tag only when `e_c = 0` (no charge on `B` or `G` and an exact `K → G` joint; the
  notch, whose A9 record states no section displacement), together with §2.5's condition on `A`; otherwise it stays
  `Faceted`, as R2's charged rim does.
- Every strip and cut vertex adds `cylCharge(δ_B)` through the rim's slack (§5.4, R2's rule).
- Each row-end vertex and each cap vertex on `G*` adds `σ = min(σ₁, σ₂) + e`:
  - `σ₂ = ρ_max·(H(a_e) + asin↑(2e/(d' − 2e)))`. `H(a_e)` bounds the move of `α`; the `asin` term the turn of `θ_c`
    when `O` and `c*` each move by `e`.
  - `σ₁ = 2e·(1 + 2⁻¹⁰)/q_row`, the first-order move of a crossing at sine `q_row` when both circles move by at most
    `e`, used only when `e ≤ 2⁻¹²·q_row²·min(ρ_min, r_G)`, which keeps the curvature remainder inside the `2⁻¹⁰`
    factor. `q_row` is a rounded-down lower bound of `cylCurve.q` at the row's `ρ`; the wall's row has `q_row = 0` and
    takes `σ₂`.
- `a_e` bounds how far `e` moves `arg = (ρ² + d'² − r_G²)/(2ρd')`: `ρ` moves by `e`, `d'` by `2e` and `r_G` by `e`.
  With `S² = ρ_max² + d'² + r_G²`, `a_e = (1 + 2⁻¹⁰)·e·(S²/(2ρ_min²·d') + S²/(ρ_min·d'²) + r_G/(ρ_min·d'))`, the
  first-order bound; the end refuses unless `e ≤ 2⁻²⁰·min(ρ_min, d', r_G)`, which keeps the second-order remainder
  inside the `2⁻¹⁰` factor.
- `arcAreaCharge` replaces `rimAreaCharge` for an arc (`readRimSide` sets `wallDelta` on a charged arc, and
  `measureChange` adds the arc's charge instead). It adds to the area slack, per rewritten piece (the region `A`
  loses, `B`'s band, the torus strip, each cap): `2P·σ_max + (π + Φ)·σ_max² + A·e/r_min`. `P` is an upper bound on
  the piece's perimeter from its exact curves (an arc's `ρ·Δθ`; a monotone end curve's `|Δu| + |Δv|`; a generator's
  height), `Φ` the total turning of its boundary in its own unrolled coordinates (bounded by `2π` plus each arc's
  `Δθ`), `A` its exact area and `r_min` its smallest radius of curvature (`R` for `B`, `r_G` for a cap, the smallest
  row radius for the torus, no term for `A`). The `2P·σ` and `Φ` terms hold for a piece that is not convex, as the
  region `A` loses and the cap are not.
- The volume bound multiplies the largest strip, row-end and cap vertex bound by the blend's exact area plus that
  term, as R2's does.
- `refuseChargedArc` stays before `readArc` and lets an arc pass only when both its ends are four-face corners whose
  `(B, G, K)` triple `TangentEnds` proves. A plane-ended arc or a C2 end on a charged wall keeps refusing ("an arc on a
  wall whose tag states a displacement is not built"), and `Fillet` without proofs refuses every charged arc.

#### 5.5.7 The tangent junction

A selected arc meets the unselected edge `A ∩ G` at `V` tangentially. That continuation is convex while the arc is
concave, so it is no tangent chain. `planChains` and `refuseChargedJunctions` skip a junction at `V` when the faces at
`V` are `{A, B, G, K}` and `TangentEnds` proves `(B, G, K)`; the arc end then takes §5.5.3. Every other junction keeps
SM2's refusal, so a mesh result's tangent junctions refuse as before. The brep receiver's SB7 note is refusal text
only and is unchanged.

#### 5.5.8 The root arcs of a brep union

`bossRootEdges` takes an arc whose `Edge.Wedge` reads reflex, as well as one `IsConvex` reports concave. `IsConvex` is
the walked convention, and by its own contract a Union's boss root walks the boss wall's sense and reads convex;
`Wedge` reads the material angle, which is what a root is. A mesh result's arcs keep the `IsConvex` reading, where
`Wedge` can be undecided. `edgesQuery` is unchanged: it selects by each edge's own `IsConvex`.

#### 5.5.9 Tests

- The spoked disc at Ø14 with `RootFillet` 0.5, and `leverForkSketch`'s fork notch with its floor arc filleted, each
  build. Each volume lies within the result's own bound of `vol(S) + vol(reg(W ∩ P))`, integrated in the test from
  the fixture's own radii (§2.2), and points just inside and outside a cap read on the right side.
- Internal tests: the record proof (exact, near-tangent, kinked and cusped joints); the tangent end's rows measured
  against 200-bit roots, each within its margin; the apex box; refusals for a four-face corner with no proof and for a
  joint neither capband test admits.
- Existing tests that change: `TestStackedBrepViewFilletReachesRetainedBlend` (both subtests build) and
  `TestBossRootFilletSpokedHub`'s "14 is refused" (it builds). Every other test is unchanged.

#### 5.5.10 Files

| File | Holds |
|---|---|
| `internal/retainedblend/arc_end_tangent.go` | `TangentEnds`, `TangentProof`, the four-face corner reading, the ideal end, `σ` |
| `internal/retainedblend/arc_end_tangent_section.go` | the tangent end curve's per-piece margins and the apex box |
| `internal/retainedblend/arc_rewrite_tangent.go` | the seam split, `G`'s loop at `V` and the cap's wall run along `K` |
| `internal/retainedblend/arc_charge.go` | `arcAreaCharge` and the charged-arc gate for proven ends |
| `internal/brepmodify/stacked_view_tangent.go` | the record proof of §5.5.2 |
| `internal/capband/join_turn.go` | `JoinTurnSinSq` |
| `boss_fillet.go` | §5.5.8 |

`retainedblend.FilletTangent` takes the proofs; `Fillet` is `FilletTangent` with none. `route.go`, `arc.go`,
`arc_rewrite.go`, `arc_end_cyl.go`, `arc_section.go`, `chain.go`, `wall_charge.go` and `brepmodify/fillet.go` change
only to call into the new files.

## 6. Prerequisites

### 6.1 The Ø14 union states `B` as one carrier

The union at Ø14 puts the boss wall on the inner window walls over the footing's depth (boss-design B3). The mesh path
tessellates the two walls with different chords, so the result carries `FacetedCurve` slivers along the arcs and no
root edge exists to fillet. Rule CO needs `B` below and above `A` on one carrier, recorded from the shared arcs: either
one face, or a face per slab on the same cylinder with a smooth seam at the slab level.

Two routes can give that:

- A record route: A1's brep build extended to a holed lower operand whose holes share arcs with the upper operand's
  outline (A1 with A3's shared spans and A5's holes), returning a `brepPayload`.
- The mesh path with a shared-span reading from the operands' records, so coincident walls take one tessellation, and
  the history read of §5.2.

### 6.2 The footing

The boss pillar starts `embed` below `A` (boss-design B3) everywhere in its section. Where the section leaves `F`, the
footing hangs into the void below `A`: at Ø15 the result holds four 1 mm footings inside the windows, 12.7 mm³ more than
disc plus pillar. The Fusion port copies that footing: the planetary gearbox cover's boss is extruded from 1 mm below
the face and joined (`addins/Planetary gearbox/commands/gearbox/parts.py`, the cover boss). So `S` keeps the footing,
and C2's topology has `G` crossing `B` below `A` as well. Rule CO is unaffected, because `P` lies above `A`.

### 6.3 Route decision: the record route

**Build §6.1 on the record route with a narrow gate: A1's brep build admits a holed operand only where one of its holes
shares a span with the other operand's outline. Every other holed pair keeps today's mesh path silently.**

| Concern | Record route | Mesh route |
|---|---|---|
| shared walls | `sketch` resolves the shared spans in the slab scene (A3), with its coincident-carrier band and the span's width charge | the mesh Boolean has no records; a new reader must find the operands' records through the history |
| coincidence and tangency proofs (§5.2) | the result's records hold every arc and joint, so `capband.JoinIsG1` and `NearTangentCorner` read them directly | the same proofs need the history read first, unestimated |
| the fillet | brepmodify restates a brep as its faceted view and runs the retained edge blend there (`retainedViewBlend`, as `shellViewBlend` and `exactCarrierViewBlend` do); one fillet construction stays in `internal/retainedblend` | the retained edge blend runs on the mesh result directly |
| bodies built today | only holed pairs with a shared span change route; today those build slivered mesh results (Ø14) or none | none outside the new reader |
| code touched | `internal/prismboolean/stacked_union.go` (G6), `internal/stackedunion/brep.go`, `internal/stackedbrep`, an entry in `internal/brepmodify/fillet.go` | `internal/meshbool` and the history readers |

- The record route gives exact proofs from records that already exist. The mesh route needs both a coincident-wall
  tessellation and a history read, and neither has a design.
- With the narrow gate, Ø12, Ø13 and Ø15 keep their mesh results, and P4 builds C2 on the mesh result directly.
- P2 lists every existing test whose union exits at G6 today and holds a shared span (coverage of `TryStackedUnion`
  names them). Each changes route on purpose and states its new faces and bounds. Every other test is unchanged.
- general-boolean §3 "A1 as a brep" and §4.1 ("hole-free regions per slab") change in P2's PR. A holed `brepStack` then
  reaches `stack_brep_scale.go` and later A1 unions, and P2 gates or tests each.

### 6.4 Probe P2a: what the A1 brep build refuses at Ø14

Probe on this branch, 2026-11-16, with G6 lifted for holed operands only and the brep build's misses logged. The edits
were reverted. In order of the build:

| # | Refusal | Where | P2's work |
|---|---|---|---|
| 1 | the slab scene charges at least 4,101 arranger segments against the cap of 4,096 (each circle or arc 256; the count stops at the cap) | `prismcells.MaxArrangementSegments`, `stackedunion/brep.go` scene | the disc's 25 arcs, the pillar circle and 8 lines need about 6,700, at about 3.7 s per scene by the cap's own rate; P2 raises the cap for the gated scene only, never globally, or thins the arranger's densification in `sketch`, which needs a `sketch` release and a version bump here |
| 2 | `CellsHaveNoVoid` reads each window as an enclosed void | `stackedunion/brep.go` slab merge | tell an operand's own hole from a new void, and assign holes to outer loops (`slabRegions` refuses any clockwise loop) |
| 3 | the Ø14 scene's crossing charge is `1.78e-15` (the span between radius 7 and `7.0000000000000009`); the build requires zero | `stackedunion/brep.go`, the `b.crossing != 0` miss | carry the charge as a band on the swept face, §5.4's charge moved into the brep record |
| 4 | not reached: the pillar's wall above `A` and the window arcs below it are different pieces, which `stackedbrep.sweptFaces` does not join | `internal/stackedbrep` | join them on one carrier, or state `B` as a face per slab (§6.1) |
| 5 | not reached: the corner rounds touch the pillar circle tangentially at the arc ends | the interface scene at `z = 5` | when the scene cuts a corner round anywhere other than its recorded vertex, the build misses (reject only); P2 gets `sketch` to cut at the recorded vertex or accepts the miss; `JoinIsG1` proves tangency inside the window record only. This is P2's main open risk |

Ø13 and Ø15 stop at refusal 2 with a zero crossing charge.

## 7. Increments

Each increment is one PR, merged after green CI and an independent review of its soundness-sensitive diff. Every
increment leaves `main` green and every result outside its gate unchanged.

| # | Increment | Unlocks | Estimate |
|---|---|---|---|
| P1 | this design, with §6.3's route decision and §6.4's probe | the plan | done with this PR |
| P2 | §6.4's refusals 1 to 5 under §6.3's narrow gate | the Ø14 union with `B` on one carrier | 4 to 7 weeks |
| P3 | built as tangent-notch-design T3 (brep-modify §2.3): the faceted-view fillet of a stacked or slab-cut BRep after the receiver routes refuse. A holed union with a carrier charge keeps its curved faces untagged until P5 | `Boss` root fillets on P2's results, after P5 | built |
| P4 (built) | C2: R4 with cylinder end faces, `B` running on past a cylinder end face below `A` (the footing), several arcs on one wall, caps on cylinders | Ø15 and transversal root ends on window or slot corner rounds | 2 to 3 weeks |
| P5 (built) | §5.4's cylinder charge on `B` in the retained edge blend, for the closed rim | near-equal carriers at Ø14 | about 1 week |
| P6 (built) | C3 tangent ends with §5.2's tangency proof, the four-face arc end of §5.5, and the charge on an arc's span and ends | Ø14, the gallery case; the lever's dart root (tangent-notch T4) | 1.5 to 2.5 weeks |
| P7 | C1 with §5.2's coincidence proof | exact-equal C1 only; the fixture's Ø13 (near-equal, on the mesh path under the narrow gate) refuses until a history read exists | 1 to 2 weeks |

The Ø14 gallery case needed P2 to P6. Gate G0 runs whenever Fusion is available. The estimates
scale from the existing R4 code (`internal/retainedblend`: `arc*.go`, `cyl_band.go`, `chain_open.go`, about 3,000 lines
with tests) and from A1's brep build (`internal/stackedunion`, about 3,000 lines). P4 and P2 touch disjoint packages and
may proceed in parallel.

**Coordination.** P2 shares `internal/prismcells` and `internal/prismboolean`, and P3 shares
`internal/brepmodify/fillet.go`, with concurrent work on the prism union and fillet path. Each PR rebases on `main`
before review and keeps its code in new files where it can.

## 8. Options

| Option | Gains | Costs |
|---|---|---|
| O1 (chosen): build P2 to P7 on the record route | Ø14 and Ø15 build as Fusion is expected to (§2.4); Ø13 only where its arcs are exactly equal | 9 to 14.5 weeks for Ø14; P7 adds 1 to 2 for exact-equal C1 only |
| O2: build P4 only; the gallery moves the windows' inner radius in, for example to 6.5, keeping Ø14 | the root ends cross the corner rounds transversally (C2), with no coincident walls, no charge and no tangency; the probe's union there has four clean `Arc3` root arcs | 2 to 3 weeks of decad work; the arms' inner window edge moves in 0.5 mm, and the footing hangs into the windows as in the port |
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
