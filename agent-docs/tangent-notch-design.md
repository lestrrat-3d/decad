# Tangent side notch design

A blind `Cut` whose tool meets the target's outer wall only at recorded vertices, tangentially, and the `Fillet` of
the concave arc it leaves on the notch floor. The gallery reports that Fusion 360 builds both (not checked here; §6
gate G0). decad refuses both. Status: design only, not
approved. Nothing in §6 is built.

Navigation (not authority): the failing case → §1; why the cut refuses → §2; why the fillet refuses → §3; the target
result → §4; soundness → §5; increments and size → §6; options → §7.

## 1. Problem

The decad-gallery swisslever lever (`models/swisslever/lever.go`, `buildLever`) is one extrude, one cut and one fillet.
Coordinates below are the lever sketch's, origin at the pallet arbor.

| Body | Profile | Interval |
|---|---|---|
| target | lever outline: two slot holes (oblique lines), arm sides, three sketch fillets, at the fork end a corner round `K` (radius `r_c` = 0.66616, centre `(±2.33384, k_y)`), a gap round `G` (radius 0.53384, centre `(±1.13384, k_y)`) and the dart (sides `x = ±0.6`, tip arc) | `z` 3.30 to 9.80 |
| tool | notch: semicircle `B` (radius `r_n` = 1.66767, centre `(0, k_y)`, under the centre), two oblique walls from `(±r_n, k_y)` to `(±1, 44)`, top line | `z` 5.80 to 10.80 |
| fillet | the one concave circular edge, `B` on the floor `z` = 5.80, radius 0.6 | |

`k_y` = 38.874650636459869. `a = (r_n, k_y)` is the inner end of `K`, the outer end of `G` and the end of `B`. All
three arcs have their centres on `y = k_y`, so all three are vertical at `a`. `K` touches `B` from outside, `G` from
inside. The mirror vertex `a'` is the same.

The result is a valid solid. Above 5.80 the outline runs along `K` into `B` with a smooth (G1) joint at `a` and `a'`.
The floor at 5.80 is the dart top plus two crescents between `G` and `B`, each narrowing to a zero-width point at `a`.

Probe on `main` at `06df1932f5`, 2026-11-17, through a `go.mod` replace in a scratch copy of the gallery:

| Variant | First refusal |
|---|---|
| exact lever (`TestLever` with its skip removed) | `Cut`: "the operands' held facets come within the chord tolerance without provably interpenetrating deeper than it …" (`meshbool.RefuseUndecidableProximity`) |
| notch radius `r_n + 0.02` | the cut builds on the mesh path; `Fillet`: "a blend circle does not reach an end face" (`internal/retainedblend/arc_section.go`) |
| exact lever as two extrudes (band S with the dart, band F with the notched outline) and `Union` | `Union`: the same proximity refusal |
| both of the above with the slot holes left out | the same refusals |
| notch radius `r_n + 0.02`, fillet skipped | the D bore `Cut` builds on the mesh path; the gallery's edge finish refuses `ErrDegenerate`: "a chamfer of 0.75 mm empties a circular wall of radius 0.666164 mm" (the corner round `K`) |

The last row is a gallery sizing matter, not a decad gap: a 0.75 mm chamfer cannot run along a 0.666 mm round, in
decad or in Fusion. It applies to the exact lever too.

## 2. The cut refusal

Three facts together give it.

### 2.1 No analytic route admits a side notch

- prism-boolean §4.4 sends a `Cut` "whose tool ends inside the target and whose boundary crosses the target's (a side
  notch)" to the mesh path.
- general-boolean §3 A7 (`prismboolean.TryCrossingBlindCut`, the `stackedunion` BRep build) is the nearest route. It
  admits a target with exactly one hole, loops of axis-aligned lines, circles and arcs only, a pocket inside the outer
  wall, and a cut slab of exactly one outer and one hole (`ubBuild.cutSlabRegion`). The lever has two holes, oblique
  lines, and a tool that reaches the outer wall.

### 2.2 The mesh path refuses the contact at `a`

The gate's refused face pairs (probe log): the target's corner round `K` (a `Cylinder`) against the tool's oblique
wall (a `Plane`), at both `a` and `a'`, with slack 1.5e-4 mm at the overlap chord and 1.55e-3 mm at the pair chord.
The wall starts at `a`, where `K` is vertical, and leaves at 7.4° into the void, so the true faces touch along the
generator through `a` and nowhere else. Their facets come within the slack without crossing deeper than it, which is
exactly the case the gate refuses (evaluator §9). The cylinder pairs `B`–`K` and `B`–`G` touch along the same line.

The shifted retry (gap-tolerance §5.2) does not open it:

- a translation must move the tool along `−x` to open `a` and along `+x` to open `a'`;
- growth (§5.2 item 5) runs only for coplanar refusals with zero slack, and a gate contact carries slack;
- growth would also notch both horns by the growth, a shape Fusion does not make and one that breaks the fillet (§3).

### 2.3 `sketch` returns a wrong arrangement at `a`

A record route (§6) asks `sketch` for the cells of the band F scene. With A7's gates lifted in a probe, the scene of
the lever region and the notch returned 3 cells (one outline cell and the two slot interiors) where 5 are needed, and `prismcells.ClassifyRegions` refused: one cell's
outer loop walks the notch arc in both directions and the gap arcs and dart edges against their material side.

A `sketch`-only scene reproduces it at the pinned version (`168cc02138ab`) and at `sketch` main (`1f155e0`): the
fork end (arm sides, `K`, `G`, the dart, the closing line) and the notch polygon, sharing `a` and `a'` exactly.

| Scene | Cells (area, mm²) |
|---|---|
| fork end alone | 1 (58.242678) |
| notch alone | 1 (18.041331) |
| fork end and notch | 1 (68.314811, the union's area; 20 edges, some walked twice), reported `Valid` |
| fork end with `G` drawn as two lines, and notch | 3 (50.273480, 8.223888, 9.817444) |
| fork end and a V-shaped notch | 5, correct |

Expected for the third row: 3 cells, about 50.27, 7.97 and 10.07. The wrong cell is reported `Valid`, so this is a
`sketch` soundness defect, not a refusal. decad caught it here only because `ClassifyRegions` met a membership
conflict. The path at the pinned commit (`geom/arrange.go`):

- The consistency gate exempts an internal tangency (around lines 1559 and 1629).
- The shared-endpoint branch (around 1695-1699) breaks before the vertex is registered for exact port order
  (`exactPortVerts`, around 1713-1719), so `a` is never a certified tangency vertex.
- `prepareExactPorts`' second door skips a same-ray tie even when the chord angles tie bit for bit (around 3843-3849).
  `G` and `B` leave `a` on one tangent ray, so curvature, which would order them, is never asked.
- Both arcs are densified with one angular step per turn (around 1306-1318), so their first chords leave `a` on one
  ray to rounding (−1.583068173097986 and −1.5830681730979772 rad). `G`'s first chord (about 0.0131 mm) is shorter
  than `B`'s (about 0.0409 mm) and ends on it, so the chorded curves overlap.

The fix belongs in `sketch` (T1): register a shared endpoint of two curved fragments on one tangent ray for exact port
order (tangent, then signed curvature); keep their chords from overlapping there; and flag the scene `Degenerate`
wherever the ports cannot be ordered (equal curvature, or a line and an arc on one ray) instead of returning a cell.

## 3. The fillet refusal

- Widened notch: `B`'s end `(r_n + 0.02, k_y)` lies inside the horn, and the oblique wall crosses `K` about 0.097 mm
  above `k_y` (the arcs `B` and `K` themselves do not meet: their circles cross at `y = k_y ± 0.138`, each point on one
  arc only). `B` therefore ends at the oblique wall, as in the exact lever. The arc blend ends each row on the plane at
  each arc end (retained-edge-blend R4). That plane lies 1.6727 mm from `B`'s axis. The floor row's circle has radius
  `r_n + 0.02 − 0.6` = 1.0877 mm and never reaches it, so `arcRim.section` refuses. The shape is boss-root-overflow X1
  (an arc ending on a wall that rises from the floor). It says nothing about the exact case.
- Exact lever: never reached, because the cut refuses first. By construction it is boss-root-overflow's rule CO on an
  inner wall. The fillet ring lies inside `B` (torus major radius `r_n − ρ` = 1.06767). Its floor row crosses `G` at
  27.86° from each end of `B`, where the crescent becomes narrower than `ρ` (a transversal crossing of a cliff
  cylinder, C2's construction on a floor row rather than at an arc end). From there to `a` the ring is trimmed by `G`
  extended upward, and at `a` the cap on `G` narrows to the point where `G` touches `B`. That is configuration C3, the
  class boss-root-overflow rates least likely to be built by Fusion and builds in P6, not yet built.
- C3 here differs from the boss's: `B` does not continue below the floor. Below `a` lies `G`; beside it `K` runs the
  full height, tangent to both `B` and `G` along the generator through `a`. The cap on `G` therefore meets `K` along
  that generator from 5.80 to 6.40.
- The selected edge `B` meets the convex floor edge on `G` at `a`, tangentially. The tangent-junction refusal
  (missing-features "Correct refusals", SB7/SM2) concerns a blendable continuation. A convex continuation of a concave
  edge is not one, and an implementation must check that rule does not fire.

## 4. Target result

- Cut: the regularized `target − tool`. Its band S slab is the target region. Its band F slab is the target region
  less the notch: one outer loop, both slot holes. The floor is the band S region less the band F region (the dart top
  and the two crescents). No wall is split: `B` meets the target loop only at its recorded vertices `a` and `a'`.
- Faces at `a`: `K` over the full height; `G` from 3.30 to 5.80; `B` from 5.80 to 9.80; the floor with a cusp at `a`.
  `K`–`G` and `K`–`B` are G1 edges along the generator through `a`.
- Fillet: rule CO (boss-root-overflow §2.2) with `A` the floor, `B` the notch wall, `D` the annulus
  `r_n − ρ ≤ r ≤ r_n`, and `G` the cliff faces. The torus is inside `B`.

## 5. Soundness

CLAUDE.md governs every admission: a decad-side residual may only falsify.

| Decision | Proof | Never |
|---|---|---|
| the band F cells | `sketch`'s arrangement, after §2.3's fix, with `TExact` on every `Partial` edge and `ClassifyRegions` on recorded edge orientation | a point-in-region test in decad |
| `B` meets the target loop only at `a` and `a'` | `sketch`'s intersection query for every pair of a tool entity and a target entity with a curved member (the query `prismcells.ProveGroupApart` reads, prism-boolean §4.2), every reported point being the shared recorded endpoint (equal floats in both records; probe: `(1.6676711944252536, 38.874650636459869)` for `K`, `G` and `B`). Whole edges alone prove nothing: the arrangement neither cuts nor flags a clean tangency | a distance below a tolerance |
| `B`–`G` tangency at `a` for C3 | not the floor loop's joint `B → G`: that joint is a cusp (B arrives along `+y`, `G` leaves along `−y`), and `capband.JoinIsG1` and `NearTangentCorner` both refuse `dot ≤ 0`. Prove it through `K` at the one exact vertex: `JoinIsG1(K → G)` on the target outline and `JoinIsG1(K → B)` on the result's band F outline; or add a reject-only cusp predicate (cross product zero, dot product negative) to `capband` | an angle residual below a tolerance |
| `K` meets the fillet prism `P` only along the generator through `a`, from 5.80 to 5.80 + ρ | boss-root-overflow A7 refuses any other face meeting `P`; T4 changes that gate for a side face G1 to both `B` and `G` at the arc end, proved by the row above | a clearance read from held facets |
| C2 crossing of the floor row with `G` | its charged sine above `minSine` (retained-edge-blend §4.5) | a sine without its charge |

The cut's result is exact where the loops are exact; arcs make its volume `Approximate` with a proven bound, as A7's
do. The mesh path's gate stays as it is: §6 adds a record route ahead of it, and every pair outside that route's gate
keeps today's refusal.

## 6. Increments

Each increment is one PR, merged after green CI and an independent review of its soundness-sensitive diff.

| # | Increment | Unlocks | Estimate |
|---|---|---|---|
| T1 | `sketch`: exact port order at a shared endpoint of two curved fragments on one tangent ray, no chord overlap there, and `Degenerate` where the ports cannot be ordered (§2.3); `sketch` release and pin bump here | correct cells for tangent arcs sharing an endpoint; no silently wrong `Valid` cell | 1 to 2 weeks: the densification change moves the golden scenes that depend on it |
| T2 | general-boolean A9, the side notch on A7's BRep build: a tool meeting the outer loop only at recorded vertices (tangent or not), any number of untouched target holes, oblique lines in untouched records, a cut slab of one outer and any holes, a floor with cusps, G1 wall junctions at those vertices (boss-root-overflow P2a refusal 5, its main open risk); occupied-volume proof through `Tessellate(VerifyAll)` on the cusped floor | the lever body; side notches whose crossings are recorded vertices | 3 to 6 weeks: the first BRep route across the outer boundary, which A7 excludes; boss P2 is sized at 4 to 7 |
| T3 | a faceted-view fillet entry for a stacked BRep result (boss-root-overflow P3, shared) | fillets on T2's results | 3 to 5 days |
| T4 | C3 tangent ends (boss-root-overflow P6) generalized to an inner wall and to §3's end topology (`G` below, `K` beside), the floor-row cliff crossing on an inner wall, and §5's A7 gate change | the dart-root fillet; the balance-wheel boss at Ø14 once its P2 and P5 exist | 2 to 3 weeks (P6 alone: 1.5 to 2.5) |
| T5 | the lever's later steps on T2's result: the D bore `Cut` (an A7 BRep admits no later analytic Boolean, general-boolean §3 A7, so it takes the mesh path) and the edge-finish chamfers over the `K`–`B` G1 joints | the finished lever | unknown until probed on T2's result |

The lever with its fillet needs T1 to T5: about 7 to 12 weeks, T5 unsized. T4 has no real fixture until T2 and T3
land, so the order is T1, T2, T3, T4. Before sizing T2 finally, re-probe the hole-free two-extrude `Union` (§1) on T1:
it is A1's case, and "A1 as a brep" may absorb part of T2. The estimates scale from A7's build
(`internal/stackedunion` and `internal/stackedbrep`, about 4,500 lines with tests) and boss-root-overflow §7.

Gate G0 of boss-root-overflow §2.4 applies to T4: record in Fusion the faces the lever fillet produces. A Fusion result
that keeps the edge or fails moves T4 to a refusal.

## 7. Options

| Option | Gains | Costs |
|---|---|---|
| O1: build T1 to T5 | the lever as designed, as Fusion builds it | 7 to 12 weeks plus T5; T4's Fusion outcome is unconfirmed until G0 runs |
| O2: build T1, T2 and T5; the gallery leaves the dart root sharp | the lever body builds exactly, in 4 to 8 weeks plus T5 | the lever loses its 0.6 mm root round in decad and in the port |
| O3: build T4 as one generalized C3 increment with boss-root P6 | one C3 construction serves both gallery models; saves about 1 week against two | P6 waits for this design's approval |
| O4: change the fork geometry so no tool face touches the target | no decad work for the cut: `r_n + 0.02` cuts today, and the bore after it | the fillet then refuses (X1, §3); no tested geometry builds both; the user has preferred decad fixes |
| O5: keep `TestLever` skipped | nothing | the lever, and every lever test, stays unbuilt |
