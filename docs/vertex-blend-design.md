# Three-edge vertex blends

`Body.Fillet` admits a complete planar-face loop together with independent
straight edges that end on that loop. The straight-edge rewrite runs first;
the loop rewrite then fillets its new arc segments. A circular wall whose
recorded radius equals the requested fillet radius produces a sphere patch at
the shared vertex.

This document narrows the selection refusals in
`docs/brep-modify-design.md` Table SB,
`docs/modify-general-design.md` Table SL, and
`docs/modify-design.md` Table S. It adds LF9 to
`docs/loop-fillet-design.md` Table LF. The ordinary route E, route L, and
loop-fillet gates still apply to each step.

## 1. Geometry and record

For an inward circular walk of radius `R`, the loop fillet's rolling-ball
centre follows a circle of radius `R-r`. At `R=r`, the circle is one point:
the walk's recorded centre `C` at the side level. The resulting patch is a
`Sphere` of radius `r`, from the side arc at the ball's equator to the
single pole at the cap level. Its two other edges are the neighbouring
straight walks' quarter-circle meridians. A right-angle box corner gives
one sphere octant.

The radius comparison first checks the held `float64` readings, then checks
in rational arithmetic that both pinned endpoint-to-centre distances squared
equal `r²`. It does not turn a near-equal arc into a sphere. Making a
sphere from a different radius would leave a gap along the existing side arc.

An LF9 walk must be open, inward, and exactly tangent (G1) to a straight
walk on each side. Both offset joins use the recorded centre `C`; the
collapsed arc contributes no cap-contour segment. The two straight offset
segments meet at `C`, which is the pole's planar coordinate. Chamfer and
ordinary offsets still reject a dropped circular carrier.

| Class | Loop walk or corner | Result |
|---|---|---|
| LF9 | inward open arc, both recorded endpoints exactly at `r`, straight G1 neighbours | sphere patch with one side arc and two meridians |
| LF2 | inward arc, `R > r` with regular cap contour | torus patch |
| LF3 | outward arc | torus patch with major radius `R+r` |
| LF4 | sharp convex corner between two selected loop lines | two cylinders meet along an ellipse |
| LF5 | exactly tangent join | one shared meridian |

The strip measurements use the existing loop-fillet coefficient integral.
For LF9's arc sweep `β`, its coefficients are `a₁=βr` and `a₂=-β/2`.
The receiver already includes the straight-edge fillets before the loop
strip is subtracted or added. The sphere patch has area `βr²`.

## 2. Selection and build

Route V partitions a fillet selection into complete planar-face loops and
independent straight edges along reference axes. For a box-like selection
with several possible complete loops, it first tries each reference axis as
the straight-edge set; the remaining edges must form complete, non-sharing
loops. Otherwise it assigns each edge covered by one complete loop to the
loop set and checks the remaining straight edges for shared vertices.
Two selected straight edges sharing a vertex remain refused.

The straight-edge set runs through route E (`brepBlendEdges`). A prism is
read through `brepOfPrism` first; a single straight prism cap edge also
uses route E. Route E keeps face and loop indices. A new route L reading
uses those indices on the rewritten brep, including arcs route E added to
the loops. Route L records its loop bands and builds one brep result. The
public `Fillet` call commits only that final result.

`filletOffsetJoins` admits LF9 and names the pole. The fillet-specific
offset builder omits its zero-length arc. `attachFilletBand` creates a
three-edge sphere face: the side arc, the trailing meridian in reverse, and
the leading meridian. The two neighbouring cylinder faces share those
meridians. A complete circular walk still takes LF7's torus path.

## 3. Meshing and proof

The band chords each wall walk once. Its side, interior, and cap rings keep
matching sample counts. LF9's cap samples all lie at the pole, so the planar
cap triangulator removes consecutive duplicate samples from its own ring.
The band retains them to form the triangular sphere strips. The mesh still
passes the vertex-link and facet-area audits.

`CapOffsetStationBound` bounds the ideal cap stations for the occupied
volume proof. At an exactly collapsed radius, every station is exactly the
recorded centre; `circularbounds.OffsetEndpointInterval` returns that point
interval directly. The other displacement and volume terms remain the
loop-fillet terms.

## 4. Refusals

| Input | Result |
|---|---|
| a partial selected loop, or loops sharing an edge after the partition | SL1, `ErrUnsupported` |
| selected straight edges sharing a vertex outside the selected loops | SL1 or SB5, `ErrUnsupported` |
| an LF9 candidate with a circular neighbour or a non-G1 join | SF1, `ErrUnsupported` |
| held radius `r` but either recorded endpoint has a different exact distance to the centre | `ErrUnsupported` |
| recorded inward arc radius slightly above `r` but its residual is within `shellTol` | `ErrUnsupported`, naming the unequal radii |
| inward arc radius below `r`, or another offset feature consumed | SX6, `ErrDegenerate` |
| an exact-radius whole circle | existing LF7 path; it does not use LF9 |
| mixed loop and single-edge `Chamfer` | existing SL1 refusal |

A trapezoid with slanted sides demonstrates the near-equal refusal: one
rewritten arc has a held radius of about `1.000000000000006 mm` for a
`1 mm` fillet, while another has an endpoint slightly off its exact
radius-one circle. The record does not define exact spheres there.

## 5. Integration fixtures

| Receiver and selection | Expected result |
|---|---|
| 40×20×20 box, all 12 edges, `r=2` | 8 spheres; volume `14848 + 848π/3` |
| same box, top loop and 4 vertical edges | 4 spheres; volume `15264 + 544π/3` |
| same box, top loop and 2 vertical edges | 2 spheres and 2 ellipse seams |
| same box, one straight top edge | route E brep; volume `15840 + 40π` |
| 40×40×10 plate with 20×10×5 pocket, floor loop and 4 vertical edges, `r=1.5` | 4 spheres; volume `15153 - 297π/8` |
| cross-drilled 40×20×20 bar, all 12 straight edges, `r=2` | 8 spheres; volume `14848 + 308π/3` |
| rounded plate P8, top loop over radius-3 side arcs, `r=3` | 4 spheres; volume `14416 + 207π`; tessellates |

The tests assert closed topology, bounded volume readings, patch kinds, and
the cap mesh for the radius-equality cases. The trapezoid test asserts a
typed refusal and leaves the receiver live.
