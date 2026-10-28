# Exact polyhedral Boolean design

`Union`, `Cut` and `Intersect` of two solids whose faces are all planes with exact vertices, decided with exact
signs instead of a chord tolerance. The predicates and implicit points are in `internal/exactbool`; the arrangement,
classification and routing land in later phases (§8).

## 1. Problem

Two faceted solids combine exactly today only when every face squares to a world axis
(`agent-docs/orthogonal-boolean-design.md` B1). Every other pair takes the float mesh route (`internal/meshbool`),
which decides contact with a chord tolerance. It refuses what it cannot decide, and it returns a result with a volume
bound.

| Refusal today | Example | Source |
|---|---|---|
| Coplanar faces of tilted planes | box ∩ the same box turned 45° about z (S9) | `general-boolean-design.md` §2 |
| Two operands sharing an edge or a vertex only | two boxes touching along one vertical edge (S12c) | same |
| Faces closer than the chord tolerance | rotated 90° box pair; a boss standing on a plate (B3) | same |
| Gap retry fails or changes the result | tools tangent to a wall; flush caps with no room to move | `gap-tolerance-design.md` §5.2 |
| Result carries a mesh bound for a polyhedral input | tilted cut on a box (S10, bound 0.69 mm³) | same |

A polyhedral pair has an exact answer. The refusals come from deciding contact with rounded coordinates, not from
missing mathematics.

## 2. Goal

For two solids whose faces are all planes with exact vertices, `Union`, `Cut` and `Intersect` return the exact result
for any contact, with no `BooleanUnsupportedContact`, no gap shift and no chord-tolerance refusal.

Non-goals for the first release:

- Curved faces (cylinder, sphere, cone, torus, free-form). They stay on the existing routes (§8 phase 4).
- A new public type. The kernel sits behind the existing `Union`, `Cut` and `Intersect`, as the grid route does.
- Replacing the grid route. It stays the fast path for axis-square pairs (B1).

## 3. Design

### 3.1 Representation

A vertex of the result is the meeting of three input planes, stored as the three plane indices (an implicit point),
plus a held float position and a bound on its distance from the exact point.

| Item | Rule |
|---|---|
| Input plane | Exact rational `n·x = h`, read from the operand's retained plane tag or from its exact triangle vertices (`mesh-solid-design.md` M4) |
| Input vertex | Exact float coordinates, the triangle corners, as today |
| Result vertex | Three plane indices, or an input vertex. The exact coordinates are the rational solution of the 3×3 system |
| Held position | The float nearest the rational point, with a bound from the rounding. Computed once, never fed back into a decision |

Decisions never read a held float. Only the final output positions are rounded, and the vertex bound already carries
that rounding (the same `VertexBound` the mesh bodies carry).

### 3.2 Predicates

Three predicates decide everything:

- `orient3d(a, b, c, d)`: the side of plane `abc` on which `d` lies.
- `side(plane, implicit point)`: the sign of `n·x − h` at a plane-triple vertex.
- `order(plane triple, plane triple, axis)`: which of two implicit points on one line comes first.

Each runs in floating point with an error bound first. When the sign is within the bound, it reruns in `math/big`
rationals. The result is always the exact sign, so a coplanar pair, a shared edge and a vertex on a face are decided
as such, not refused. This is the filtered-predicate scheme the proof arithmetic (`internal/proof`) already uses for
volumes.

### 3.3 Arrangement

1. Build a box tree over both operands' triangles (reuse `meshbool`'s tree).
2. For each pair of triangles with overlapping boxes, intersect them exactly: classify the six vertex–plane signs,
   then emit the segment, point or coplanar polygon in implicit coordinates.
3. Split each triangle along its segments. A coplanar overlap is clipped in 2D with the exact `side` predicate
   (the same clipping the contact patch uses, `multibody-dynamics-design.md` §9.4).
4. Merge equal implicit vertices by comparing plane triples, not floats.

Step 3's 2D work is the one place `sketch` cannot answer the question, because the polygons carry implicit
vertices that `sketch` does not take. The design doc for the code states that reason, as the hard rules require.

### 3.4 Classification and output

- Each split triangle is inside, outside, or on the boundary of the other operand. A winding number from a ray cast
  with exact `orient3d` decides inside and outside. A triangle lying in the other's plane takes the same-side or
  opposite-side rule of the operation (the standard coplanar rule: keep for `Union` and `Intersect` when normals agree,
  drop when they oppose).
- The operation keeps the triangles its truth table keeps (`Union`: outside the other or on the boundary, agreeing
  normals; `Cut`: first outside, second inside flipped; `Intersect`: inside the other).
- The kept triangles become a `facetedrecord.Payload` through the same builder the mesh route uses, so every consumer
  (`Verify`, `Fillet`, `Shell`, `Volume`) works unchanged. Faces regroup by plane, as `SolidFromMesh` does.
- The volume is the exact signed sum over the output triangles in rationals, rounded outward once. The reported bound
  is that rounding plus the vertex bounds, not a chord tolerance.

### 3.5 Soundness

- The kernel only decides with exact signs, so it admits nothing from a residual. It never repairs or infers, which
  keeps the hard rule on reject-only checks.
- Input that is not exact is refused: any operand with a non-zero vertex bound that is not itself a kernel result
  (`ErrUnsupported`). A kernel result carries a bound, so chained use needs its implicit vertices kept (§3.6).
- The output passes the same held audit every mesh body passes (`facetedrecord.Build`). A failure there is a bug and
  returns an error, not a wrong body.

### 3.6 Chained operations

A result vertex has float coordinates that are not the exact point. Feeding the result into a second Boolean as plain
floats would lose exactness. The result therefore keeps its plane triples in the payload (`Payload.Implicit`, new),
and the next Boolean reads planes, not floats. A body without them (placed by a rotation whose images round, or
imported) is an operand only if its vertices are exact floats.

## 4. Fit with the hard rules

| Rule | How the kernel meets it |
|---|---|
| Layering `decad → sketch → r3 → units` | New code is `internal/exactbool` and imports only `r3`, `units`, `proof` |
| Scalars are `units.Value` | Volumes and bounds leave through the existing `Volume` and `Bound` fields |
| No triangles as the representation | The payload is the existing faceted payload; faces are grouped by plane |
| No target-out parameter, no mutation | Same signature as today; operands are not touched |
| Observable correctness | Every phase ships tests against volumes computed independently (§7) |
| No new module | `math/big` only |

## 5. What it closes

| Row | Closed by |
|---|---|
| Coplanar mesh Boolean of tilted planes (S9) | phase 1 |
| Edge-only and point-only contact (S12c) | phase 1 |
| Sub-tolerance gaps on polyhedral pairs (B3 with a polyhedral boss) | phase 1 |
| Mesh bound on a polyhedral tilted cut (S10 with a prism tool) | phase 1, bound drops to rounding |
| `Hole`, `Boss`, `Emboss` and `Draft` on tilted faces without the shift tricks | phase 2, the existing compositions call the kernel |
| Fillet and chamfer on kernel results (oblique planes carry retained tags) | phase 3 |
| Curved faces with exact contact | not closed; phase 4 is research |

Phase 3 matters beyond Booleans: oblique walls of mesh solids carry no retained plane tags today (the probe on an
exact hexagon prism refused "an edge end meets a face that is not a retained plane"), so no mitre or vertex blend
reaches them. Plane tags on kernel results remove that limit.

## 6. Cost and risk

| Item | Estimate |
|---|---|
| New code | about 6,000 lines in `internal/exactbool`, plus the payload field and routing |
| Speed | The filter keeps nearly all predicates in float. A rational fallback costs microseconds and runs only for degenerate input, which is the input that refuses today |
| Memory | Implicit vertices add three ints per result vertex |
| Largest risk | Coplanar clipping and the same-side rule give wrong inside/outside when a pair shares many planes. Mitigation: differential tests against the grid route on axis-square pairs (the grid result is already exact) |
| Second risk | Chained Booleans grow plane-triple depth only through planes, never through coordinates, so depth stays bounded; a test builds a 50-step chain and compares volumes with the closed form |

## 7. Tests

Each phase's tests are written before its code and shown to fail against a mutated kernel (the repo's fixture rule).

- Differential: random axis-square pairs against `orthoshell.Combine`, volumes equal to the last bit and face sets equal.
- Rotation: a box and the same box turned 45° about z, all three operations, against closed-form volumes
  (`Intersect` is the regular octagon prism).
- Contact: edge-only, point-only, coplanar, nested, and a one-micron gap, each against the expected topology.
- Mutants: swap the coplanar same-side rule, drop the rational fallback, compare plane triples by float. Each must turn
  a named test red.
- Chain: fifty alternating cuts and unions of tilted prisms, volume against an independent polygon-area integral.

## 8. Phases

| Phase | Content | Done when |
|---|---|---|
| 0 (shipped) | Design doc with layout row; predicates and implicit points in `internal/exactbool` | predicates agree with the rational reference on 6×10⁴ mixed and 4×10⁴ rounded-coplanar quadruples |
| 1 | Arrangement, classification, output for two exact-vertex polyhedra, routed after the grid route and before the mesh route | S9, S12c and the tolerance rows build exactly; differential test green |
| 2 | Chained operands via `Payload.Implicit`; `PatternJoin` and tool features call the kernel | 50-step chain test green; shift tricks for polyhedral pairs removed |
| 3 | Retained plane tags for every kernel face; oblique mitre and vertex blend | fillet of an exact hexagon prism's top loop builds |
| 4 | Curved faces: quadric–plane and quadric–quadric curves with certified enclosures | separate proposal; not part of this one |

Phases 0–2 are one PR each, merged on green, with the design doc on the same branch as its implementation.

## 9. Other kernels the remaining rows ask for

| Kernel | Rows | Relation to this proposal |
|---|---|---|
| Vertex blend (three filleted edges meeting at a corner) | Fillet of box corners | independent; a design is partly worked out (sphere octant at each corner, setback `r` on each edge) |
| Variable-radius and face-to-face fillet | two rows with no entry point | independent; needs a public API decision first |
| Curved-surface exact contact | tangent and shallow-angle rows | phase 4 above |
| Ellipse coil profiles | `CS7` | independent; interval arithmetic in `internal/coil` |

## 10. Decisions

| Decision | Choice |
|---|---|
| First kernel | Polyhedral. It closes the §5 rows with no new public API and has an exact differential oracle (the grid route). Curved faces are a separate proposal (phase 4). |
| Exact chaining | `Payload.Implicit` carries each result vertex's three planes, so a second Boolean reads planes and not rounded floats. |

## 11. Phase 0 (shipped)

`internal/exactbool` holds `Orient3D` (a float determinant with Shewchuk's error bound, then exact dyadic arithmetic
from `internal/proof`), `Plane` (`PlaneThrough`, `SideOf`, `SideOfPoint`), `Point` (`Meet`, `Order`, `Equal`, `Float`).
`predicates_test.go` compares every sign with a `big.Rat` reference on integer-lattice quadruples (many exactly
coplanar), ulp-nudged quadruples, wide random floats and rounded coplanar quadruples, and checks that three planes meet
at a point that lies on all three exactly. Setting the filter bound to zero fails the rounded-coplanar test; dropping
the determinant sign in `SideOfPoint` or `Order` fails the implicit-point test.
