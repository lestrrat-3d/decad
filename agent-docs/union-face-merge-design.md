# Union Face Merge Design

Merge adjacent, exactly coplanar, same-facing planar faces of different operands in a mesh Boolean result into one
face: current behavior (§1), required behavior and definitions (§2), approach (§3), the merge rule (§4), provenance and
history (§5), scope across ops and routes (§6), interaction with coplanar contact (§7), soundness (§8), performance
(§9), tests (§10), risks (§11), open questions (§12), tasks (§13).

Companion to `agent-docs/evaluator-design.md` §3 ("Canonicalize at build") and §9 ("Ordinary mesh output"), and to
`agent-docs/coplanar-contact-design.md` (answers its §15 Q1). Adds no public API and no option.

## Router

Navigation, not authority.

| Question | Section |
|---|---|
| What does decad do today with a flush Union? | §1 |
| Which faces merge, exactly? | §2, §4 |
| Where does the merge run, and why there? | §3 |
| What roles, tags and history does a merged face carry? | §5 |
| Does it apply to Cut, Intersect and the analytic routes? | §6 |
| Why must coplanar contact land first? | §7 |
| Why can a merge never change geometry or hide a real edge? | §8 |
| What does the implementer build, in what order? | §13 |

## Assumptions

- The merge runs only on the generic mesh Boolean's public result (`booleandispatch.meshBody`). Internal callers of
  `booleaneval.Evaluate` (Verify, rounded fillet, cone trim, saddle tool) get unmerged payloads, because several read
  source groups by operand face ordinal.
- Faces of the SAME operand never merge with each other directly. Their split was the operand builder's own decision
  (evaluator §3 keeps coplanar Loft rungs apart). They can end up in one face through a face of the other operand that
  is adjacent to both (§4.4).
- "Exact" means the result's own vertex bound `β(v) == 0` for every vertex of every facet of the group
  (`Payload.VertexBound`, `agent-docs/faceted-vertex-bounds-design.md` §2). No new exactness notion.
- A payload carrying a structural record (`ObliqueBoss`, `CrossCylinderBore`) is not merged. Those records feed
  record-based rebuild routes proved against the unmerged face set. `PrismUnionFillet` does not block the merge:
  every Union of two prism bodies carries it, its consumers require single-role faces at the selected edge and
  decline on a merged face, and its rebuild runs a fresh Boolean from the source prisms. An edge beside a merged face
  therefore takes the ordinary route, whose volume bound is looser.
- decad tests use neutral minimal fixtures and name no consumer model (`agent-docs/testing.md` "Test scope").

## 1. Current behavior

Measured on `origin/main` at `3427283341` (dated illustration, 2026-11-18; probe kept in
`.tmp/zz_scratch_probe_test.go.txt` of the design worktree, not committed).

| Pair | Route | Result |
|---|---|---|
| box A `[0,1]³`, box B `[0.5,1.5]×[0,1]×[1,2]` built on an offset sketch plane | brep (`TryStackedUnion`) | 10 faces; each y wall is ONE face (`face(4)`, `face(6)`). Coplanar walls are already merged. The merged walls carry only fresh roles: `boolupstream` attributes a face only when exactly one operand face lies on its plane. |
| A, B side by side, equal sweep interval | prism (`prismboolean.Try`) | 6 faces; merged sides carry every segment role (`side(0,0) side(0,1) side(0,2)`) |
| A `[0,10]³`, B the same box moved by `Transform` to `(0.5, 0, 10)` | mesh path | 12 faces; B shifted by the gap retry (y walls at `-2.1e-6`), `CodeGapShiftApplied`. Walls not even coplanar. |
| same, with `coplanarcontact.With(ctx, true)` | mesh path | 12 faces, no shift, every face a retained `Plane`; A's and B's y walls are separate faces on one plane (seam edge) |
| host box + perpendicular hexagonal prism with oblique flanks, chamfered, ends flush with the host's ends | mesh path | 15 faces; tail shrunk by the gap retry (ends at `y = 1.88e-6` and `15.999998`), two `1e-5 mm²` sliver faces, tail faces `Faceted`. The rendered seam is a real 1.9 µm step. |
| same, with `coplanarcontact.With(ctx, true)` | mesh path | 14 faces, no shift, all faces retained `Plane`, every end-face vertex `β = 0`; host end face (`side(0,0)`) and tail end face (`capEnd`) are separate faces on `y = 0`; same at `y = 16` |

Findings:

- **The claim "nothing merges coplanar faces of a Union result" is false for the analytic routes.** Prism, stacked
  and brep results build one face per exact planar region. `solidFromMesh` (the orthogonal route) groups facets by
  exact oriented plane (`meshplane.Groups`). The plain flush box Union of the earlier probe went through the brep route
  and its 10 faces ARE the merged result.
- **The mesh path never merges.** `facetedrecord.assembleBody` builds one face per connected patch of one source group
  (`facetproof.SourcePatches`, evaluator §9). Two operand faces are two groups, so they stay two faces.
- **Today the dovetail-like pair is not flush at all.** The gap retry moves the second operand. Coplanar contact
  (`agent-docs/coplanar-contact-design.md`) removes the move. After it lands, the faces are exactly coplanar and
  exact, and only this design's merge is missing.
- `topology.go` `Face.Origins` states that canonicalization may merge coplanar faces and a merged face carries all
  contributing roles. That holds for prism segment canonicalization (evaluator §3). It does not hold for the brep
  route's merged walls, which lose the operand roles (§12 Q2).

## 2. Correct behavior and definitions

**Rule.** In a mesh Boolean result, two faces that come from different operands merge into one face when they are
adjacent, both exact, on the same exact carrier plane, and face the same way. The merged face carries every role and
history record of every face it joins.

| Term | Definition |
|---|---|
| Adjacent | The two faces' held facets share at least one held edge of positive length (an edge of `facetproof.MeshAudit.Adj`). Faces that meet only at a vertex are not adjacent. |
| Exact | Every vertex of every facet of the face's source group has `β(v) == 0`. Then each held facet is the true boundary piece (`Payload.MeshBound` doc), so the held polygons ARE the true face. |
| Same exact carrier plane | All facets of both groups lie on one plane as exact rationals over the held float coordinates: `meshplane.Groups` gives them one group (normal `(b−a)×(c−a)` and offset `n·a` equal up to a positive factor). |
| Orientation | The facets' winding gives the outward normal. `meshplane.Groups` keys on the oriented plane, so one group means equal outward normals. Opposite-facing faces on one plane never merge. |
| Material side | The side opposite the outward normal. Equal outward normals put the material of both faces on the same side, so the shared edge has a flat (half-turn) material angle and is no feature of the solid. |
| Holes | A merged region may have holes, including a hole neither face had (a ring closed by the other operand). The outer loop is chosen by evaluator §9's area-vector rule. |
| Non-simply-connected, pinched | A merged region whose boundary touches itself at a vertex (4 or more region-boundary edges at one vertex) is NOT merged; its faces stay as they are (§4.4). |
| Planes equal within a bound | Not merged. A face with any `β > 0`, or two planes that differ in any bit of the held rationals, keeps its own face and the seam edge stays. Only exact carrier planes merge. |

**Out of scope:** curved faces; faces of one operand with each other (Assumptions); the analytic routes, which already
merge (§6); faces with a positive bound whose planes some record proves equal (§12 Q4); rewriting the operand roles the
brep route drops (§12 Q2).

**Success criteria.**

- **S1.** After coplanar contact is on, the flush host-box + perpendicular trapezoid-prism Union has exactly one face on
  each end plane, and no edge whose two faces both lie on that plane.
- **S2.** The `Transform`-placed flush box-on-box Union has 10 faces, each y wall one face.
- **S3.** Volume, area, `MeshBound`, `VolSymDiff`, every vertex position and bound are bit-identical with and without
  the merge (§8).
- **S4.** `FaceCreatedBy` on any contributing role selects the merged face; a merged face reports no role that did
  not contribute to it.
- **S5.** No boolean that succeeds today fails because of the merge.

## 3. Approach

Relabel the result's source groups before assembly. `booleandispatch.meshBody` calls a new pure function,
`facemerge.Merge`, on the evaluated payload. It finds each set of adjacent exact coplanar same-facing patches from
different operands, appends one merged `facetedrecord.Group` per set, and relabels those facets (`Src`) to it. Assembly,
topology, placement, tessellation and every later consumer then see an ordinary group whose facets form one patch.
Nothing else in the faceted pipeline changes.

Why this works with no other change:

- `facetedtopology` breaks edge chains only where the face pair changes (`breakAt`). A seam inside one face is no
  boundary, and a former seam end with two remaining edges of one face pair becomes a chain-interior point. Edges and
  vertices come out maximal without new code.
- Placement copies the payload (`next := fp`), and rewriters (`retainedblend`, `trimrebuild`) keep `Src` and `Groups`.
  A merged group survives them as any group does.
- Every consumer that assumes one group per face (`retainedblend.groupGeom` keys a face by its first facet's `Src`)
  stays correct.

**Alternatives considered.**

- *Face-level merge in `assembleBody`, keeping groups and storing a per-group merge class in the payload.* Rejected:
  it breaks the one-group-per-face assumption that `retainedblend` and others rely on, and every payload rebuild site
  would have to carry the class.
- *Merge in `facetedrecord.BuildWithProof` for every faceted builder.* Rejected: structural builders (loft, revolve
  shells) and internal `Evaluate` callers read groups by operand ordinal, and their face splits are their own decision.
- *Merge inside `booleaneval.evaluate` (live and replay).* Rejected: Verify and internal callers would pay for it and
  see relabelled groups; replay would need budget-charged parity for no consumer.
- *Group-level relabel (move all of group g2 into g1).* Rejected: a patch of g1 that touches no g2 patch would report
  g2's roles, violating S4.
- *Tolerance-based coplanarity.* Rejected: a merge on planes equal only within a bound could join faces whose true
  planes differ (§8).

## 4. The merge rule

### 4.1 Input

`facemerge.Input` holds the stitched result: `Verts`, `VertexBound`, `Tris`, `Src`, `Groups` (from
`booleaneval.Evaluation.Payload`), `Adj` (`Evaluation.Audit.Adj`), and `OperandAGroups` (groups `[0, OperandAGroups)`
came from the first operand, the rest from the second; `booleaneval.evaluate` appends A's faces first, eval.go "for i
:= range totalFaces").

### 4.2 Candidate groups

Group `g` is a candidate when all hold:

1. it owns at least one facet, and `Groups[g].Planar`;
2. `Groups[g].Surface == nil`, or `Groups[g].Retained` with a `surfacegeom.Plane` surface, `Denoted == nil`,
   `ExtentDelta == 0` and `!ViewCharge` (a tag whose validity depends on the face's extent cannot move to a larger face);
3. every vertex of every facet of `g` has `VertexBound == 0`;
4. every facet of `g` lies in one `meshplane.Groups` group that is `Planar` (run `meshplane.Groups` once over the
   candidate facets of steps 1–3 only). A zero-area facet is its own non-`Planar` group and so rejects its source group.
   Measured with the coplanar switch on (dated illustration, 2026-11-18): R1 and R2 (§10.2) hold no zero-area facet.
   If a later result's end-face group holds one, S1 fails for it; §12 Q5 owns that case.

### 4.3 Merge edges and components

1. `patch := facetproof.SourcePatches(budget, len(Tris), Src, Adj)`.
2. For every facet `t` and neighbour `u ∈ Adj[t]` with `u > t`: when `Src[t] != Src[u]`, both groups are candidates,
   exactly one of them is `< OperandAGroups`, and `t` and `u` have one `meshplane` group, union `patch[t]` and
   `patch[u]` (union-find).
3. A component is a union-find class of two or more patches.

### 4.4 Admission of a component

`Adj[t][k]` is the facet across held edge `(Tris[t][k], Tris[t][(k+1)%3])` (`internal/facetproof/faceted_adjacency.go`);
the boundary count below walks those slots. A component is merged only when its region boundary is a set of vertex-disjoint simple loops: for every vertex `v` on
a component-boundary edge (a held edge with exactly one adjacent facet in the component), the number of
component-boundary edges at `v` is exactly 2. Otherwise none of its patches change. Transitive joins are allowed: two
patches of A both adjacent to one patch of B join through it.

### 4.5 Output

- For each admitted component, in increasing order of its smallest facet index, append one merged group and relabel
  every facet of the component to its index. Original groups keep their indices; a group whose facets all moved stays
  in `Groups` with none (orphan groups already occur for operand faces the boolean removes).
- `changed == false` returns the input slices unchanged.
- `OperandAGroups > len(Groups)`, or any length mismatch among `Tris`, `Src`, `Adj`, `Verts`, `VertexBound`, is
  `decaderr.ErrBooleanFailed`.

### 4.6 The merged group

Members are the distinct groups of the component, in increasing index.

| Field | Value |
|---|---|
| `Origins`, `Upstream` | concatenation over members in order, first occurrence kept |
| `Source` | `&SourcePatch{Producer, Operand, Ordinal: copied from Members[0], Members: each member's own Source, in order}`; `Parent` nil, so ancestry readers walk `Members` |
| tag donor | the first member with `Retained` and a `Plane` surface; none when no member has one |
| `Surface`, `Retained`, `CapRecord`, `AxialDelta`, `HasAxialDelta`, `NormalBound`, `Reversed` | the donor's; zero values without a donor |
| `Planar` | true |
| `Centroid`, `HasCentroid`, `Denoted`, `ExtentDelta`, `ViewCharge` | zero values |

## 5. Provenance, tags and history

| Consumer | Effect |
|---|---|
| `Face.Origins`, `FaceCreatedBy`, `Walls`, `CapStart`/`CapEnd` refs | The merged face lists every member's roles; each selects it (evaluator §3: "a canonicalization merge UNIONS the merged faces' roles"). Selections like `Facing(-y).Exactly(2)` that counted both faces now see one (counts churn, `api-design.md` §3). |
| `topo.Face.Upstream` | union of the members' `Upstream`; `Origins()` appends it as today |
| `topo.Face.Source` (`provenance.SourcePatch`) | merged record with `Members`. No production reader of `Face.Source` exists today besides the shift-certificate seals (Task 2 makes them walk `Members`). When operation history implements "Face correspondence" (`operation-history-design.md`), it must expand `Members`, which keeps every incoming relationship as that doc requires. |
| retained `Plane` tag, `CapRecord` | the donor's; valid for the whole merged face because every member lies on the donor's true plane (§8 step 4). Sketch-on-face (reading `Face.Surface().(Plane).Frame`) and `ToFace` read it. |
| `facetedrecord.SourceTag` in a later Boolean | reads the merged face's tag and `Source` as for any face; the merged face becomes one source group of the next result |
| `boolchain.Expandable`, regroup, Section from history | unchanged: the merge adds no `OperationWarnings` entry and keeps the payload type |
| placement, `Copy`, `Transform`, `ScaledAt`, `Refine` | copy `Src` and `Groups`; the face set is preserved |
| `retainedblend`, `trimrebuild` rewrites | keep groups; the merged group is an ordinary planar group |
| consumers reading groups by operand ordinal after `meshBody` (`LowerSupport.SourceGroup`, `LoftFacesCap.Group`) | read original indices. `LowerSupport` needs the other operand strictly above the base's lower face, so that face has no coplanar partner and never merges. Any other such reader sees an orphan group and declines; its own proof stays sound. |

## 6. Scope across ops and routes

- **Union, Cut, Intersect: all three.** The rule reads only the result boundary, never the op. Each op can produce the
  defect: a `Cut` that removes a step down to a lower face of the target leaves the tool's reversed face next to the
  target's face; an `Intersect` of two stepped prisms leaves A's top on one part of a plane and B's top on the next.
  Fusion's Join, Cut and Intersect all canonicalize these.
- **Analytic routes** (prism, stacked, brep, class B, revolve, loft, orthogonal): unchanged. They already build one
  face per exact planar region (§1). Their provenance gap is §12 Q2.
- **Shifted results** (`eval.Shift != nil`): skipped. A moved operand's faces carry the move in `β` and are never
  candidates; the skip makes it explicit.

## 7. Interaction with coplanar contact

- **Dependency.** Two result faces from different operands can share an exact edge on one plane only if the operand
  faces touched coplanar along it. Today that contact is refused and the gap retry moves an operand, so the merge
  never fires. Coplanar contact Task 10 (switch removal) makes such results exist. Wiring (Task 3 here) depends on it
  and on coplanar contact Task 11, whose API tests count faces this merge changes.
- **Q1 there.** `coplanar-contact-design.md` §8 kept two faces and §15 Q1 deferred the merge. This design answers Q1
  with the merge. Task 5 updates §8, §12 and §15 there to current state.
- **Exact-face reading.** Coplanar contact admits operand facets on `FacetBound == 0` plus a zero face gate reading
  (§3.1 there), and its §7 makes result faces with float corners hold `β == 0`. This design reads the result's own
  `β`, which is that same zero bound one step later. Measured with the switch on: every end-face vertex of the
  dovetail-like result already reads `β = 0`, before §7 lands.
- **Planar simplification (§7 there)** runs inside the stitch, per source id, before the merge. The merge changes no
  vertex, so it needs no re-simplification. Former seam vertices stay as chain-interior points of maximal edges.

## 8. Soundness

Claim: a merge changes no point of the held or true boundary, no measurement and no bound, and joins only faces whose
true union is one planar face of the solid.

1. **Held equals true.** A candidate's facets have `β = 0` at every vertex, so `δ(t) = 0` and each held facet is the
   true boundary piece (`Payload.MeshBound`: no true boundary point lies farther than `δ(t)` from its held piece).
2. **Exact identity.** `meshplane.Groups` decides plane identity by exact dyadic arithmetic over the held floats, with
   no residual and no threshold. By step 1 the true faces lie in the planes the held facets span, so equal held
   oriented planes mean equal true oriented planes. This is an exact proof over decad's own held data, not an
   admission gate on a residual (CLAUDE.md "A decad-side check may only FALSIFY"), and it computes no 2D answer that
   `sketch` would own.
3. **No hidden edge.** The two faces share a held edge (adjacency through `Adj`, which the closure audit built as a
   2-manifold), lie in one plane and have equal outward normals. Material lies on one side of the plane on both sides
   of the edge, so the material angle there is a half turn: the edge is no edge of the solid. A real edge between two
   faces has a different plane or opposite normals on one side, and §4.3 never joins those.
4. **Tags stay valid.** The donor's `Plane` tag denotes the donor's true plane, which by step 2 is every member's plane.
   Its `NormalBound` covers the tag's departure from that plane and does not depend on extent; extent-dependent tags
   (`ExtentDelta`, `Denoted`, `ViewCharge`) are not candidates. `CapRecord` names the same plane. `Reversed` describes
   the tag against the outward normal, which all members share.
5. **No geometry change.** `Verts`, `Tris`, `VertexBound`, `MeshBound`, `VolSymDiff`, `AreaSlack`, `DPair`, the volume
   and its proof are untouched; only `Src` and `Groups` change. Assembly integrates the same facets, so body area and
   volume are bit-identical; face areas partition the same sum.
6. **Topology stays valid.** §4.4 admits only regions bounded by vertex-disjoint simple loops, which
   `facetedtopology` already traces for ordinary multi-loop planar patches. Patches that do not admit keep today's
   faces.
7. **Provenance is exact.** Membership is per patch, so a merged face lists exactly the groups whose facets it holds.
8. **Placement.** A rigid or similar image keeps true coplanarity and orientation, so a merged group stays a correct
   statement about the image even after its held coordinates round.

## 9. Performance

- Cost per mesh Boolean result: one pass over facets and `β` (linear), `meshplane.Groups` over candidate facets only
  (one exact dyadic cross product and plane key per facet), one `SourcePatches` pass (assembly runs it again later),
  one pass over `Adj` (3 per facet), union-find, and the boundary check (linear in component facets). All linear; no
  facet-pair work.
- Against the Boolean itself (exact facet-pair predicates, stitch, closure audit, assembly) this is expected well under
  5% of a Union's time. Task 2's `BenchmarkMerge` and Task 3's before/after `go test -bench` numbers in the PR verify it.
- Fewer faces make assembly and later selection marginally cheaper. Verify and other internal `Evaluate` callers pay
  nothing (§3).

## 10. Test plan

Every test asserts computed geometry: face counts, loop counts, roles, surface kinds, volumes, bounds. "Seam check"
means: first assert every face of the body is planar (`Plane` surface, or `NormalTo`-selectable through the public
API), failing otherwise; then assert no edge has two faces with outward normals of dot product `> 1 − 1e-12` and plane
offsets within `1e-9 mm` (test-side tolerance; production never uses one).

### 10.1 Unit tests (`internal/facemerge`, hand-built closed meshes, all `β = 0` unless stated)

Base fixture: a 2×1×1 box triangulated with a vertex row at `x = 1` on its four faces parallel to x; on each of those,
the `x < 1` half is a group of operand A and the `x > 1` half a group of operand B. The `x = 0` face is one A group and
the `x = 2` face one B group. Non-candidate marker for U5 and U6: a retained `Plane` group with `ExtentDelta > 0`
(rule 2), which `AssembleBody` handles like any retained plane.

| ID | Test function | Case | Expectation |
|---|---|---|---|
| U1 | `TestMergeSplitBox` | base fixture | 4 merged groups appended; the `x = 0` and `x = 2` facets keep their groups; `facetedrecord.AssembleBody` gives 6 faces, 12 edges, 8 vertices |
| U2 | `TestMergeSameOperand` | base fixture, every group in operand A | `changed == false` |
| U3 | `TestMergePositiveBound` | base fixture plus one vertex `(0.5, 0.5, 1)` inside the top's A half (fanned), with `β = 1e-15` | the top pair stays unmerged; the other 3 pairs merge |
| U4 | `TestMergeOffsetPlane` | base fixture with the two top vertices at `x = 2` raised by `2^-40` (the top's B half tilts; every other face stays planar) | the top pair stays unmerged |
| U5 | `TestMergePinch` | 3×3-cell top over a 3×3×1 box; the centre cell and the corner cell `[2,3]×[2,3]` are marker groups (no candidate); the other 7 top cells alternate A and B | no top facet changes group (the C-shaped region touches itself at vertex `(2, 2, 1)`) |
| U6 | `TestMergeRing` | as U5 but only the centre cell is a marker group; the 8 ring cells alternate A and B | one merged top group; its assembled face has 1 outer and 1 inner loop |
| U7 | `TestMergeGroupFields` | base fixture with distinct roles, `Upstream`, `Source`, one member `Retained` `Plane` with `CapRecord` | §4.6 table, field by field; `Source.Members` length 2 in group order |
| U8 | `TestMergeTagGates` | one member with `ExtentDelta > 0`; then `Denoted != nil`; then a `Cylinder` surface | that pair unmerged in each case |
| U9 | `TestMergeDeterministic` | base fixture twice; then with A's groups permuted among themselves and B's among themselves | equal output for equal input; under the permutation, the same facet partition into merged groups and merged groups ordered by smallest facet index (`Origins` order follows group index and may differ) |
| U10 | `TestMergeBentGroup` | one group's facets on two planes | that group is no candidate |

### 10.2 Root tests (`union_face_merge_test.go`, package `decad`)

Run after coplanar contact Task 11. Each asserts the result payload is `facetedrecord.Payload` (the mesh path). If a
fixture reaches an analytic route instead, stop and report; do not change production code to force the route. Every
fixture below was measured on the mesh path with the coplanar switch on (dated illustration, 2026-11-18; "before"
counts in brackets).

| ID | Case | Expectation |
|---|---|---|
| R1 | A = 10×10×10 box at the origin; B = the same box moved by `Transform` to `(0.5, 0, 10)`; `Union` | 10 faces [12]; one group each for `y = 0` and `y = 10` holds facets of both operands; that group's `Source.Members` has 2 entries; volume 2000 exact; seam check passes |
| R2 | host: rectangle `[-10,10]×[0,16]` on XY extruded 10 `Against` (toward −z); tool: profile `(-5,-1) (5,-1) (5,0) (6,6) (-6,6) (-5,0)` on the plane through the origin with axes +x, +z (normal −y), extruded 16 `Against` (toward +y); `Union` | 10 faces [12]; exactly one face on `y = 0` and one on `y = 16`, each with one loop, which is outer; each lists the host's `side` role and the tool's cap role; seam check passes; volume 4256 exact (`3200 + 16·66`) |
| R3 | R1's result moved by `Transform` (translation, then an exact quarter turn about z) and by `Copy` | face count, loop counts and every face's `Origins` unchanged |
| R4 | R1 pair with B rotated 30° about z then −30° (rounding vertices, `coplanar-contact-design.md` §11.3) | `CodeGapShiftApplied`; no merged group; 12 faces |

### 10.3 API tests (`apitest/union_face_merge_test.go`)

| ID | Case | Expectation |
|---|---|---|
| A1 | R1 pair | `Faces(Facing(-y)).Exactly(1)` resolves; `FaceCreatedBy` with A's and with B's `side(0,0)` role each select that one face; surface `Plane`; no operation diagnostic |
| A2 | R2 pair | `Facing(-y)` and `Facing(+y)` each `Exactly(1)`; seam check; volume within bound |
| A3 | `Cut` step-down: target profile `(0,0) (20,0) (19,10) (10,10) (10,5) (0,5)` on R2's tool plane, extruded 10 `Against`; tool: box `[0,11]×[0,12]×[0,6]` moved by `Transform` by `(10, -1, 5)` | 6 faces [7]; `Facing(+z)` `Exactly(1)` with both the target's and the tool's roles; volume 987.5 exact |
| A4 | `Intersect` of stepped prisms on R2's tool plane, each extruded 10 `Against`: A profile `(0,0) (12,0) (12,20) (6,20) (6,10) (0,10)`; B profile `(1,0) (13,0) (13,10) (4,10) (4,20) (1,20)` moved by `Transform` by `(1, 0, 0)` (B's high part spans x `[2,5]`, so no riser touches) | 6 faces (box `[2,12]×[0,10]×[0,10]`); `Facing(+z)` `Exactly(1)` with roles from both (A's top bounds x `[2,6]`, A's copy where both tops coincide on `[5,6]`; B's top bounds `[6,12]`); volume 1000 exact |
| A5 | R1 pair with B moved by `(0.5, 2^-40, 10)` | the two `Facing(-y)` faces stay separate (a sliver of A's top lies between them) |
| A6 | R1 result, then `Chamfer` of the edge between B's top and the merged `-y` face, distance 1 | succeeds; volume drops by `0.5·1·1·10` within bound; seam check passes on the result. If `Chamfer` refuses, stop and report the refusal; do not weaken the test |
| A7 | R1 result, then `Cut` of a 2×2×2 pocket box moved by `Transform` to `(4, 4, 19)` (into B's top) | the Cut succeeds on the mesh path; the merged `-y` and `+y` faces are each still one face with both operands' roles |

A3, A4 and A7 were not all measured. If any of them refuses, or its result is not a Faceted mesh result (check
in a root test as §10.2 does), stop and report; do not change production code to force the route.
Measured on Task 4: A4 takes the prism route and a mesh-path variant refuses the coplanar overlap, so A4 pins the
route's own one-face result and the mesh merge has no Intersect coverage. A7 with a box pocket takes the orthogonal
route, which mints fresh roles, so Task 4 uses a cylindrical pocket (radius 1 at (5, 5), z from 19 to 21), which
stays on the mesh path.

### 10.4 Existing tests this change flips

Tests that count faces, edges or vertices on mesh results of flush pairs: at least the coplanar contact API tests
(`apitest/coplanar_contact_test.go` P1 and any face-count assertion on A1, A2, A5, R1–R4), tests asserting two
faces on one plane, and `faceted_group_source_test.go` / `faceted_group_source_identity_test.go` where they assert a
per-face `Source` on a merged face. Find them with `go test ./... 2>&1 | grep -E '^(--- FAIL|FAIL)'` after Task 3. Rule: update a
count only where the difference is exactly the merged pairs, and list each updated test in the PR. Any other failure
is a regression: stop and report.

### 10.5 Consumer acceptance (gallery; not committed in decad)

In the gallery worktree with a `replace` of decad to this worktree: build the dovetail demo for every parameter set of
its tests. Required: the tail block has exactly one face on each end plane and passes the seam check; no
`CodeGapShiftApplied`; the rendered end faces show no line across the tail root.

## 11. Risks

| Risk | Effect | Mitigation |
|---|---|---|
| A reader of groups by operand ordinal after `meshBody` meets an orphan group | that record-based route declines on a merged result | skip list (Assumptions); §5 table; full suite after Task 3; stop and report any decline |
| `ToFace`/`MirrorFace` require a convex outline on a tilted face; a merged face can be non-convex | a stop that worked on one of the halves refuses on the merged face | the stop reads the face the user selects; record any flipped test; §12 Q3 |
| Face/edge count churn for callers using `Exactly(n)` | caller assertions fail | counts are not contract (`api-design.md` §3); docs state the rule (Task 5) |
| Merge on results of coplanar contact tests while that series is still landing | file conflicts in shared test files | Task 3 depends on coplanar contact Task 11; Task 5 on its Task 12 |
| A pinch or loop shape the boundary check misses | topology build error turns a success into a failure (S5) | §4.4 check; U5, U6; Task 3 full suite |

## 12. Open questions

- **Q1. Merge faces of the same operand too?** Option A, never (designed): gains stable face sets for operands whose
  builder kept coplanar faces apart on purpose (Loft rungs, evaluator §3); costs leaving seams an operand already had.
  Option B, any pair: gains removing inherited seams; costs overriding evaluator §3's Loft rule, more test churn, and
  face-count changes on results whose seams this boolean did not make. Recommendation: A, because it changes only seams
  the boolean creates.
- **Q2. Restore operand roles on the brep route's merged walls?** Option A, leave (designed here): no cost now; a
  brep-route flush wall answers `FaceCreatedBy` only for its fresh `face(i)` role, unlike the mesh route. Option B,
  extend `boolupstream` to give a face the roles of every operand face on its plane that it overlaps: gains one
  provenance answer across routes; costs an exact area-overlap proof per face and operand face (today it reads planes
  only), a separate design. Recommendation: B as a follow-up design, because selection by an operand role differs by
  route today.
- **Q3. Keep a non-convex merged face as one face when `ToFace` needs a convex outline?** Option A, merge anyway
  (designed): gains the Fusion-like face set; costs a `ToFace` stop onto such a tilted merged face that the separate
  halves would have accepted (whether any test hits it is unknown until Task 3's full run). Option B, skip components
  whose union is non-convex: keeps `ToFace` reach; costs the seam on exactly the dovetail-like faces (they are
  non-convex), failing S1. Recommendation: A, because B defeats the purpose.
- **Q4. Merge bounded planar faces whose planes a record proves equal (`CapRecord`, `capplane`)?** Option A, no
  (designed): no such adjacency arises while coplanar contact refuses bounded faces (its §4). Option B, yes: gains
  merges once bounded contact is admitted; costs a record-identity path for side walls (which carry no `CapRecord`) and
  a proof that the donor tag's bound covers the merged extent. Recommendation: A until a route admits bounded coplanar
  contact.

- **Q5. Zero-area facets in a candidate group.** Option A, a zero-area facet rejects its group (designed): simple;
  costs a missed merge, with no message, if a result ever holds such a sliver on a coplanar face (none in R1 or R2,
  measured). Option B, treat a zero-area facet whose vertices lie exactly on the group's plane as neutral: gains
  robustness to stitch slivers; costs one exact on-plane test per such vertex and a §4.4 rule for its edges.
  Recommendation: A until a test or consumer shows a sliver.

## 13. Tasks

Each task merges alone with `go test ./...`, `go vet ./...` and `golangci-lint run` passing. A Sonnet implementer
builds each task; a fresh Opus `design-doc-implementation-reviewer` reviews it before merge. Production PRs carry the
Module boundary gate evidence (`agent-docs/module-boundary.md`): owning section §4 of this doc, input class "planar
faces of a generic mesh Boolean result (Union, Cut, Intersect)", the focused tests, and no new root adapter.

### Task 1: Land the design doc

- **Files:** `agent-docs/union-face-merge-design.md`, `agent-docs/layout/design-docs.md`.
- **What:** Commit this doc and a layout row under 300 characters.
- **Depends on:** none.
- **Done when:** `go test . -run '^TestCLAUDEMDLayoutStaysCompact$' -count=1` passes.

### Task 2: `internal/facemerge` and the merged source record

- **Files:** `internal/provenance/source_patch.go`, `internal/provenance/source_patch_test.go` (new),
  `internal/booleaneval/shift_certificate_capture_seal.go`, `internal/booleaneval/shift_certificate_snapshot.go`,
  `internal/facemerge/merge.go` (new), `internal/facemerge/merge_test.go` (new),
  `internal/facemerge/merge_bench_test.go` (new), `agent-docs/layout/booleans.md`.
- **What:**
  - Add `Members []*SourcePatch` to `provenance.SourcePatch`, with a doc comment: set only on the record of a group a
    canonicalization merge joined (agent-docs/union-face-merge-design.md §4.6); lists each member's record in group
    order; `Producer`, `Operand` and `Ordinal` copy `Members[0]`; `Parent` is nil, so ancestry readers walk `Members`.
  - The slice makes `SourcePatch` non-comparable. Add `func (p SourcePatch) Same(q SourcePatch) bool` (every scalar
    field and `Parent` equal; `Members` equal in length and element pointers). Replace `*source.ptr != source.value`
    (capture seal) and `*x.ptr != x.value` (snapshot) with `!...Same(...)`. Make both seals' ancestry walks (count and
    collect) visit `Members` as well as `Parent`, under the existing depth 64 and budget charges.
  - Then §4 as

  ```go
  // Input is a mesh Boolean result's held mesh, source groups and closure-audit adjacency.
  type Input struct {
  	Verts          []r3.Vec
  	VertexBound    []float64
  	Tris           [][3]int
  	Src            []int
  	Groups         []facetedrecord.Group
  	Adj            [][]int
  	OperandAGroups int
  }

  // Merge relabels every admitted component (agent-docs/union-face-merge-design.md §4) to one appended group.
  // It never modifies in. changed is false when nothing merged; src and groups are then in.Src and in.Groups.
  func Merge(ctx context.Context, in Input) (src []int, groups []facetedrecord.Group, changed bool, err error)
  ```

  Charge `proofbound.NewWorkBudget(ctx)` one `Step` per facet visit, as `facetproof.SourcePatches` does. A length
  mismatch among `Tris`, `Src`, `Adj`, `VertexBound`/`Verts`, or an out-of-range `Src`, is `decaderr.ErrBooleanFailed`.
  No production caller yet.
- **Depends on:** 1.
- **Pattern refs:** `facetproof.SourcePatches` (`internal/facetproof/body.go`); `meshplane.Groups`
  (`internal/meshplane/groups.go`); `meshPlaneGroups` in `solid_from_mesh.go`. U1 and U6 obtain `Adj` from
  `facetproof.AuditFacetedMesh` and assemble with `facetedrecord.AssembleBody`.
- **Tests:** U1–U10 (§10.1). `TestSourcePatchSame`: equal records, a differing scalar, a differing `Members` pointer.
  `BenchmarkMerge`: a 64×64-cell two-operand plate (alternating cells), reported in the PR.
- **Done when:** `go build ./...` succeeds, `go test ./internal/facemerge/ ./internal/provenance/ ./internal/facetedrecord/ ./internal/booleaneval/ -count=1` passes and `go test ./internal/facemerge/ -run '^$' -bench Merge -benchtime 5x -count=1` runs.

### Task 3: Wire the merge into the mesh Boolean result

- **Files:** `internal/booleaneval/eval.go`, `internal/booleandispatch/dispatch.go`, `union_face_merge_test.go` (new),
  `.github/test-shards.txt`, every test file §10.4's rule updates (list each in the PR).
- **What:** Add `Evaluation.OperandAGroups int`, set to `targetFaces` in `evaluate`. In `booleandispatch.meshBody`,
  after the recognizers and before `facetedrecord.BuildWithProof`: when `eval.Shift == nil`, `eval.Audit != nil`, and
  `ObliqueBoss` and `CrossCylinderBore` are both nil, call `facemerge.Merge` with `eval.Audit.Adj`
  and store the returned `Src` and `Groups` in `eval.Payload`. The audit stays valid for `BuildWithProof`: it reads
  only `Verts` and `Tris`. A `Merge` error returns through `hooks.AsBooleanError` like a `BuildWithProof` error.
- **Depends on:** 2, and coplanar contact Task 11 merged on `origin/main` (Task 10 removes `internal/coplanarcontact/`;
  Task 11 adds `apitest/coplanar_contact_test.go`, whose counts this task updates). If either is missing, stop.
- **Tests:** R1–R4 (§10.2). Record `go test ./apitest/ -run '^$' -bench 'Union' -benchtime 5x -count=1` before and
  after in the PR (§9).
- **Done when:** `go test . -run 'TestUnionFaceMerge' -count=1` and `go test . ./apitest/ -run '^TestCI' -count=1` pass
  locally, and the PR's CI (the full suite) is green.

### Task 4: API coverage

- **Files:** `apitest/union_face_merge_test.go` (new), `.github/test-shards-apitest.txt`.
- **What:** A1–A7 (§10.3) as `TestUnionFaceMerge*` functions; a shared `requireNoSeam(t, body)` helper implementing
  §10's seam check.
- **Depends on:** 3.
- **Done when:** `go test ./apitest/ -run 'TestUnionFaceMerge' -count=1` and `go test . ./apitest/ -run '^TestCI' -count=1` pass.

### Task 5: Owning docs to current state

- **Files:** `agent-docs/evaluator-design.md` (§3 "Canonicalize at build": add the mesh Boolean rule as a pointer to
  this doc, and state the exception that two coplanar faces of one operand (Loft rungs included) join when a face of
  the other operand is adjacent to both; §9 "Ordinary mesh output": a patch may join patches of other operands per this doc),
  `agent-docs/coplanar-contact-design.md` (§8 coplanar-rim bullet, §12 last row, §15 Q1: point here),
  `internal/facetedrecord/group.go` (package doc sentence on patches), `topology.go` (`Face.Origins` doc),
  `boolean.go` (`Union` doc: "its faces are grouped by the operands' source faces" gains the merge),
  `docs/api.md` and `docs/data-structures.md` (only where `grep -n -i 'source face\|connected patch\|FaceCreatedBy' docs/api.md docs/data-structures.md` finds the per-source-face statement).
- **What:** State current behavior only; no changelog wording.
- **Depends on:** 4, and coplanar contact Task 12 merged (its doc is no longer being edited).
- **Done when:** `go test . -run '^TestCLAUDEMDLayoutStaysCompact$' -count=1` and `go vet ./...` pass, and
  `grep -n 'stay separate faces on one plane' agent-docs/coplanar-contact-design.md` prints nothing.

### Task 6: Consumer acceptance run

- **Files:** none in decad (scratch module under the gallery worktree's `.tmp/`).
- **What:** §10.5 against the decad commit of Task 4.
- **Depends on:** 4.
- **Done when:** the dovetail demo tests pass with the replace, and a script over the tail block reports one face per
  end plane and no seam edge for every parameter set; report the numbers to the parent.
