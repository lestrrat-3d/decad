# Charged window chamfer design

`Chamfer` of a window rim of a holed A1 stacked BRep after a retained edge blend: the gallery balance wheel's windows
after its Ø14 `Boss` root fillet. Status: approved for building in two increments (§6). W1 is built. W2 is not built.
A top-face window run that meets a root fillet (§5) is out of scope and refused.

Navigation (not authority): what the wheel needs → §1; the oblique walls (W1) → §2; the charged chain (W2) → §3;
readers → §4; the refused run → §5; increments and size → §6.

## 1. Problem

The wheel is a disc of radius 42 with four windows between arms. Each window is a closed loop of an inner arc
(radius 7), two oblique sides, an outer arc and four corner rounds. A Ø14 boss with `RootFillet` 0.5 stands on the top
face (boss-root-overflow C3), a D bore is cut, and every rim and window edge of both caps is chamfered by 0.5.

Probe on `main` at 2172cce965 (scratch copy of the gallery):

| Step | Result |
|---|---|
| boss fillet, bore cut | every swept circular wall keeps its charge (δ = 9.8e-14, retained-edge-blend §6.1); the 8 oblique window sides and the 4 fillet faces are `Faceted` |
| rim circles on both caps | builds |
| a bottom window's arcs | "a tangent chain whose walls state a displacement is not built" |
| a bottom window's sides | `FacetedCurve` rims: the stacked view leaves an oblique swept plane untagged (brep-modify §2.3) |
| windows' inner radius 7.6 instead of 7 | every rim and window chamfers today (the union is C0) |

Two gaps stand between the bottom-face windows and a build: the oblique sides carry no tag (W1, §2), and a tangent
chain reads no charged wall (W2, §3).

## 2. The oblique walls (W1)

### 2.1 Why `AxialDelta` does not fit

The view rewrites a swept wall between canonical vertices (brep-modify §2.3). An oblique plane passes through its
walked ends, and each canonical vertex lies within its allowance of the point it denotes (general-boolean §3,
"Displacements"). So the tag lies within a small distance of the true wall over the face, but it may tilt. A plane's
`AxialDelta` states a parallel offset everywhere, and every plane route reads it so (`planeCharge`, the edge-line rule,
`MirrorFace`, `ToFace`). A tilted tag does not meet that claim away from the face.

### 2.2 What the view states

For each swept planar wall `F` of a stacked or slab-cut record that is not a planar record face, when
`w = max(SectionDelta, CarrierCharge)` is finite and positive (`stackedViewTilt`,
`internal/brepmodify/stacked_view_tilt.go`):

- The sweep axis `a` is the exact image of the swept face's frame normal under the record's placement
  (`Linear(bp.Xform)·Frame.N()`, read as rationals). The true wall is swept along `a` by the record's definition.
- The tag is `F`'s own `Plane`. Its frame normal `n` must satisfy `n · a = 0` exactly in rationals, else `F` stays
  untagged: only then is a point's distance from the tag independent of its height along `a`.
- `e`, the largest published `Bound` of `F`'s boundary vertices, rounded up. The record builds that bound as a length
  from the face's section displacement, the level's, the placement's lift rounding and the walk-end allowance
  (`internal/breprecord/build.go`), so each true vertex lies within `e` of its held position.
- `E`, the extent bound: the largest exact distance from a boundary vertex of `F` to the tag plane, rounded up, plus
  `e`.
- `L`: the largest distance between two boundary vertices of `F` measured at right angles to `a`, in rationals,
  rounded down.
- `θ = min(2, √2·x)`, rounded up, with `x = 2E/(L − 2e)`: the distance between the two unit normals, which is the
  measure `NormalBound` states. `F` stays untagged unless `L > 4E` and every value is finite.
- The walked ends `p`, `q` (below) must lie more than `2e` apart at right angles to `a`, which tells the wall's rim
  from a side line whose ends differ only by rounding; else `F` stays untagged.
- Orientation: with `p → q` the wall's walked start and end in walk order, the tag's outward normal (`N`, negated when
  `Reversed`) must satisfy `N · (s·(q − p) × a) > 0` exactly, `s = −1` when the placement reflects and `+1`
  otherwise, which is the record's own outward rule (tangent × axis for a counter-clockwise outer walk and a clockwise
  hole walk). Else `F` stays untagged.

The group keeps `Surface` and `Retained`, takes `NormalBound = θ` (assembly makes it the face's `TagDeparture` and
`NormalBound`) and the new field `ExtentDelta = E`.

**Claim T.** Every point of `F`'s true patch lies within `E` of the tag plane, and the true wall's normal lies within
`θ` of the tag's.

*Proof.* The true patch is the strip swept along `a` over the true trace between two true points `p*`, `q*`, each
within `e` of a boundary vertex `p`, `q` of `F`. Because `n · a = 0`, a point's distance from the tag does not change
along `a`, and it is affine along the trace, so its largest value on the strip is at `p*` or `q*`, where it is at most
`dist(p, T) + e ≤ E`. Both planes contain the direction `a`, so their angle is the angle between the traces at right
angles to `a`, whose sine is at most `(d(p*) + d(q*)) / |p* − q*|_⊥ ≤ 2E/(L − 2e) = x`. The held and the true trace run
the same way: put the tag's trace on the x-axis, with `u = q − p` and `u* = q* − p*` measured at right angles to `a`.
Then `|u_y| ≤ 2(E − e)`, `|u*_y| ≤ 2E`, `|u*| ≥ L − 2e > 4E − 2e` and `|u − u*| ≤ 2e`, and opposite signs of `u_x`
and `u*_x` would need `12E² − 16E·e + 4e² ≤ 0`, which `E ≥ e` rules out except at `E = e`, where `L > 4E` does. Both
outward normals are their trace turned the same way about `a` (the true one by the record's rule, the tag's by the
orientation check), so `φ < π/2`. Two unit normals at
angle `φ ≤ π/2` lie `2·sin(φ/2) ≤ √2·sin φ` apart. ∎

`ExtentDelta` holds only over the face. A reader that needs the plane elsewhere refuses the face, which every plane
route already does because `TagDeparture ≠ 0` (§4).

### 2.3 Edges

A rim between the oblique wall and a cap plane publishes a `Line3` (retention §10.1 E2: two plane tags stand for two
true planes however far they lie off them; a `Line3` carries no parameters). A seam with a corner round is a plane and
a cylinder with a departed tag and keeps its chord chain (E12). So the wheel's window sides publish `Line3` rims on both
caps after the fillet, which a selector names by direction.

### 2.4 After the blend

A retained edge blend's result keeps the group as it keeps any plane group (`publishChargedWalls` reads curved groups
only). A mesh Boolean carries `ExtentDelta` with the tag (`SourceTag`): the true result patch is a piece of the true
operand face. A placement, the shifted retry, an exact rotation copy, `ScaledAt`, and the face copies of `Stitch`,
`Unstitch` and a patch chain drop `ExtentDelta` and keep the departed tag (the face copies keep its `NormalBound`), so the face stays refused by every plane route and W2 does not read it. Every list that clears a tag's
`AxialDelta` (assembly's loop-less faces, the blend's and the view's untagging, the displaced-wall and chorded-loft
readers) clears `ExtentDelta` with it. A snapshot key that compares a faceted group's or face's `AxialDelta` compares
`ExtentDelta` too.

## 3. The charged chain (W2)

Design text to be completed in W2's increment. Its basis, reviewed with W1:

- Scope: a closed chain (C1, retained-modify-reach §5.1) chamfered with equal or two setbacks. Open chains, fillet
  chains and arc ends keep refusing charged walls.
- Walls: a cylinder with a finite `AxialDelta` δ ≥ 0 (retained-edge-blend §6.1), a plane with `AxialDelta`, and a
  plane with `ExtentDelta` (§2, whose `TagDeparture` the chain then reads through `ExtentDelta` instead of refusing)
  are read with their charge; the junction's identity 2 (§5.2 of retained-modify-reach)
  stays a decision on the tags.
- **Claim J′.** With `δ` the largest wall charge, the true wall curve lies within `3δ` of the tag curve in plan,
  because at a junction the true junction point lies within `e_J = β(v) + |v − V|` of the tags' junction `V` (the held
  junction vertex `v` and its bound, already measured by the falsifier) and the two tags separate monotonically from
  `V`. A chamfer removes a sublevel set of `h + d` (depth below the base plus plan distance to the curve), so the
  true and built chamfers differ inside a shell of half-width `3δ + δ_P` about the built surface, `δ_P` the base
  plane's charge. Each group's area slack adds `w·e_J` per junction, `w` the group's width (`√(t_P² + t_W²)` for a
  strip). A crease the true curve may have at the junction adds about `t·e_J²/R²`, which the per-face charge absorbs.
- Each charged wall's δ joins the chain's per-face charge sum (`chain_rewrite.go`, `planeAmp·δ`), so every gate,
  SR6 and every bound carries it. Nothing admits on a residual.
- Reach: a chain edge must be `RetainedCurve`; beside an unmarked charged cylinder a rim curve declines once δ
  exceeds its chain's arithmetic allowance (retained-edge-blend §6.1), so W2 reaches the wheel's δ = 9.8e-14.

## 4. Readers of an oblique wall's tag

A published oblique wall's face states `TagDeparture = NormalBound ≥ θ` on every path, because nothing that keeps a
plane tag lowers `NormalBound`. `ExtentDelta` travels only where §2.4 carries it. Each reader below that receives a
faceted body charges the wall, refuses it, or is listed in `agent-docs/missing-features.md`.

| Reader | With an oblique wall |
|---|---|
| plane routes of `internal/retainedblend` (R1 line, corner and mitre through `retainedPlane`; R2 and R4 through `planeAndWall`; R3, R5, R6; tangent ends; chain base, walls, ends and bands; wall sine and slant; band provenance) | refuse `TagDeparture ≠ 0` |
| `facetedrecord.RetainedEdgeCurve` E2 | publishes a `Line3` between two planes; its falsifier slack adds `planeRimAmp·ExtentDelta` per plane as it adds `AxialDelta`, which only widens a reject-only test |
| `NormalAt`, `edgewedge` | charge `NormalBound` |
| retention §11 slab bound (`retainedPlaneOf`), `MirrorFace`, `ToFaceStop`, `trimgraph`, `retainedTag` and the partial loop chamfer, the faceted tangent chain, STEP's tag check | refuse `NormalBound ≠ 0` or `TagDeparture ≠ 0` today |
| `ToFaceAngularStop` (`internal/extent/angular_stop.go`) | refuses a plane that states `ExtentDelta` |
| `parts` joint inference (`parts/faces.go` `addPlane`) | reads a plane that states `ExtentDelta` as an unread face (`facedeparture.Charged`). After a placement drops `ExtentDelta`, it reads the wall as any departed plane, with `NormalAt`'s bound as its position bound (listed in `agent-docs/missing-features.md`) |
| `capplane.OfFace` | reads only a `CapRecord`, which an oblique wall never carries |
| `Hole`'s planar site, `resolvePlanarFace` (`Boss`, `Emboss`) | refuse a plane that states `ExtentDelta`. After a placement drops it, they read the tag's frame as the true plane, as they read any departed plane tag (listed in `agent-docs/missing-features.md`) |
| selectors (`Planar`, `NormalTo`, `Facing`), `planarDihedral`, `bossRootEdges`, `embossRimEdges`, `boundaryaudit` | read the tag normal to select edges or faces, or for a message, within their own tolerance; a later blend refuses the wall by its own gate |
| `Face.Surface()` | shows the tag with no exactness flag (api-design §5) |

Readers gated by payload class (contact geometry, clearance, `mirrorjoin`, `capplane` through a prism record,
`Section`'s faceted arm, verify) never read a faceted plane tag.

## 5. The refused top-face run

On the top face, once a caller leaves the root-fillet edges out of the selection, each window is a run of seven edges
whose ends lie where a corner round's rim meets the fillet. There the round carries the fillet's cap above the face,
and the cap edge continues the rim tangentially up to where the cap pinches (boss-root-overflow §2.5, §5.3). So the run
neither closes nor ends at a plane corner. No chain class reads it, and what Fusion does there is not established.
Building it needs a Fusion gate like boss-root-overflow G0, recorded with and without tangent-chain selection, and a
reading of the fillet face from its blend construction. Its refusal is stated in W2.

## 6. Increments

Each increment is one PR, merged after green CI and an independent review of its soundness-sensitive diff.

| # | Increment | Unlocks | Estimate |
|---|---|---|---|
| W1 | §2: the view tags oblique walls with `TagDeparture` and `ExtentDelta`; `SourceTag` carries it; placement and scale drop it | `Line3` window side rims after a blend; no new blend | built |
| W2 | §3: a closed chamfer chain reads charged walls | the wheel's bottom-face window chamfers | 1.5 to 2.5 weeks |

The options weighed before this design: building §5 as well (3 to 6 weeks more, gated on Fusion), or moving the
windows' inner radius to 7.6 in the gallery (no decad work). The user kept the radius at 7 and chose W1 and W2.

## 7. Rejected shortcuts

- Tagging an oblique wall with `AxialDelta = E`: it claims a parallel offset everywhere, which a tilted tag does not
  meet past the face.
- Admitting a junction of charged walls by a residual on the true walls' tangency: a small residual proves nothing.
  Claim J′ needs no true tangency.
- Chamfering the windows before the boss: the boss then refuses ("the boss's root circle is not an edge of the
  result").
