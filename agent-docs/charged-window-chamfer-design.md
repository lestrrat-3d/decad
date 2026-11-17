# Charged window chamfer design

`Chamfer` of a window rim of a holed A1 stacked BRep after a retained edge blend: the gallery balance wheel's windows
after its Ø14 `Boss` root fillet. Status: approved for building in two increments (§6). W1 and W2 are built.
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
operand face. A placement, the shifted retry, an exact rotation copy and `ScaledAt` drop `ExtentDelta` and keep the
departed tag. The face copies of `Stitch`, `Unstitch` and a patch chain keep the plane tag and its `NormalBound` and
drop `RetainedTag`, `TagDeparture` and `ExtentDelta`. Either way the face stays refused by every plane route and W2
does not read it. Every list that clears a tag's
`AxialDelta` (assembly's loop-less faces, the blend's and the view's untagging, the displaced-wall and chorded-loft
readers) clears `ExtentDelta` with it. A snapshot key that compares a faceted group's or face's `AxialDelta` compares
`ExtentDelta` too.

## 3. The charged chain (W2)

### 3.1 Scope

A closed chain (C1, retained-modify-reach §5.1) chamfered with equal or two setbacks reads charged walls and a charged
base. Open chains, fillet chains and arc ends keep refusing charged walls, by name (§3.5).

| Face | Read as | Charge `δ_k` |
|---|---|---|
| a wall `Cylinder` with a finite `AxialDelta ≥ 0` (retained-edge-blend §6.1) | its tag | `AxialDelta` |
| a wall `Plane` with a finite `AxialDelta ≥ 0` | its tag | `AxialDelta` |
| a wall `Plane` with `TagDeparture ≠ 0` and a finite `ExtentDelta > 0` (§2) | its tag | `ExtentDelta` |
| the base `Plane` with a finite `AxialDelta ≥ 0` and no departure | its tag | `δ_P = AxialDelta` |

The junction's identity 2 (retained-modify-reach §5.2) stays a decision on the tags, exact or within
`tangentRoundoff`. A wall that states its charge is never read as exact: the chain reads it only on this path, and
every bound below carries the charge.

### 3.2 Claim J′

Write `h(x)` for the depth of `x` below `P` on the chain's side and `d(x)` for the distance from `x`, at right angles
to the walls, to the chain's wall curve `Γ` (the walls' trace in `P`). Away from other faces, which the gates of
retained-modify-reach §5.6 keep out, an equal-setback chamfer of size `t` removes `{x : h(x) + d(x) < t}` near `Γ` on
the material side of a convex chain, and fills the same set on the void side of a concave one. A two-distance chamfer
removes `{h/t_W + d/t_P < 1}`. The construction builds that set's boundary from the tags (Claim J, §5.4: the junction
rows lie on both tag blends).

**Claim J′.** Let `δ = max δ_k` over the chain's walls and `δ_P` the base's charge. The true chamfered solid differs
from the one built on the tags only inside a shell about the built chamfer surface whose half-width along its normal
is at most `3δ + δ_P`.

*Proof.*

- Each true wall trace lies within `δ_k ≤ δ` of its tag trace over the edge: by retained-edge-blend §6.1 for a
  charged cylinder, by the parallel offset for a plane's `AxialDelta`, and by Claim T over the face for an oblique
  wall.
- At a junction the tags are tangent at `V`. The record's true traces meet at a recorded point `V_r` within `δ` of
  both tag traces. The held junction vertex `v` lies within its bound `β(v)` of `V_r`, and the falsifier measures
  `|v − V|`, so `e_J = β(v) + |v − V|` bounds `|V − V_r|`. It is a bound term only.
- `Γ` switches tags at `V` and `Γ*` switches true traces at `V_r`, so between them a point of `Γ*` follows the true
  trace of one wall while `Γ` follows the other wall's tag. The two tags separate monotonically from `V` (each is
  straight or of one curvature on that span, and they are tangent at `V`), and at `V_r` they lie within `2δ` of each
  other, since `V_r` lies within `δ` of both. So on the span the two tags stay within `2δ`, and a point of `Γ*`,
  within `δ` of its own tag, lies within `3δ` of `Γ`. Elsewhere it lies within `δ`. The same holds the other way:
  a point of a tag lies within `δ` of its true trace, for an oblique wall because the distance to the true plane is
  affine along the true segment and at most `E` at its ends (Claim T). So the Hausdorff distance is at most `3δ`. No
  tangency of the true walls is needed.
- Every wall, true or tag, contains the sweep axis `a` (a cylinder's axis, a plane's `n·a = 0` gate, the record's
  sweep), so a point's distance from a wall is its plan distance from the wall's trace, and `d` is a plan distance.
- So `|d* − d| ≤ 3δ` everywhere near the chain and `|h* − h| ≤ δ_P`. The removed sets differ only where
  `|h + d − t| ≤ 3δ + δ_P` (two setbacks: `|h/t_W + d/t_P − 1| ≤ (3δ + δ_P)·max(1/t_W, 1/t_P)`), which lies within
  `3δ + δ_P` of the built surface along its normal, because the gradient of `h + d` (scaled to unit setbacks) has
  length at least 1. ∎

**The crease.** The true curve may turn by an angle `ψ` at `V_r`: the two true traces each lie within `δ` of a tag,
the tags are tangent at `V` to `tangentRoundoff`, and the true tangents at `V_r` differ by at most
`ψ ≤ ψ_max = Σ_k ψ_k + ψ_0`, rounded up, summed over the junction's two walls: `ψ_k = (e_J + 2δ)/R_k` for a
cylinder of tag radius `R_k` (its tag tangent turns by `(e_J + δ)/R_k` between `V` and `V_r`'s foot, and the true
wall's shift adds `δ/R_k`), `ψ_k = θ_k` for an oblique plane (its `TagDeparture`), `ψ_k = 0` for a displaced plane;
`ψ_0 = √(4g/R_min)` bounds the turn at which two walls whose tags are off by the `tangentRoundoff` gap `g` meet,
`√(2g(1/R_1 + 1/R_2))` for two cylinders (a line counts as `1/R = 0`). `ψ_k` reads the chord `e_J + 2δ` as an angle;
for `e_J + 2δ ≪ R_k` the arc exceeds it by a factor near 1, inside the factor 2 between `c_J` and the corner's
leading term `t·ψ²/8`. A kernel mitres such a corner where the
sublevel set rounds it; the two differ by at most `c_J = t·(1/cos(ψ_max/2) − 1) ≤ t·ψ_max²/4` (for `ψ_max ≤ 1`), on the
reflex side within the junction's slab. `σ = 3δ + δ_P + max_J c_J` is the shell half-width that every bound below uses.

### 3.3 Bounds

- **Slack.** Each charged wall's `δ_k` joins the chain's per-face charge sum (`rewriteChain`, `planeAmp·δ` per face,
  `planeAmp = 4/minSine ≥ 4`), cylinders and extent planes included, and the chain adds `max_J c_J`. The sum holds
  every charged face, so it is at least `planeAmp·(δ + δ_P) + max_J c_J ≥ σ`. Every gate of retained-modify-reach
  §5.6, SR6 and every strip and cut vertex's bound then carry it. Nothing admits on a residual.
- **Junction rows.** Each junction row's points, `P_J` and `S_J` included, take `τ + slack + e_J`: the true section
  lies at `V_r`, up to `e_J` along the curve from the tags' `V`.
- **Sizes.** On a charged chain, an arc whose base lies inside its circle needs `R − t_P > slack`, the form of
  §5.6's size gate, since the true radius may be `R − δ`; the sublevel model needs the setback inside it.
- **Strips beside an oblique wall.** A line strip whose wall states `ExtentDelta` is not published as a tag: its true
  plane tilts with the wall, which no plane `AxialDelta` states. It stays `Faceted`, with its area shortfall charged.
  A strip beside a charged cylinder stays `Faceted` as retained-edge-blend §6.1 states; a strip beside a displaced
  plane keeps today's `AxialDelta` tag.
- **Area.** Each chain edge's blend group, and each wall and base group the rewrite changes, adds to `AreaSlack`
  `2·P_g·σ + (π + Φ_g)·σ² + A_g·σ/r_g + w_g·Σ_J e_J`, with `σ` as §3.2 defines it, rounded up, `P_g` an upper bound on the
  piece's perimeter from its exact curves, `Φ_g` its total turning, `A_g` its exact area, `r_g` its smallest radius of
  curvature (no term for a plane), the sum over the junctions the piece meets, and `w_g` the piece's width there:
  `√(t_P² + t_W²)` for a strip, the setback for a wall or base band, times `max(1, ρ_max/r_min)`, `ρ_max` the
  largest distance of any of the chain's arc pieces from its axis and `r_min` the chain's smallest arc radius (the
  slice between the planes through `V` and `V_r` widens away from an arc's axis; the chain-wide ratio bounds every
  piece's own). It is the form of boss-root-overflow §5.5.6's arc charge with the junction's shift
  added.
- **Volume.** Claim K (retained-edge-blend §6.3) with every strip vertex's bound raised by the slack.

### 3.4 A selection with chains and other edges

The wheel's one `Chamfer` names its window loops and its rim circles together. A chamfer selection that holds closed
or open chains and other edges builds when no other edge shares a vertex with a chain edge: the other edges take their
own routes first, one at a time on the evolving mesh as today, each keeping its ordinal in the selection for its blend
role, and every chain, the first included, is then found again on the result by its edges' end points and built. The
rim's base clearance keeps a window loop out of a rim's annulus, and the chain's base band then tests the rim's new
setback circle. An open chain reads its end faces on that result, so an end face another edge's blend rewrote is read
as a later call would read it (a ramp whose top edge is chamfered in the same call, retained-modify-reach §5.5 step 5).
An edge that touches a chain, and any fillet selection that mixes them, keeps today's refusal ("a selection that holds
a tangent chain names only chain edges").

### 3.5 Refusals

| Selection | Error |
|---|---|
| an open chain with a charged wall or base, including a top-face window run that meets a root fillet (§5) | `ErrUnsupported`: "an open tangent chain on walls that state a displacement is not built" |
| a fillet chain with a charged wall or base | `ErrUnsupported`: "a tangent chain fillet on walls that state a displacement is not built" |
| a charge that is negative or not finite, or an oblique wall without `ExtentDelta` | the wall is not read; today's texts |

Junctions are read in the charged mode for both a chamfer and a fillet, so a fillet or an open chain reaches its own
refusal text. `WithTangentChain` expansion (`Chainable`, `Tangent`) stays strict: it reads no charged face, and a seed
on one stays SX9.

### 3.6 Reach

A chain edge must be `RetainedCurve`. Beside a charged cylinder the rim curve declines once δ exceeds its chain's
arithmetic allowance (retained-edge-blend §6.1), so W2 reaches charges inside that allowance, as the wheel's
(δ = 9.8e-14) is.

## 4. Readers of an oblique wall's tag

A published oblique wall's face states `TagDeparture = NormalBound ≥ θ` on every path, because nothing that keeps a
plane tag lowers `NormalBound`. `ExtentDelta` travels only where §2.4 carries it. A mesh Boolean also mints the same
tag from a route-L chamfer band's flat patch that states a tilt (faceted-source-retention §3.2, A2, Claim F), and every
reader below reads that face as it reads an oblique wall. Each reader below that receives a
faceted body charges the wall, refuses it, or is listed in `agent-docs/missing-features.md`.

| Reader | With an oblique wall |
|---|---|
| plane routes of `internal/retainedblend` (R1's faces `A` and `B`, corner and mitre through `retainedPlane`; R2 and R4 through `planeAndWall`; R3, R5, R6; tangent ends; open chains' ends; wall sine and slant; band provenance) | refuse `TagDeparture ≠ 0` |
| R1's end plane `G` (retained-edge-blend §6.1, "An oblique end plane") | reads the plane with `ExtentDelta` as its charge and `TagDeparture` as its tilt, when the tilt is at most `0.5` and `G` gives up the cap |
| a closed chamfer chain's walls (§3) | read the wall with `ExtentDelta` as its charge |
| `facetedrecord.RetainedEdgeCurve` E2 | publishes a `Line3` between two planes; its falsifier slack adds `planeRimAmp·ExtentDelta` per plane as it adds `AxialDelta`, which only widens a reject-only test |
| `NormalAt`, `edgewedge` | charge `NormalBound` |
| retention §11 slab bound (`retainedPlaneOf`), `MirrorFace`, `ToFaceStop`, `trimgraph`, `retainedTag` and the partial loop chamfer, `WithTangentChain` expansion (`Chainable`), STEP's tag check | refuse `NormalBound ≠ 0` or `TagDeparture ≠ 0` |
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
reading of the fillet face from its blend construction. It refuses as an open chain on charged walls (§3.5).

## 6. Increments

Each increment is one PR, merged after green CI and an independent review of its soundness-sensitive diff.

| # | Increment | Unlocks | Estimate |
|---|---|---|---|
| W1 | §2: the view tags oblique walls with `TagDeparture` and `ExtentDelta`; `SourceTag` carries it; placement and scale drop it | `Line3` window side rims after a blend; no new blend | built |
| W2 | §3: a closed chamfer chain reads charged walls, beside other edges of one selection | the wheel's bottom-face window chamfers | built |

The options weighed before this design: building §5 as well (3 to 6 weeks more, gated on Fusion), or moving the
windows' inner radius to 7.6 in the gallery (no decad work). The user kept the radius at 7 and chose W1 and W2.

## 7. Rejected shortcuts

- Tagging an oblique wall with `AxialDelta = E`: it claims a parallel offset everywhere, which a tilted tag does not
  meet past the face.
- Admitting a junction of charged walls by a residual on the true walls' tangency: a small residual proves nothing.
  Claim J′ needs no true tangency.
- Chamfering the windows before the boss: the boss then refuses ("the boss's root circle is not an edge of the
  result").
