# Modify Design

How the v1 evaluator builds the three modify ops — `Body.Fillet`,
`Body.Chamfer`, `Body.Shell`: what a modify op owes and how it refuses (§1),
the reduction that makes a straight prism's modify ops exact (§2), the class it
admits and the class it rejects at the call (§3), where the 2D work is allowed
to live (§4), the fillet (§5), the chamfer (§6), the shell (§7), how the
results stay Exact (§8), the recipe, the provenance and the replay (§9), what
the result is to everything already shipped (§10), and the increment plan
(§11). Companion to `docs/api-design.md`, which owns the three signatures, the
selector contract and the retire rule ("core §N"), and to
`docs/evaluator-design.md`, whose §11 row 5 this document is ("evaluator §N").
`docs/verification-design.md` owns how the results are judged
("verification §N"). Nothing here changes those contracts.

## 1. What a modify op owes, and how it refuses

A modify op consumes one body and returns another (core §6). It has exactly
two honest outcomes, and the whole of this document is the line between them:

- **the body the caller asked for, built exactly** — analytic faces, Exact
  measurements, a boundary valid by construction; or
- **an error at the call**, naming which of the two things went wrong.

There is no third outcome. A blend clipped back because the radius did not fit,
an offset whose self-overlap was quietly trimmed away, a corner patched with a
surface nobody can name — each is a body the caller did not ask for, wearing
the Exact measurements of one they did. That is core §1's confidently-wrong
failure, produced by the very operation an agent reaches for last, on the model
it is about to commit to.

**The sentinel follows the intent, and the test is whether the body exists.**
Evaluator §2 stages what the evaluator cannot build; core §12 rejects what
cannot be built at all. The two sentinels are not interchangeable, and one
question separates them:

| The caller asked for | Sentinel |
|---|---|
| a body that **does not exist** — a fillet of a radius for which no blend surface exists at all (the two faces' material-side offsets never meet), a shell thicker than the material it hollows | `ErrDegenerate` |
| a body that **exists** and this evaluator cannot build — a blend across a cap edge, a blend whose cutback overruns the wall it is trimmed from, a shell that opens a side wall, an offset whose exact form changes the section's topology | `ErrUnsupported` |

Both are returned from the call, before anything is recorded: a failed
evaluation leaves the recipe and the document untouched (evaluator §8), so a
refused modify op retires nothing and the caller still holds a live body.
Neither is ever deferred into a `Verify` reading — a body the evaluator declines
to build is not a body whose soundness is in question, it is a body that does
not exist. The staging split of evaluator §11 is exactly this: an intent the
evaluator cannot BUILD is an error at the call; only a question it cannot
ANSWER reads `Suspect`.

## 2. The reduction: a prism's modify ops are its section's

A straight prism is a 2D region swept along a straight line. Every face, edge
and vertex of it is the sweep of a feature of the recorded section
(evaluator §5), and the correspondence is total:

| In the section | In the prism |
|---|---|
| a boundary segment | a side face (`Plane` for a `LineSeg`, `Cylinder` for a circular one) |
| a corner between two segments | a **lateral edge** — a `Line3` parallel to the sweep, running cap to cap |
| the region itself | the two cap faces |

**So a modify op that touches only lateral edges and caps is a rewrite of the
section, and the modified body is the prism over the rewritten section.** A
fillet of a lateral edge is the section's corner rounded by an arc of radius
`r`; a chamfer of one is the corner cut off by a chord; an inward shell is the
section eroded by the thickness. Each rewrite maps a line/arc region to a
line/arc region — the offset of a line is a line, of a circle a concentric
circle, and a corner closes with an arc or a miter — so the result is again a
`ProfileRecord` in the increment-1 vocabulary, and the body is again built by
`evalPrism` from it.

That is the whole design, and everything else follows from it:

- **exactness is inherited, not re-argued.** The mass-property engine
  (evaluator §4) integrates the rewritten region in closed form, so volume,
  area and centroid stay Exact with a zero bound (§8);
- **validity is by construction, exactly as evaluator §10 claims it.** A prism
  over a simple closed line/arc region is watertight and manifold structurally.
  What changes is who proves the region simple: for an extrude it is `sketch`;
  for a modify op it is decad's own audit of its own construction (§4), run
  before anything is built;
- **everything downstream already works.** The tessellator, the analytic
  surveys, the clearance kernel, the stops and `Placed` all consume the
  evaluator's own prism payload, and the result of a fillet or a chamfer IS one
  (§10);
- **the ops compose.** A filleted prism is a prism, so a chamfer of it builds,
  and a fillet of a chamfered prism builds.

**The reduction is the evaluator's, never the recipe's.** The `Step` records
"fillet these edges at 2 mm" — a selector and a radius (core §6.2) — and never
the rewritten section. An exact kernel replays the same step and builds the
blend as a true rolling-ball surface against the true faces; it produces the
same body by a different route, which is precisely what core §2 promises a
second evaluator will do. A recipe that recorded the rewritten section would
have recorded this evaluator's *method* as the caller's *intent*, and vN would
inherit a 2D reduction it does not need.

## 3. The admitted class

**Fillet and chamfer.** The receiver must be a body this evaluator built as a
prism, and every selected edge must be a **lateral** edge of it:

| Input | Reads |
|---|---|
| a prism body; every selected edge lateral; every corner a genuine corner | **builds** |
| a selected edge that is a cap edge (a `Line3`, `Arc3` or `Circle3` in a cap plane) | `ErrUnsupported` — the corner problem (§5) |
| a receiver that is not a prism — a revolve, a shelled prism, a `Faceted` boolean output, an imported body | `ErrUnsupported` |
| a corner whose two segments meet **smoothly** (tangent) or in a **cusp** (anti-tangent) | `ErrDegenerate` — there is no corner to blend |
| a section carrying a free-form segment kind | `ErrUnsupported` — it does not build as an extrude either (evaluator §5) |
| a radius or distance for which **no blend surface exists** — the corner's two material-side offsets never meet, or an offset arc's radius has reached zero | `ErrDegenerate` (§5, §6) |
| a cutback that reaches or passes the far end of a walk — including a far end another corner in the same call claims — or any rewritten loop that crosses itself or another loop | `ErrUnsupported` — the surface exists; reaching the body needs the blends merged, or trimmed against the geometry they run into, and this evaluator's per-corner rewrite has no resolution step (§4, §5) |
| a rewritten loop that has turned itself inside out — its signed area has changed sign | `ErrDegenerate` (§4) |

A full-circle loop is a single closed wall with **no** lateral edge at all
(evaluator §5 emits no seam), so a cylinder has no edge a fillet can name but
its cap rims — and those are `ErrUnsupported`. That is the honest reading of the
class, not an oversight: the blend of a cap edge is the vertex-blend problem
(§5), and this increment does not solve it.

**Shell.** The receiver must be a prism, and the selector must resolve to cap
faces only:

| Input | Reads |
|---|---|
| a prism; the removed faces are one or both caps | **builds** (§7) |
| a removed face that is a side wall | `ErrUnsupported` — the cavity is then the offset of an open chain, closed against the removed wall's own surface; a different 2D machine |
| a receiver that is not a prism | `ErrUnsupported` |
| an **inward** thickness at or beyond the section's inradius | `ErrDegenerate` — the erosion is empty; there is no such body (an outward thickness has no such limit, §7) |
| an offset whose exact form merges, splits or drops a boundary loop | `ErrUnsupported` — the body exists; this evaluator's offset is topology-preserving (§7) |

**Magnitudes are magnitudes.** The radius, the distance and the thickness are
`units.Value` of Kind Length: another `Kind` is `ErrUnitKind`, a negative value
`ErrNegativeMagnitude` (core §12 names all three by role), a non-finite one
`ErrNotFinite`, and **zero is `ErrDegenerate`** — a zero-radius fillet, a
zero-distance chamfer and a zero-thickness shell each name a body identical to
the one the caller already holds, which is a question with one answer and no
content, exactly as `Verify`'s zero tool is (verification §2).

**A selector that matches nothing is an error, loudly** (core §9): a query
carrying no cardinality assertion that resolves to zero entities is
`ErrNoMatch`, and a failed `Exactly(n)` / `AtLeast(n)` is `ErrCardinality`,
zero matches included (core §12). Selection happens against the live receiver,
before the build; a retired receiver is `ErrRetiredBody`, and every gate above
runs before a single face is made.

## 4. Where the 2D work lives, and what proves it

The section rewrite computes tangent points, offset curves and their crossings —
2D geometry, in a repository whose first hard rule is **never re-derive a 2D
answer**. The line is the one evaluator §4 already draws for the mass
properties, and it is drawn by *whose claim it is*:

- **`sketch` decides the sketch**: what closes, what is valid, where the
  caller's entities cross, what a trim is. decad consumes those answers and
  never recomputes one. That is unchanged and untouched here.
- **decad owns its own records.** A `ProfileRecord` is decad's own geometry
  (seam §2), and the rounded corner, the bevel chord and the offset wall exist
  in **no sketch at all** — no entity was ever drawn there, so there is no
  upstream answer to consume, and reaching back for one is impossible in any
  case: the evaluator evaluates from the record, and a replay holds no sketch
  (evaluator §1). The rewrite is new geometry decad synthesizes from decad's
  own data, in the same class as the boundary integrals of evaluator §4 and the
  inscribed-disk kernel of `survey2d.go`.

**The falsify-only rule is not in tension with the audit, because there is no
upstream claim to bless.** That rule governs admission of what `sketch` hands
over — a `Partial` fragment's `TExact`, which decad may disprove and may never
certify (seam design). The rewritten section is not handed over by anyone; decad
constructed it, so decad owns its validity, and it proves it with an **exact,
closed-form** test rather than a residual:

> **The rewrite is admitted only when the loops it produces are proven simple.**
> Every pair of segments in a rewritten loop, and every pair drawn from two
> loops of the section, is tested for crossing in closed form — line×line,
> line×circle, circle×circle over decad's own segments, the same closed forms
> the clearance kernel's 2D reduction and `survey2d.go`'s boundary walks use.
> A crossing, a loop whose signed area has changed sign (it has turned itself
> inside out), a segment trimmed past its own other end, or an offset arc whose
> radius has reached zero, **rejects the call**.

The audit's verdict is one of the two sentinels of §1, by the same test: a loop
that has turned inside out, and an offset arc whose radius has vanished, say the
modification consumed the region — there is no such body, `ErrDegenerate`. A
crossing, and a segment trimmed past its own other end, say the pieces the
rewrite produced must be resolved against each other — merged, or trimmed
against the geometry they run into — before they bound anything. That body
exists and a resolving kernel builds it; this evaluator's rewrite trims each
corner on its own and has no resolution step, so it is `ErrUnsupported`. Either
way the call is refused before a face is made.

A residual proves nothing and admits nothing; a crossing test on exactly
represented line and arc segments is a **decided** fact of decad's own data, and
its verdict is the same under every evaluator. It is what makes the build's
"valid by construction" claim (evaluator §10) survive a modify op: the region is
proven simple before the prism over it is built, so no unproven body is ever
made, and `Verify` reads the result exactly as it reads an extrude's — Exact,
and structurally sound.

## 5. Fillet

```go
func (b *Body) Fillet(sel EdgeSelector, r units.Value, opts ...FilletOption) (*Body, error)
```

The rolling-ball blend of a lateral edge is a **cylinder**: the ball of radius
`r` rolling in the corner sweeps its center along a straight line parallel to
the sweep direction, and the surface it envelops is the cylinder of radius `r`
about that line, trimmed to the sweep interval. In the section, that cylinder is
one arc.

**The blend center is pinned by the material, not by the edge's convexity flag.**
Let a corner join walk `A` (arriving) to walk `B` (leaving) at the corner point.
The center is the point at distance `r` from both carriers, **on the material
side of each**, and each carrier's material-side offset is a curve of the same
kind:

| Carrier | Its `r`-offset into the material |
|---|---|
| a line | the parallel line, `r` into the material |
| a circle of radius `R`, material inside it | the concentric circle of radius `R − r` |
| a circle of radius `R`, material outside it | the concentric circle of radius `R + r` |

Which side the material is on is a **decided** fact of the record — the loop's
winding and the segment's own sense — never a guess. The center is the
intersection of the two offset carriers nearest the corner; the tangent points
are its feet on the two carriers. Line×line, line×circle and circle×circle are
closed form with at most two roots, and the root is chosen by the corner it
belongs to, not by proximity to a sample. **No intersection means no blend of
that radius exists** — two parallel offsets, or two concentric ones — and the
call is `ErrDegenerate`.

`Edge.IsConvex` is therefore not an input to the construction at all; it is what
the caller *selected* with (`Convex()` / `Concave()`, core §9). A convex corner
and a concave corner take the same construction, and differ only in which way
the arc winds — the convex corner rounds material off, the concave one fills
material in, and both are exact.

**The cutback gate refuses; it never clips.** The blend consumes an arc length
`aA` of `A` and `aB` of `B`, measured along each carrier from the corner. Each
must fit **strictly** inside the length that walk still has, after the
modification at its *other* end has taken its own cutback — two adjacent
corners of a short wall are filleted in the same call, and they claim the wall
from both ends. A cutback that reaches or passes the far end is
`ErrUnsupported`: the blend cylinder itself is unharmed — it is pinned by its own
corner's offsets, and the wall's *length* never entered its equations — so the
body the caller named exists, and a kernel that merges the two blends into one
surface (or trims the blend against the wall it has run into) builds it. This
evaluator trims each corner on its own, so it declines, by §1's test. What it
will not do is clip the blend, slide it, or shrink `r` to fit: those are all the
same failure — a body the caller did not ask for, with a radius they did not
name. `ErrDegenerate` is kept for the case where there is nothing to build at
all: no center at that radius, so no blend surface exists.

**The rewrite.** `A` is trimmed to its tangent foot, `B` is trimmed to its own,
and an `ArcSeg` of radius `r` about the center joins them, wound so the loop
keeps its orientation. The result goes through the §4 audit and then through
`evalPrism`, which produces, with no special case:

| Entity | What it is |
|---|---|
| the blend face | `Cylinder{Origin: the center lifted to the section plane, Axis: the sweep direction, Radius: r}`, trimmed to the sweep interval |
| its two tangent edges | `Line3`, each shared with a trimmed side face — a **smooth** junction: the surfaces meet tangentially |
| its two cap edges | `Arc3`, one per cap, each shared with the cap face |
| the old lateral edge | gone; the old corner vertices are replaced by the arcs' endpoints |

**The build table needs one correction, and the fillet depends on it — it does
not introduce it.** A circular wall's face orientation — which way its outward
normal points — must be read from the **walk's own sense**: a circular walk that
runs clockwise in the plane frame has its material *outside* the cylinder, so
the face is reversed; counter-clockwise, inside, and it is not. The prism build
reads the loop's *role* instead (an outer loop's wall not reversed, a hole's
reversed), which is a proxy, and the proxy is wrong wherever an **outer loop
carries a clockwise circular walk**. That is not a shape only a fillet can make:
the seam records an arc's walk sense in the segment's own range (`TStart` >
`TEnd` says the walk runs against the curve, `seam.go`), so a plain sketch
produces one — a plate with a semicircular bite taken out of one edge walks that
arc clockwise on its outer loop, and the role rule gives its wall a normal
pointing into the material. It is a live defect in the prism build, fixed there,
in its own change. Increment 5 needs the walk-sense rule because a concave round
is the first thing a fillet emits; it is a prerequisite of this increment, not a
deliverable of it.

**The corner problem is excluded, not fudged.** Where two blends meet at a
shared vertex — a lateral edge's blend running into a cap edge's blend — the
rolling ball has no single surface to sweep: the two cylinders meet in a curve
that is not on either, and the patch that closes the corner (a vertex blend, a
setback) is neither a cylinder nor a sphere in general. It is not in the shipped
surface set, it is not exactly derivable from the record, and an evaluator that
invented one would be guessing at the very corner an agent asked it to check.
So the class is drawn where the problem does not arise: **a lateral edge's blend
terminates on the two caps, and two lateral blends never share a vertex** —
distinct lateral edges are disjoint, and each blend runs cap to cap, so there is
no vertex at which two of them meet and no patch that would close one.

Two lateral blends can still **interfere**, and interference is refused, never
patched. Two corners of one wall claim it from both ends, and that is the cutback
gate above. Two corners that share no wall at all — opposite ends of a thin neck,
two corners of one loop that are not adjacent, a corner of the outer loop and a
corner of a hole across a thin section — can have their rewritten pieces cross
without either overrunning a walk, and those are caught by §4's audit, which
tests every pair of segments **within** a rewritten loop and **across** every two
loops of the section. Both refusals name `ErrUnsupported`, both fire before a face
is made, and neither produces a corner needing a surface nobody can name. A
selector that names a cap edge is `ErrUnsupported` at the call (§3), and the
vertex blend stays where it belongs: an open question (§12), not a surface this
evaluator makes up.

`FilletOpts` carries nothing in this increment, so a fillet `Step`'s `Opts` is
nil (core §6.2: nil when the op takes none). A variable-radius or setback option
lands in it, with the struct, when it ships.

## 6. Chamfer

```go
func (b *Body) Chamfer(sel EdgeSelector, d units.Value, opts ...ChamferOption) (*Body, error)
```

The same edge class, the same gates, one simpler surface. **The bevel of a
lateral edge is a plane**: in the section, the corner is cut off by a straight
chord, and the chord swept along the sweep direction is a planar face.

**The setback is measured along the adjacent boundary curve**, `d` from the
corner along each — a length along a `LineSeg`, an **arc length** along a
circular one. On a straight prism that is exactly the geodesic setback from the
edge across the adjacent face, because the boundary curve *is* that face's
cross-section: the two readings coincide, so the definition is unambiguous
rather than merely convenient. Equal distance both ways is the whole of v1: an
asymmetric chamfer — two distances, or a distance and an angle — is an option
that has not shipped, so it is not an option a caller can pass, and nothing is
silently narrowed (core §8.1: an option that cannot be recorded does not ship).

The rewrite trims both walks back by `d` and joins the feet with a `LineSeg`.
The gates are the fillet's, unchanged in every respect: a setback that reaches
or passes the far end of a walk — including a far end another corner in the same
call has already claimed — is `ErrUnsupported`, refused and never clipped; a
smooth or cusped corner is `ErrDegenerate`; a rewritten loop that crosses itself
or another loop is `ErrUnsupported`, and one that has turned inside out is
`ErrDegenerate` (§4). A corner with a circular neighbour builds: the chord
from a point on a line to a point on an arc, or from arc to arc, is still a
`LineSeg`, and the bevel face is still a `Plane` — a chamfer against a
cylindrical wall meets it in a straight ruling, because both are parallel to the
sweep.

A convex corner's chamfer cuts material away; a concave corner's fills material
in. Both build, from the same construction, for the same reason the fillet's
two cases do.

`ChamferOpts` carries nothing this increment, so a chamfer `Step`'s `Opts` is
nil.

## 7. Shell

```go
func (b *Body) Shell(sel FaceSelector, thickness units.Value, opts ...ShellOption) (*Body, error)
```

`sel` names the faces to **remove** — the openings. What remains of the solid is
a wall of the given thickness behind every face that was *kept*. On a prism, with
the removed faces restricted to caps (§3), the whole construction is the
section's own offset.

**Inward is the default sense, and outward is enumerated, never signed.** The
thickness is a magnitude, so it carries no sign (core §8.1's rule, applied
here): the sense is a `ShellSense` — `Inward` (the wall grows into the original
solid; the outer skin does not move) or `Outward` (the wall grows off it; the
original solid becomes the cavity) — set by `WithShellSense`, recorded in
`ShellOpts`, and defaulting to `Inward`, which is what "shell this box" means
everywhere it is said. `ShellOpts` is the one `StepOpts` variant this increment
fills, and its `Sense` encodes as a named text token, exactly as `Direction`
does.

**The section offset is exact, and it is closed in the recorded vocabulary.**
Write `P` for the section. The inward offset (the erosion) `P ⊖ t` is bounded by:

| Feature of `P` | Its offset |
|---|---|
| a line segment | the parallel line, `t` into the material |
| a circular segment, material inside | the concentric circle of radius `R − t` |
| a circular segment, material outside — a hole wall, a concave round | the concentric circle of radius `R + t` |
| a **convex** corner | a miter: the two offset curves meet, and the corner stays sharp |
| a **reflex** corner | an **arc of radius `t` centered on the corner point** — the nearest boundary feature there is the corner itself, so the erosion's boundary is at distance exactly `t` from it |

Every piece is a line or an arc, so `P ⊖ t` is a `ProfileRecord`. The outward
offset is the same table with the two corner rules exchanged (a convex corner
rounds, a reflex one miters) and the radii moved the other way. Both are exact,
and the class that admits them is exactly the increment-1 segment set: a
free-form section has no offset in the vocabulary (the offset of a spline is not
a spline), and it does not build as an extrude either.

**Two gates, and they are different questions.**

- **Does the body exist?** This is the inward sense's question, and the erosion
  answers it: `P ⊖ t` is non-empty exactly when `t` is strictly less than the
  section's **inradius** — the radius of its largest inscribed disk, which
  `survey2d.go` already computes **exactly** as part of the wall survey. At or
  beyond it the wall has eaten the part and there is no cavity: `ErrDegenerate`,
  and the reading that refuses is the same one that answers `MinWallThickness`.
  An outward shell has no such limit — a dilation of a non-empty region is never
  empty, at any thickness.
- **Can this evaluator build it?** The raw offset is admitted only when it
  **preserves the section's topology**: every offset loop simple, no two offset
  loops crossing, the offset outer loop still containing each offset hole, and the
  holes still mutually disjoint — all decided by the §4 crossing audit in
  closed form. Inward, a section with a slot narrower than `2t`, or two holes
  closer than `2t`, has an exact offset whose *pieces merge*; outward, a hole
  narrower than `2t` closes up and the loop is *dropped*. Resolving either needs
  a trimmed-offset kernel this evaluator does not have. The body exists, so
  the answer is `ErrUnsupported` (§1) — never a self-overlapping loop swept into
  a solid, which is the same principle evaluator §12 states for the tapered
  extrude. Refusing costs the caller a shell decad could in principle build;
  producing one costs them a part that is wrong where they cannot see it.

**The result, by which caps were removed and which way the wall grew.** Write `Q`
for the offset section — `P ⊖ t` inward, `P ⊕ t` outward — and `[z0, z1]` for the
sweep interval. **The sense decides which of the two sections is the outer
boundary and which is the cavity**: inward, the outer boundary is `P` and the
cavity is `Q`; outward, the outer boundary is `Q` and the cavity is `P`, because
the wall grew off the original solid and the original solid is what it now
encloses. Write `O` for the outer section and `C` for the cavity section under
whichever sense was asked for. Taking the removed cap as the top (`z1`), a removed
bottom being its mirror:

| Removed | Sense | The body |
|---|---|---|
| **both caps** | `Inward` | a tube: the prism over `P` with `Q = P ⊖ t` as its **hole(s)**, on `[z0, z1]` |
| **both caps** | `Outward` | a tube: the prism over `Q = P ⊕ t` with `P` as its **hole(s)**, on `[z0, z1]` — no cap is kept, so no material is added along the sweep |
| **one cap** | `Inward` | a cup: the outer prism over `P` on `[z0, z1]`, with the cavity the prism over `Q = P ⊖ t` on `[z0 + t, z1]` — the kept cap does not move, and the floor is `t` of the original material |
| **one cap** | `Outward` | a cup: the outer prism over `Q = P ⊕ t` on `[z0 − t, z1]`, with the cavity the prism over `P` on `[z0, z1]` — the original solid *is* the cavity, and the floor is `t` of new material below the kept cap |

Either tube is a plain `prismPayload` — an outer loop with holes — and
canonicalizes to one: everything that consumes a prism consumes it (§10), and a
fillet of a tube's lateral edges builds.

The cup is the one new payload this increment introduces: two co-directional
prisms over the same plane — the outer region on its interval, the cavity region
on its own — which is enough to re-evaluate the body under a placement
(`Body.Placed` composes the motion and rebuilds, evaluator §8) and enough for
every measurement (§8).

**A shelled body has no void, and `Shell.IsVoid()` is false on it.** An opening
is what a removed face *is*, so the cavity's skin reaches the outside through
the rim, and the inner and outer skins are one connected shell — a cup and a
tube alike. A hollow **closed** body — the one shape whose inner skin is a
genuine void shell — would be a shell that removes *no* face, and core §9's rule
that a selector matching nothing is an error, loudly, is exactly what a caller
would have to defeat to ask for it. v1 does not let them: `Shell` removes at
least one face, and how a no-opening shell should be *spelled* is an open
question (§12), not a silent behaviour of the empty match.

Faces of the cup, and each is analytic: the outer walls (planes and cylinders
over `O`) and the kept cap, the **rim** — the removed cap's plane, trimmed to the
annulus between `P` and `Q` — the cavity walls (planes and cylinders over `C`),
and the cavity's own cap. Every edge bounds exactly two faces, so the body
is manifold and watertight by the same structural argument the prism enjoys
(evaluator §10), on a region the §4 audit has already proven simple.

## 8. Mass properties, and why they stay Exact

**A modify op introduces no bound.** Its result is one prism, or two, over
regions the mass-property engine (evaluator §4) integrates in closed form —
`LineSeg`, `CircleSeg` and `ArcSeg` walks are exactly the kinds it already
integrates, and the arcs a fillet and an offset add are those kinds. So every
quantity is `Exact` with a zero `Bound`, and the verification gate passes it at
any tolerance (verification §5):

| Quantity | On a filleted / chamfered prism | On a cup |
|---|---|---|
| `Volume` | `A·h` on the rewritten section | `A_O·h − A_C·h_c` — the outer prism less the cavity prism, on each one's own interval (§7) |
| `Area` | caps + Σ (segment length · h); an arc's length is `rθ`, exact | outer walls + kept cap + rim annulus + cavity walls + cavity cap |
| `Centroid` | the rewritten region's centroid, lifted to the interval's signed midpoint | the mass-weighted difference of the two prisms' centroids |
| `Bounds` | per-segment analytic extremes over the interval | the outer prism's — the cavity is interior |

Each is a difference or a sum of quantities the engine already produces exactly;
none is sampled, and none is fitted. The `Exactness` a modify op reports is
therefore the one it inherited, and there is no path by which a fillet of an
Exact body yields an Approximate one.

**The rewritten section is the body's truth, in exactly the sense a recorded one
is.** A tangent foot is the root of a closed-form equation, computed once, in
floating point — as is every coordinate the seam records and every vertex an
extrude places. `Exact` means the number is the analytic answer and no
approximation was made of the *shape*; it does not mean the arithmetic was
performed in infinite precision, which is a property no evaluator has and which
verification §4 already accounts for, in the only place it can be accounted for:
the coordinates.

## 9. The recipe, provenance, and replay

**The `Step`.** Each op appends one step (core §6.2), and each depends on and
**consumes** the receiver — the receiver is retired from the document and the
result registered, by the uniform rule of core §6:

| Field | Fillet | Chamfer | Shell |
|---|---|---|---|
| `Op` | `OpFillet` | `OpChamfer` | `OpShell` |
| `Inputs` | `[the receiver's StepRef]` | same | same |
| `Selectors` | `[the edge query]` | `[the edge query]` | `[the face query]` |
| `Values` | `[r]` | `[d]` | `[thickness]` |
| `Opts` | nil | nil | `ShellOpts{Sense}` |

Everything else — `Profile`, `Plane`, `Extent`, `Angular`, `Axis`, `Placement`
— is absent, and the wire codec omits it, exactly as the shipped `Step` codec
omits the fields an op does not key.

**The selector is recorded unresolved, and deep-copied.** The query is a value
(core §9), and the step stores a **clone** of it — the same discipline
`extent.go` and `selector.go` already keep: no caller-owned pointer survives into
a recorded step, and none escapes `Recipe()`. The step never records the edges
or faces the query resolved to; that would be the topology index invariant #3
forbids, one level down.

**Replay is deterministic because resolution is.** Selector resolution is a
filter over `Body.Edges()` / `Body.Faces()` in the body's own deterministic
order (evaluator §7), the body being resolved against is itself rebuilt from its
own step, and every gate in §3–§7 is a closed-form test on that geometry. A
replay therefore selects the same edges, computes the same tangent feet, and
builds the same body — which is what makes a recipe the deliverable core §2 says
it is.

**Provenance is inherited, and trimming is not creating.** The rewrite maps each
surviving side face of the receiver to exactly one face of the result — the same
wall, shorter — and the caps to the caps. Those faces carry **the origin roles
they already had**, including the merged multi-role set a canonicalized face
holds (evaluator §3). The modify step's own role attaches only to the faces
whose **surface it introduced**:

| Role | The face |
|---|---|
| `fillet(i,j)` | the blend cylinder at loop `i`'s junction `j` |
| `chamfer(i,j)` | the bevel plane at loop `i`'s junction `j` |
| `shellSide(i,j)` | a cavity wall |
| `shellCap` | the cavity's cap (a cup only; a tube has none) |

So `FaceCreatedBy(filletRef)` selects exactly the blends, `FaceCreatedBy` of the
original extrude still selects the walls it made — a trim does not re-parent a
face — and the rim of a shelled cap keeps the cap's role, its new hole loop
notwithstanding. The body's own `Origin()` is the modify step, role `"body"`.
Roles derive from the record and the deterministic walk order, so a replay
reproduces every one of them (evaluator §3).

## 10. What the result is to everything already shipped

The reduction pays here, and the ledger is short:

- **Tessellation** (`tessellate.go`) chords a prism payload, and a filleted or
  chamfered body IS one — the blend cylinders chord through the same per-curve
  machinery, shared by every face meeting the curve, so the mesh stays
  watertight by construction. The cup's payload is new, and the tessellator
  extends to it in the same increment: its faces are the same analytic variants,
  and the rim annulus is a polygon-with-holes the shipped cap triangulator
  already builds. `STL` and `OBJ` follow for free.
- **The wall survey** (`survey.go`, `survey2d.go`) reduces a prism to its
  section, and the rewritten section is a section: a filleted or chamfered
  body's `MinWallThickness` is read by the shipped 2D kernel with no new code. A
  **chamfer** meets its neighbours at dihedrals of `π` minus the bevel's half
  turn — far beyond any legal draft allowance on a right-angled edge — so it
  reads as an *edge, not a wall*, which is precisely what verification §6 says a
  chamfer is. A **fillet**'s junctions are tangent, and the material's interior
  angle there is `π`: a smooth continuation, not the closing wedge of
  verification §6's knife edge (whose interior dihedral goes to *zero*). So a
  fillet mints no spurious zero wall, and nothing in §6 has to be relaxed to say
  so. The cup's reading is the same kernel run twice: the spanning balls of the
  wall are the annulus section's, with the wall's height as the vertical fit,
  and the balls spanning the floor are pinned between the kept cap and the
  cavity's cap — diameter exactly `t`. Lateral and cap contacts sit 90° apart,
  past every legal allowance, so the enumeration is complete and the reading is
  Exact.
- **Undercuts** are a per-face survey of exact normal ranges, and every face a
  modify op makes is a `Plane` or a `Cylinder` whose range the shipped survey
  already reads. A blend cylinder whose axis is parallel to the pull is
  everywhere perpendicular to it and provenly clears; one wrapped across the
  pull opposes over part of its range and is listed. No new rule, and no new
  undecided case.
- **`MinRadius`** must see a concave fillet's radius, and it does: the survey
  walks the section's arcs against the material, and a fillet of a *concave*
  edge is a concave arc of radius `r` in the section. A fillet of a *convex*
  edge adds a convex cylinder, which is not a concave feature and rightly does
  not appear. A shell's cavity is read the same way. The sharp concave edge
  where a cup's wall meets its floor carries no radius — the survey reads faces'
  principal radii, and a spec about the *edges* themselves is one no option
  states, so no verdict enforces it (verification §2).
- **Clearance** (`docs/clearance-design.md`) reads faces, not payloads, and
  every face here is in its table. A filleted body is a first-class operand.
- **The boolean** (evaluator §9) tessellates its operands, so it takes these
  bodies as it takes any other.
- **Body-relative stops** read a body's exact extent along a direction. A
  filleted prism keeps the prism payload that answers it; the cup's payload
  answers with its outer prism's extent, the cavity being interior. So a
  modified body can be the stop of a later `ThroughAll` or `ToFace`, and its
  `StepRef` is recorded in that step's `Inputs` like any other (core §6.2).
- **`Verify`** reads Exact quantities on a body proven valid by construction, so
  a modified body is `Sound` on the same terms an extrude's is. Nothing here adds
  a `Suspect` path.

## 11. Increments

PR-level staging inside evaluator increment 5. Everything not yet landed is
`ErrUnsupported` **at the call** — never a `Verify` reading, and never a body:

| PR | Lands | Still `ErrUnsupported` after it |
|---|---|---|
| 1 | the section rewrite and its §4 audit, `Fillet` on lateral edges (line/line, line/arc, arc/arc corners), the `Step` wiring and provenance inheritance | chamfer, shell; every cap edge; every non-prism receiver |
| 2 | `Chamfer`, equal distance | shell; the asymmetric chamfer |
| 3 | `Shell`: cap removal, the exact erosion and dilation, the topology-preservation audit, the cup payload, and `Tessellate` + the surveys extended to it | side-wall removal; the no-opening shell; the topology-changing offset |

The walk-sense wall orientation (§5) is a **prerequisite**, not a deliverable: a
clockwise circular walk on an outer loop mis-orients its wall in the prism build
today, on profiles a fillet has nothing to do with, and the fix belongs to that
build. PR 1 depends on it landing; it does not carry it.

## 12. Open questions

- **The vertex blend.** A cap-edge fillet, and therefore any blend chain that
  turns a corner, needs a surface for the vertex where three blends meet. It is
  not a cylinder, not a sphere, and not exactly derivable from the section —
  which is why §5 excludes it rather than approximating it. What it should be
  (a setback patch? a spherical corner where the three radii agree, and
  `ErrUnsupported` where they do not?) is undecided, and it is the single
  largest thing standing between this increment and general edge chains.
- **Blend merging.** Two corners of one short wall, filleted in one call so that
  their cutbacks overlap, is `ErrUnsupported` today (§5): the two blend surfaces
  exist, and a kernel that merges them into one — or trims each against what it
  runs into — builds the body. Whether this evaluator should grow that resolution
  step, or whether a caller who asks for it is better served by the refusal, is
  undecided.
- **The topology-changing offset.** A shell whose exact offset merges two loops
  (a slot narrower than twice the wall) is `ErrUnsupported`. Resolving it needs
  a trimmed-offset kernel — the same machinery the tapered extrude wants
  (evaluator §12), which suggests one kernel, one increment, and both callers.
- **The no-opening shell.** A hollow closed body — the only shape with a genuine
  `IsVoid` shell — is a shell that removes no face, and the selector vocabulary
  has no way to ask for it: core §9 makes an empty match an error, and a
  cardinality assertion takes a positive count, so a *satisfied* assertion of zero
  is not a spelling the contract offers. Asking for it therefore means adding
  something — a nullary `WithNoOpenings()` option is the obvious candidate — and
  nothing is chosen. Admitting a zero-count assertion instead would be a change to
  the selector contract, which is core §9's to make, not this document's.
- **The asymmetric chamfer**, and the **variable-radius fillet**: both are
  options with nowhere to land until `ChamferOpts` and `FilletOpts` carry a
  field, and both are recordable when they do. Neither is in v1.
- **Modify ops on a revolve.** A revolve's meridian section stands to its body
  exactly as a prism's section stands to its own — a latitude-circle edge is a
  meridian corner, and rounding that corner sweeps a **torus**, which is in the
  shipped surface set. The reduction of §2 looks like it generalizes intact. It
  is not designed here, and whether it is increment 5's or a later one's is
  open.
