# Finite Revolve and trimmed point-Loft Union

## Source admission

The first public `Union` route accepts a finite, unplaced, full axis-X Revolve
of one closed Sketch section and one point-Loft sector after authenticated
toe Cut and heel Intersect. The section consists of whole straight edges.
Exactly forward-collinear edges become one geometric span; every such span
keeps the complete set of recorded source roles and its retained face. The
section has one axial span. A candidate root span must have a finite positive
conical surface, and its recorded meridian is compared with that surface over
the source fan's complete axial reach. No fixed number of geometric spans
enters this source certificate.

The point-Loft source must still be the current Sketch profile, revision,
frame and ordered entity list recorded by Loft. Sketch alone certifies two
exact crossings against the reported cone. The first admitted class has two
connector-line crossings next to one strictly inside root arc. The second
has two fitted-spline crossings with strict source-side transitions and a
strict local coordinate derivative. All other whole entities need strict
Sketch side certificates with margins above the source-fan displacement and
finite-meridian allowance. A missing, stale, approximate or ambiguous answer
refuses the route. Partial entities are refused even when their endpoints are
exact: this route has no side certificate restricted to a partial span.

The certificate retains ordered section spans, both axis endpoints through
the axial span, root face and cone, Sketch crossing/side answers, source
revision, sector endpoints, pre-refinement exposed loop, and crossing bound.
The exterior path chooses the unique pair of source fan cells that contain
the two contacts. The fitted route uses Sketch's parameter brackets, local
coordinate derivative and recorded Loft stations. Both routes charge the
crossing enclosure separately from the source fan displacement.
The loop keeps source group and root-contact labels on each edge. The joined
body retains the original sources and published boundary, occupied-volume
and area allowances. Placement clears
the direct join certificate and records an authenticated rigid copy of the
original trimmed source.

## Sector construction

The builder meshes every recorded meridian span at one shared angular plan.
It omits a root-face interval only where a certified sector cap replaces that
interval. Its rings follow section order, so a shoulder or split cap adds a
ring without changing any positional profile role. The source cap is
triangulated and refined before placement by each authenticated axis
rotation. The original unplaced source controls each cone map; placement is
applied once after that map. The placed caps share the exact ring vertex IDs
at every root station.

For repeated sectors, the complete trimmed source solids must be separate by
one certified projection for every pair. The builder first checks expanded
world-coordinate ranges. When those ranges overlap, it chooses a YZ direction
from the held box centers and projects every held source vertex with exact
rational arithmetic. Each projection range expands by the source spatial
bound and authenticated placement charge. A direction whose enlarged ranges
overlap refuses the pair; choosing the direction alone proves nothing.
A seam is selected from the
unused angular complement. Each pair of root intervals must remain separate
after source mesh, crossing, and placement bounds are converted to an angular
allowance at the finite root radius. The seam uses held root intervals and
those pairwise margins; it is not a separate Sketch side certificate. All
admitted sectors use one ring mesh.
Serial and variadic `Union` use the same constructor; serial joins retain the
original Revolve, source certificate, placed sources, and dynamic ring IDs.

Each construction must pass connected orientation, vertex links, the exact
triangle contact audit, a 0.1 mm boundary bound, finite area and occupied
volume bounds, and a measured-volume increase over the original Revolve.
For each cap whose held triangles form a single graph over the plane normal
to the Revolve axis, the audit may omit pairs within that cap. It proves
strict and consistent projected triangle orientation, opposite use of each
internal edge, one directed boundary loop, and a simple exact projected
boundary. These facts make the oriented triangle chain cover its boundary
region once, so distinct triangle interiors cannot meet. Failed cap proofs
leave every pair in the exact audit. Separate cap families and all cap-wall
and wall-wall pairs always remain in the audit. A family-aware box tree
counts its node visits and leaf comparisons under the existing per-call
scan ceiling before allocating candidate pairs.
The audit triangle allowance follows the section-ring and cap cell counts,
with a one-million-triangle memory ceiling. The crossing scan allowance is
the smaller of all triangle pairs and 25 million scans per admitted sector.
If the cone cap interpolation term exceeds the remaining 0.1 mm boundary
budget, the builder splits only cap cells above that term's target. Shared
edges split together, including root edges. It rebuilds the shared ring and
recomputes the cone cap and full boundary bounds after refinement. Each
construction attempt permits at most three additional cap passes within the same mesh
ceiling; a cap that still exceeds the budget refuses the join.
An occupied-volume refusal may refine the angular plan up to eight times.
A specific cap/root adjacency contact may also trigger refinement; the new
mesh must pass the same exact audit. Refusal leaves all operands live.
The current source class requires a canonical unplaced, full axis-X Revolve.
Off-axis and transformed Revolves use the ordinary Boolean route or refuse.

The real neutral acceptance model uses a tapered Revolve hub whose outer cap
has two exactly collinear source edges. A separate Sketch makes a radial lug,
point-Loft makes its solid fan, and two real conical tools trim its toe and
heel. Public `Union` passes `Tessellate(0.1 mm, VerifyAll)` for both straight
connector and fitted flank sources. The mesh has Euler characteristic two,
both verification flags are true, and the joined computed-volume lower bound
exceeds the hub's computed-volume upper bound. Its joined faces retain both
split-cap source roles and an analytic root cone.

## Removal ledger

No old production file is removed in this milestone. Each row names the
generic replacement and focused proof required before deletion.

| Old path or record | Generic replacement | Focused proof |
|---|---|---|
| `bevel_join_source.go` | `revolve_loft_source.go` and `revolve_loft_sector.go`, then a dynamic builder without `legacyRevolveLoftBlank` | `TestUnionTaperedHubAndRadialLug` plus split-cap, stale-source, foreign-trim and near-contact refusals |
| `bevel_join_fit.go` | `revolve_loft_fit.go` and a generic fitted-seat/fan-cell builder without `legacyRevolveLoftBoundary` | `TestUnionTaperedHubAndFittedRadialLug` plus fitted stale-source and near-contact refusals |
| `bevel_join.go`, `bevel_join_two.go`, `bevel_join_pattern.go`, `bevel_join_combine.go` | Dynamic section rings, sector caps and shared N-sector construction | Neutral seven-span, fitted, serial, variadic and separation-refusal tests pass; real gallery consumers remain to be checked |
| `bevelTooth` and `bevelJoin` payloads and dispatch | `revolveLoftSector` and `revolveLoftJoin` records | Real neutral rotated lugs pass public `Union`, 0.1 mm `VerifyAll`, Euler two and computed-volume tests |

The old specialized files remain until a gallery consumer pins and passes
the generic builder. The generic constructor does not call those files.
