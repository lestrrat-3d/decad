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

## First construction class

This milestone converts the general source certificate to the existing
one-sector builder. That temporary adapter accepts six geometric section
spans, a cone with zero axial origin, and six whole sector entities. It uses
the existing finite-root crossing, fan-cell, topology/contact, source-face,
area and volume audits. The ordinary Boolean route remains available when
the certificate or adapter refuses. A later dynamic sector builder will use
the ordered spans and exposed loop directly. Until then, this route is not a
replacement for multi-sector joins or every finite Revolve section.

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
| `bevelTooth` and `bevelJoin` payloads and dispatch | `revolveLoftTooth` and `revolveLoftJoin` records, then dynamic placement and repeated-sector `Union` | A real neutral rotated-lug producer through public `Union` and `VerifyAll` is still required |

The temporary conversion still calls `buildBevelOneToothJoin`. A gallery
consumer must pin and pass the later fully generic builder before the old
file family can be deleted.
