# Loft Mirror Union

This document owns the certified `Union` of one two-section Loft and its
`MirroredCopy` across the Loft's end section. It extends the mirror copy API in
`docs/mirror-pattern-design.md` and the Loft mesh proof in
`docs/loft-design.md`. The result is a faceted solid with retained source and
placement records for later certified consumers.

## Admission and ideal solid

Both operands must be live, solid, authenticated Lofts with the same source
identity, section frames, planes, alignment, and exact recorded profiles.
The lower Loft is unplaced. Its section frames use the XY basis at positive
ascending Z stations. The upper operand has exactly the transform returned by
`r3.Reflection` across the end-section frame. The copy therefore has the same
ideal source, with its Z interval reflected to the other side of the common
section. Their interiors are disjoint and their true middle caps coincide.
Independent equal-looking Lofts cannot establish this identity and refuse the
specialized join.

## Held boundary and proof

Each operand first passes `Tessellate(VerifyAll)`. Its mesh must exactly match
its authenticated Loft payload's vertices and triangles. The end cap vertices
must agree exactly after reflection, and the cap triangulations must have the
same indexed triangles with opposite winding. The join keeps the lower and
upper exterior caps and both sets of source walls. It removes the two middle
caps and identifies their common station vertices. Its three rings comprise
the lower start, common middle, and reflected upper start stations. Whole-mesh
vertex-link and triangle-crossing audits must accept the sewn shell.

The result's boundary displacement is at most the greater of the two source
mesh bounds, because every retained wall and cap is one of their verified
pieces. The occupied-volume symmetric difference is at most the sum of their
source allowances: union is non-expansive for symmetric difference, and the
removed caps occupy no volume. The area allowance conservatively sums both
source area allowances and charges twice the larger allowance for the removed
middle caps. Non-finite proof values refuse. The ordinary faceted builder
then proves topology, mass and bounded area and volume readings from this
record. `Tessellate(VerifyAll)` checks the final result at a caller's target
tolerance.

The `LoftMirrorJoin` record preserves both original Loft operands, separate
lower and upper wall groups, outer cap groups, and the number of vertices in
each station. Both retained outer caps keep their source origins and planar
surface, with new Union cap roles appended. A later rigid `Copy` keeps a
source-backed placement record, gives those caps copy-local roles, and
revalidates the held mesh and displacement bounds. This record does not by
itself authorize any other Boolean case.

## Scope

`Union` outside this one-source reflection rule follows the existing Boolean
dispatch. A centered-cylinder Join proves its own contacts, sections, and
occupied volume before it consumes this record.

## Centered cylinder and repeated reflected tabs

`Union` also accepts one exact centered circular prism and a mirrored Loft
pair retained by this route. Later unions may add rigid XY rotations of the
same pair, up to the circular Join's 32-source limit. A completed result keeps
the exact blank and pair operands for each next step. A placed result loses
that incremental source record.

The blank must have exact XY sections, one whole circle, and no section
displacement. Its bottom equals the lower Loft's start plane. The mirrored
pair's middle plane must be the exact dyadic midpoint of the blank's two cap
planes. Every later copy must retain the first pair's source identity and
use an axial rotation with no translation. The reflection and each admitted
rotation commute, so the true union is the reflection of a lower-half union
against the exact lower half of the blank. A rotation whose axial basis is
one float step below one uses the bounded bridge below.
`loftcylinder.MirrorBlank` checks the blank and `loftcylinder.MirrorPairMotion`
reads each pair's recorded placement.

For every step, the existing Loft-to-cylinder Join rebuilds that lower-half
union from the original source Loft and the admitted rotated copies. It proves
the radial seam, pairwise source separation, held boundary, and occupied
volume. The result is reflected using the faceted placement proof. The two
held middle caps must have identical triangles with opposite winding, and
their vertices must coincide exactly. Both caps are removed. Vertex-link and
whole-mesh crossing audits check the sewn shell.

The full body's boundary departure is the larger half bound. Its occupied-
volume error is at most the sum of the two half errors because the true halves
have disjoint interiors. Its area allowance sums the two half allowances and
twice the larger allowance for the removed common cap. The held diameter plus
twice its boundary departure supplies `D` to the faceted centroid allowance
`min(D, volSymDiff × D / volumeFloor)`.
The route refuses a non-finite proof or a boundary departure above 0.1 mm;
ordinary Boolean chord admission is unchanged.

The exterior cap groups keep the original blank and every retained pair's
source origins, plus the new Join cap roles. Exact axial placements keep
both cap planes; a bridged top cap reports a faceted surface with its charged
bound. `VerifyAll` independently
checks the final held shell and occupied-volume allowance at the caller's
chosen tolerance.

## One-pass mirrored pattern Join

Multi-body `Union` accepts the same centered prism and two to thirty-two distinct
copies of one authenticated mirrored pair. It checks the common source and
each recorded placement before any operand is consumed. The circular Join
builds the lower half once from every original lower Loft; it proves their
continuous sector separation, root contacts, cap disks, boundary and volume.
The mirrored result is sewn and audited once. This avoids repeatedly
rebuilding all prior teeth for each binary `Union`.

The admitted placed copies have zero translation and an XY rotation. Their
Z basis is either exactly one or `Nextafter(1, 0)`. For the latter, the lower
circular Join uses its authenticated axial proxy and charges the difference
between the original lower Loft and that proxy. Let `F` reflect about the
recorded middle height `m`, and let `T` be the copy's stored affine motion.
The true upper source `T(F(L))` differs from the reflected true lower source
`F(T(L))` by the exact constant Z translation `2m(1-T.EZ.Z)`. The evaluator
computes an upward rational bound for that shift. It adds the shift to all
reflected upper vertex and boundary bounds and charges the source pair's
certified area times the shift to occupied-volume difference
(`loftcylinder.MirrorUpperBridge`). The lower
circular Join's moving-plane tooth-separation test also widens each affected
tooth's margin by the upper shift times its maximum XY radius and the plane's
normal change per unit height. This keeps upper teeth apart, including the
thin level where a shifted upper tooth meets another tooth's lower half. A
fixed through-axis separator needs no height allowance. The lower
circular Join's area allowance already includes the held joined area, blank
area, and every source tooth's area. This also covers any short surface
exposed by the shifted upper tooth. The original pair's common cap moves as
one; the blank fills the small level difference. This proof does not admit an
arbitrary tilted or translated copy.

The final result retains its authenticated private lower-half Join, the
original full blank and pair bodies, and the exact bottom and top cap groups
and outline loops. Both final cap triangulations must tile those outlines;
the loops and lower-half source record support later certified consumers.
The boundary limit remains 0.1 mm, the final mesh still passes whole-shell
vertex-link and crossing audits, and the faceted builder publishes bounded
volume, area, centroid and diameter readings before the atomic commit.

## Centered through-bore

`Cut` accepts an exact centered circular prism through an unplaced, audited
reflected Loft/cylinder join. The join record retains its certified lower-half
Loft/cylinder join and the full body's exterior cap groups and outlines.
Both exterior cap disks are checked against those recorded outlines before
the Cut. The existing lower-half bore route authenticates the cutter and
proves its inner wall and annular caps. The cutter must extend beyond both
exact end planes of the full blank.

The bored lower half is reflected through the original Loft end frame. The
two middle annuli must have matching held vertices and oppositely wound
triangles; they are removed and their vertices are welded. The remaining
mesh passes vertex-link and whole-mesh crossing audits. Its boundary departure
is the greater half bound. The occupied-volume allowance sums both half
allowances, and the area allowance also charges the removed middle annuli.
The final held diameter and departure bound the centroid reading. Both
exterior annuli must tile the blank's exact end planes. The result keeps the
blank's cap origins and receives Cut cap roles; `VerifyAll` checks its boundary
and occupied-volume allowances at the requested tolerance.

For a near-axis placed pair, the Join record carries its upper source
translation bound and swept-volume charge into the Cut. The reflected upper
vertex and mesh bounds include that translation, and the bored top cap retains
the faceted surface tag when the original upper tooth ends below it.

## Both exterior cap Chamfers after a centered bore

An unplaced, audited mirrored Join with its certified centered through-bore
accepts one equal setback on both complete exterior outer cap loops. The inner
bore loops and all side edges stay sharp. The private bored lower half gets
one start-cap Chamfer. Its opposite cap and middle annulus remain unchanged;
the audited half is then reflected, its matching middle annuli are removed,
and the common middle vertices are welded. Separate cap and wall audits prove
the two exterior annuli, unchanged bore wall, and strict separation of the
lower and upper slabs outside their shared seam.

The lower-half Chamfer is `loftcylinder.ChamferBottomCap`, the two-cap
builder of the loft-cylinder design run with its top cap inactive.
The one-cap cutter uses exact side and inset loops, three source-occupied
split rings, and exact full-band distance checks. Its band can cross the
side-level ring in XY when the three-dimensional contact audit proves the
band touches the old held wall only on the intended side ring. The source
cap and retained side audits still prove both directions of boundary reach.
The reflection uses the placed-mesh rounding envelope
(`loftcylinder.MirrorChamferReflectionReserve`), checked against the
measured upper-half bound. The final boundary allowance reserves this rounding
before building the lower half.

For a near-axis placed pair, the actual upper source tooth lies below its
reflected lower proof tooth by the Join's authenticated axial shift. The
three lower cap-band split rings are checked again against each tooth at its
shifted physical source level. The circular Join sorts lower teeth by angle,
so `loftcylinder.MirrorChamferUpperShifts` matches each shift to its
authenticated placed tooth source instead of its pair input position. An
exact section through the reflected upper band at the lowest possible source
cap proves the intervening cap sliver's
reach. The measured vertical and lateral reach is charged to the upper
vertex and boundary bounds. The Join's swept-volume charge is carried into
the final occupied-volume allowance. Both charges must fit the caller's
0.1 mm `VerifyAll` tolerance.

`loftcylinder.AssembleMirrorChamfer` sews the two halves from their payloads
and the bore record. It runs the seam, vertex-link, exterior annulus and
bore-wall audits and the upper cap section, then proves the result's bounds.
The boundary bound is the larger of the lower half's bound and the upper
half's bound plus the measured cap reach; a bound above 0.1 mm refuses. The
occupied-volume allowance sums both half allowances and the Join's
swept-volume charge. The area allowance sums both half allowances and twice
the larger one for the removed middle annuli. The held diameter and boundary
bound give the centroid allowance. The root reflects the lower half within its
document and gives the exterior cap groups the bored Join's cap origins;
`loftcylinder.TagCaps` tags them with the blank's planar cap surfaces.
