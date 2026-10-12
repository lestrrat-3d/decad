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

The `loftMirrorJoin` record preserves both original Loft operands, separate
lower and upper wall groups, outer cap groups, and the number of vertices in
each station. Both retained outer caps keep their source origins and planar
surface, with new Union cap roles appended. A later rigid `PlacedCopy` keeps a
source-backed placement record, gives those caps copy-local roles, and
revalidates the held mesh and displacement bounds. This record does not by
itself authorize any other Boolean case.

## Scope

`Union` outside this one-source reflection rule follows the existing Boolean
dispatch. The separate tooth-to-cylinder join must prove its own contacts,
sections, and resulting occupied volume before it can consume this record.
