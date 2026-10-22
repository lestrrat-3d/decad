# Boolean of axial extrusions by the 2D Boolean of their sections

A Union of bodies that are each an axial extrusion of one recorded section between the same two parallel planes is the
axial extrusion of the 2D Boolean of those sections. `internal/axialsection` builds that result: it asks `sketch` for the
2D answer and records it as a matching axial loft, so the loft routes (`agent-docs/loft-design.md` §17, §18) read it.

The rule names no shape. It applies to any such operands, whatever the sections hold.

## 1. Operands

`axialsection.OperandOf` reads a payload as an axial extrusion:

- A `prismPayload` with no section displacement, no sweep-level displacement, the identity placement, no blend and no
  holes.
- A matching axial `loftPayload` (loft-design §17): two identical recorded profiles with no holes, equal section axes,
  end planes that differ only along their shared normal (`prismplacement.SharedAxisOf`, exact), the identity placement,
  and no bore, blend, through-bore Cut or surface result. A loft's own chord displacement is not read: this route rebuilds
  from the recorded profiles, as the loft's own placement does.

A prism that carries a section displacement is not an operand, because the displacement states how far its record sits
from the solid it stands for and this route rebuilds from records.

Operands are admitted together only when every one lies between the same two planes: equal section axes and equal exact
offsets of both end planes along the normal. At least one must be a loft, which records the planes. Anything else is a
miss, and the Union takes its next route.

## 2. Flow

1. Admit every operand (§1). A set none of whose sections holds a free-form segment is left to the analytic prism routes
   (`prism-boolean-design.md`), which carry their own proofs for line, arc and circle sections.
2. Build one private `sketch` scene holding every operand's recorded outer loop: one entity per distinct recorded curve
   across all operands, every point fixed from the records. Only whole edges are admitted (a segment whose range is not the
   whole curve is a miss), and the kinds the scene builder creates are line, arc, circle and fit spline.
3. Declare one loop per operand (`Sketch.DeclareLoop`) with its entities in recorded order.
4. Read the arrangement's regions (`Sketch.Profiles`). A valid region that is `Inside` at least one loop belongs to the
   union. A region that is not `Inside` any loop and is `Unproven` for some loop leaves the question open, which is a miss.
5. `UnionProfiles` of the chosen regions. A union `sketch` declines to build, or one with a `Partial` edge that is not
   `TExact`, is a miss. This is decad's reject-only rule (CLAUDE.md): sketch's own certificate decides, and decad never
   repairs a fragment it reports inexact.
6. Record the union profile through the seam (`sectioncert.RecordProfile`) and build the loft of two identical sections
   between the operands' end planes with `loftrecord.Build`. The result is a matching axial loft whose own bounds are
   computed fresh.

The route runs for the many-operand Union (`Union(base, tools...)`) ahead of the other many-operand routes, and for a
two-operand Union ahead of the analytic prism routes and after the loft routes that need a recorded join.

## 3. What is a miss, and what is an error

A miss returns `ok = false` with a nil error and the Union takes its next route, so a set this route cannot certify behaves
as it did before it existed. An error is a genuine failure of the build (a canceled context, a loft build refusal after the
section was certified).

Misses: fewer than two operands; an operand that is not an axial extrusion; operands on different planes; a hole in any
section; a trimmed segment in any section; a segment kind the scene does not create; a region `sketch` cannot place inside
or outside; a union `sketch` declines to build, such as two edges that share only part of a line (a flush rib); and a
fragment `sketch` reports inexact.

A trimmed segment is the one that matters in practice: the union of a disc and a lobe records the circle between them as a
fragment, so the result is not an operand of a second two-body Union. The many-operand Union joins all operands in one scene
and so avoids it. A later `DeclareLoop` element that names an entity with a parameter range would admit chains.

## 4. Limits

- Rotated copies. A `Copy` with rotation of a prism keeps the section record and carries the rotation as a placement, and a
  `Patterned` instance of a prism whose segments are all line, arc or circle moves the record and charges its rounding as
  a section displacement. Neither is an operand: the first has a placement, the second a displacement. A spline section needs
  a certified bound on how far a fit spline through rounded points departs from the rotated original, which this route does
  not have, so a part with spline sections draws each instance at its angle in the sketch.
- The loft fillet (§17) takes a line or arc meeting an arc, or a fit spline meeting a circle. A line meeting the circle
  fragment this union leaves is outside it, so a root fillet does not apply to the result.
- Union only. Cut and Intersect, and sections with holes, are later.
- The 2D union adds a certified `sketch` arrangement per Union: about 20 ms for 25 regions with fit splines.

## 5. Tests

- `axialsection` unit tests (no free-form segment needed at this level): a plate and a rib that crosses its top edge
  (outline segments and area), three rectangles forming an L and a T, flush rectangles (a miss), a disc and N lobes with fit
  splines on its circle for N = 8, 12 and 17 (area equals the sum of the parts, 6N outline segments), and the misses (one
  section, a hole, a trimmed segment).
- `apitest.TestAxialSectionUnionLobesOnADisc`: the joined volume against disc plus lobes within its bound, `VerifyAll`
  boundary and volume verified, for N = 8, 10, 12 and 17. `TestAxialSectionUnionPlateAndTongue`: a plate and a curved-top
  tongue, the overlap an exact rectangle. `TestAxialSectionUnionFinishes`: a bore Cut and an outer-cap Chamfer on the result.
