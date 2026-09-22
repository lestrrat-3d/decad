package decad

// This file is the LEVEL half of the shared-denotation certificate
// docs/surface-design.md §5.2 and §14 Table D row 3 name: a minted identity,
// never a coordinate or a bound, that lets Body.Patch admit a bounded chain
// of rim vertices its exact gate 3 arm refuses today. The other half — a
// CURVE token proving two edges denote the same curve, which lifts Stitch's
// own Table J row J5 — is a separate proof over a separate code path and is
// not part of this file or this increment; coplanarity and coincidence do
// not follow from one another (§14 Table D row 3).
//
// levelID is a document-local identity, minted once per (recorded plane
// frame, denoted sweep level) pair and never reused. Two vertices or rim
// edges that carry the SAME non-zero levelID are provably coplanar: their
// true positions are frame.Origin + u·U + v·V + L·N for one denoted level L,
// whatever bound each one's own held coordinate carries, because both were
// stamped by the SAME evaluator call over the SAME recorded frame and level.
// This is a coplanarity proof by shared construction, never by comparing a
// coordinate, a residual or a bound magnitude — CLAUDE.md's rule that a
// small residual never admits an input has nothing to check here, because
// nothing is ever compared numerically. The zero value means "no
// certificate" and always declines, so every vertex and edge no builder in
// this package stamps keeps refusing exactly as it does today.
//
// mintLevel's one caller today — prism_build.go's evalPrismContext — mints
// only when pp.sectionDelta == 0 AND the payload's frame axes are RECORDED
// rather than computed: a straight prism's frame is always the sketch
// plane's own frame, carried through unchanged from
// extrude.go/fillet.go/chamfer.go/shell.go/patch.go, never derived by this
// evaluator through a rotation or a cross product of its own — so
// sectionDelta == 0 alone already states the whole precondition for every
// prismPayload this evaluator builds. A revolve's axis frame, by contrast, is
// COMPUTED (revolve_axis.go's axisFrame), and a revolve's own seam
// half-plane has a BOUNDED normal, not merely a bounded offset — so
// revolve_build.go mints no level token at all, and a revolve seam chain
// stays refused by Body.Patch's existing exact gate (R6). A section
// re-expressed by the analytic prism boolean reduction sets sectionDelta
// nonzero and so mints no token either: its coordinates are computed, not
// recorded, and the whole point of this certificate is that it never covers
// a computed coordinate.
//
// Every mint is fresh: two separate evaluator calls — two separate Extrude
// calls building the identical profile at the identical extent, or one call
// and a later Placed/Duplicate/PlacedCopy rebuild of it (evalPrismContext is
// their shared entry point) — never share an id, because the document's own
// counter never resets and never repeats. A caller cannot manufacture two
// edges that share a level by any means but having this evaluator stamp them
// within the SAME build.
type levelID int

// mintLevel returns a fresh levelID, document-local and never reused.
func (d *Document) mintLevel() levelID {
	d.nextLevel++
	return d.nextLevel
}

// sameLevel reports whether a and b denote the same plane: both non-zero
// (the zero value is "no certificate" and always declines) and numerically
// equal. It is the only comparison this certificate ever makes, and it is an
// identity comparison, never a numeric one over a coordinate or a bound.
func sameLevel(a, b levelID) bool {
	return a != 0 && a == b
}
