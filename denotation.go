package decad

import "github.com/lestrrat-3d/r3"

// This file is the LEVEL half of the shared-denotation certificate
// docs/surface-design.md §5.2 and §14 Table D row 3 name: a minted identity,
// never a coordinate or a bound, that lets Body.Patch admit a bounded chain
// of rim vertices its exact gate 3 arm refuses today. The other half — a
// CURVE token proving two edges denote the same curve, which lifts Stitch's
// own Table J row J5 — is a separate proof over a separate code path and is
// not part of this file or this increment; coplanarity and coincidence do
// not follow from one another (§14 Table D row 3).
//
// levelToken is minted once per (recorded plane frame, denoted sweep level)
// pair and never reused. Two vertices or rim edges that carry a levelToken
// with the SAME non-zero id are provably coplanar: their true positions are
// frame.Origin + u·U + v·V + L·N for one denoted level L, whatever bound
// each one's own held coordinate carries, because both were stamped by the
// SAME evaluator call over the SAME recorded frame and level. This is a
// coplanarity proof by shared construction, never by comparing a
// coordinate, a residual or a bound magnitude — CLAUDE.md's rule that a
// small residual never admits an input has nothing to check here, because
// admission (sameLevel) reads id ALONE and nothing else in the struct.
// The zero value means "no certificate" and always declines, so every
// vertex and edge no builder in this package stamps keeps refusing exactly
// as it does today.
//
// origin and normal carry what admission itself never reads: the SAME
// world-space point and direction prism_build.go's own capFrame already
// derives for a solid prism's caps (pp.point(0, 0, z) and pp.dir(0, 0, 1)),
// never a value fitted to held vertex coordinates. This is what lets
// Body.Patch's level arm publish an EXACT plane for a chain it admits by
// identity alone: the held vertices at one recorded level are only
// APPROXIMATELY coplanar in float64 — frame.ToWorldUV(u, v) rounds
// differently for each distinct (u, v), so two vertices sharing one level
// generally do NOT land on bit-identical planes, even though their true
// (unrounded) positions do. Fitting a normal to that approximately-coplanar
// data (Newell's method, patchChainOrientedNormal) would publish a plane
// tilted from the true one by that same float rounding, with no term in
// axialDelta to cover it — axialDelta only ever states the OFFSET
// displacement along an already-exact normal, on the same terms
// prism_build.go's own cap axialDelta does, never a tilt of the normal
// itself. Carrying origin/normal on the token, and reading them instead of
// fitting one, is what keeps that promise: the published normal is exactly
// pp.dir(0, 0, 1) under whatever placement composes onto it, and only its
// ORIGIN'S position along that exact direction is ever in question.
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

// levelToken is what a builder mints and stamps onto a Vertex or Edge: an
// identity (id) admission compares, plus the recorded plane data
// (origin, normal) a consumer reads AFTER admission to publish the chain's
// plane exactly, never by fitting one to held (only approximately
// coplanar) vertex coordinates. See this file's own doc comment.
type levelToken struct {
	id             levelID
	origin, normal r3.Vec
}

// mintLevel returns a fresh levelToken, document-local and never reused:
// origin and normal are the level's own world-space point and direction,
// read verbatim from the caller's own frame-derived construction (never
// fitted).
func (d *Document) mintLevel(origin, normal r3.Vec) levelToken {
	d.nextLevel++
	return levelToken{id: d.nextLevel, origin: origin, normal: normal}
}

// sameLevel reports whether a and b denote the same plane: both non-zero
// ids (the zero value is "no certificate" and always declines) and equal.
// It is the only comparison this certificate ever makes to decide
// admission, and it is an identity comparison over id alone — origin and
// normal never enter it.
func sameLevel(a, b levelToken) bool {
	return a.id != 0 && a.id == b.id
}
