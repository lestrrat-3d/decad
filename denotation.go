package decad

import "github.com/lestrrat-3d/r3"

// This file is both halves of the shared-denotation certificate
// docs/surface-design.md §5.2, §6.2 and §14 Table D row 3 name: two minted
// identities, never a coordinate or a bound, proving two different things
// over two different code paths — coplanarity and coincidence do not follow
// from one another, so admitting one never discharges the other.
//
// The LEVEL half (levelToken, below) lets Body.Patch admit a bounded chain
// of rim vertices its exact gate 3 arm refuses today: N vertices sharing one
// levelID are provably coplanar by shared construction.
//
// The CURVE half (curveToken, at the foot of this file) lets Stitch admit a
// bounded free-edge pair its Table J row J5 refuses today: two edges (or two
// vertices) sharing one non-zero curveID, stated under the same recorded
// motion, provably denote the same curve or point — never by comparing a
// coordinate, a residual or a bound magnitude, and never for two
// independently built bodies, which mint distinct ids by construction.
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

// curveID is the CURVE half of the shared-denotation certificate
// (docs/surface-design.md §6.2 Table J's amendment, §14 Table D row 3): a
// document-local identity minted once per denoted curve or point by the
// evaluator that FIRST builds it, and propagated unchanged — never re-minted
// — by every copier that reproduces the same geometry: rebuildStitchTopology
// (stitch.go), copyFaceUnderContext (unstitch.go) and copyPatchFacesUnder
// (patch_body.go). Two edges (or two vertices) sharing one non-zero curveID
// therefore descend from the SAME mint, and so denote the same curve or
// point exactly, whatever bound each one's own held coordinate carries —
// this is an identity proof by shared construction, never a coordinate,
// residual or bound-magnitude comparison, exactly as levelToken's own doc
// comment states for coplanarity. The zero value means "no certificate" and
// always declines, so every vertex and edge no builder in this package
// stamps keeps refusing exactly as it does today.
//
// Two SEPARATE evaluator calls — two independent Extrude calls building the
// identical profile at the identical extent, say — never share a curveID,
// because the document's own counter never resets and never repeats. That is
// the narrow-but-sound choice §14 Table D row 3 states: it refuses a pair
// that is provably equal (two identically-built rims), and it never admits a
// pair that is not. A caller cannot manufacture two edges that share a curve
// by any means but having ONE evaluator call, or a proven copy of its
// output, stamp them.
type curveID int

// curveToken is what a builder mints and a copier propagates: an identity
// (id) admission compares, and xform, the rigid motion this token is STATED
// UNDER. A placement breaks the certificate: two edges that shared a
// denotation before one body moved no longer denote the same curve AT THE
// SAME PLACE, so xform is part of the comparison alongside id
// (sameCurve, below) — the identical discipline stitch.go's own
// `xform != r3.Identity()` placement check already applies to a whole
// stitched body. A copier that applies an ADDITIONAL motion on top of
// whatever a token already carries restates it with compose, never by
// overwriting xform outright, so a token nested through more than one copy
// (an unstitched face later placed, say) still states the true accumulated
// motion.
type curveToken struct {
	id    curveID
	xform r3.Transform
}

// mintCurve returns a fresh curveToken, document-local and never reused,
// stated under the identity motion: the evaluator that mints one always
// builds at its own record's own coordinates, never at an already-displaced
// copy of them (a PLACED rebuild re-evaluates the record and mints AGAIN,
// fresh, which is safe because a fresh id only ever declines).
func (d *Document) mintCurve() curveToken {
	d.nextCurve++
	return curveToken{id: d.nextCurve, xform: r3.Identity()}
}

// compose restates t under an ADDITIONAL rigid motion applied on top of
// whatever t is already stated under: a copier reading a pristine source
// token and applying its own xform parameter calls this rather than
// building a curveToken literal directly, so a token nested through more
// than one copy states the true total motion. The zero token composes to
// itself (a decline stays a decline), and a motion this evaluator's own
// Transform.Then cannot compose (an overflowing translation, or a linear
// part whose orthonormality has drifted past tolerance) restates as the zero
// token — a conservative decline, never a wrong acceptance, since this
// certificate only ever narrows admission.
func (t curveToken) compose(applied r3.Transform) curveToken {
	if t.id == 0 {
		return curveToken{}
	}
	composed, err := t.xform.Then(applied)
	if err != nil {
		return curveToken{}
	}
	return curveToken{id: t.id, xform: composed}
}

// sameCurve reports whether a and b denote the same curve or point stated
// under the same motion: both non-zero ids (the zero value is "no
// certificate" and always declines), equal ids, and equal xform. It is one
// of the comparisons this certificate ever makes to decide admission — the
// other being stitch_weld.go's own bit-identical guard layered on top,
// which this comparison never substitutes for (docs/surface-design.md §6.2:
// "the certificate admits; bit-identity further narrows; bit-identity alone
// never admits").
func sameCurve(a, b curveToken) bool {
	return a.id != 0 && a.id == b.id && a.xform == b.xform
}
