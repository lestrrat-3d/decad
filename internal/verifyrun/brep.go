package verifyrun

import (
	"maps"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/capblendsurvey"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/radiussurvey"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// This file holds the undercut and minimum-radius surveys over a breprecord.Payload's
// faces (agent-docs/general-boolean-design.md §4.5). internal/breprecord's
// measure.go measures the body (§4.3).

// brepUndercuts surveys a brep body's faces against the pull
// (agent-docs/general-boolean-design.md §4.5's undercut row, DX7's reading): a
// planar face carries one normal, its frame's N turned outward, and a swept
// face sweeps its wall walk's normal range exactly as a prism's side does.
// Both readings go through the exact three-valued decisions analyticundercut.Prism
// uses (survey2d.CapNormalDecision, survey2d.WallNormalDecision) over each
// face's own placed frame, so a straddling face sets undecided without
// discarding a face already proven to oppose. Every outward normal maps
// through the placement's linear part, which a reflection maps correctly. A
// fillet band's pipe patches are read by brepFilletUndercuts (Table DF's DF7).
// The reading is a normal-direction membership, unaffected by a face's
// displacements, as a prism's is (agent-docs/prism-boolean-design.md §12).
func brepUndercuts(budget *proofbound.WorkBudget, b *topo.Body, bp breprecord.Payload, pull r3.Vec) undercutOutcome {
	if bp.BossShell != nil || bp.PocketShell != nil {
		return undercutOutcome{reason: surveyPayloadStaged}
	}
	p, ok := pull.Normalize()
	if !ok {
		return undercutOutcome{}
	}
	roles := facesByRole(b)
	nodeRoles := make(map[string]*topo.Face, len(roles))
	maps.Copy(nodeRoles, roles)
	faces := []*topo.Face{}
	undecided := false
	work := freeform.NewFreeformWork()
	for _, f := range bp.Faces {
		face := nodeRoles[f.Role]
		if face == nil {
			return undercutOutcome{}
		}
		verdict, ok := brepgeom.PullDecision(brepgeom.PullFace{
			Frame: f.Frame, Transform: bp.Xform, Wall: f.Wall,
			Planar: f.Planar(), Outward: f.Outward,
		}, pull, work)
		if !breprecord.ListVerdict(&faces, &undecided, face, verdict, ok) {
			return undercutOutcome{}
		}
	}
	// A route L body's band patches are no face of the record: each is read
	// through its own built Face.NormalAt in its band's face frame, by the
	// reader a cap-blend body's patches take (modify-general Table DG's DG7).
	for bi, band := range bp.LoopBands {
		if bi >= len(bp.LoopPatches) {
			return undercutOutcome{}
		}
		if band.Kind == breprecord.BandFillet {
			if !breprecord.FilletUndercuts(budget, nodeRoles, bp, band, pull, &faces, &undecided, work) {
				return undercutOutcome{}
			}
			continue
		}
		pl := bp.Faces[band.Face].View(bp.Xform)
		if !capblendsurvey.PatchUndercuts(nodeRoles, pl, bp.LoopPatches[bi], p, &faces, &undecided) {
			return undercutOutcome{}
		}
	}
	if undecided && len(faces) == 0 {
		// Keep an entirely undecided result distinct from a proven all-clear.
		return undercutOutcome{ok: true, undecided: undecided}
	}
	handles := make([]*topo.Face, len(faces))
	copy(handles, faces)
	return undercutOutcome{faces: handles, ok: true, undecided: undecided}
}

// brepMinRadius is the tightest concave radius over a brep body's faces
// (§4.5's minimum-radius row, DX8's reading): only a swept face whose wall is
// a circle or an arc walked clockwise in its own frame, with the material on
// its left, curves away from the material — a hole's wall, a groove — and
// its radius is the walk's own, entering the §9.2 aggregate under its own
// proven radius bound, as radiussurvey.Prism takes a prism side's. Planar faces
// carry no radius. A record carrying any section displacement leaves the
// question undecided, as radiussurvey.Prism does: the radius is read off the
// recorded wall, which the face only denotes within that displacement.
//
// A route L body's band patches add no radius the record's walls do not
// already read, on capblendsurvey.CapBlendMinRadius's reduction: a
// Plane is flat, a Cone's tightest azimuthal radius is its side wall's own,
// and an apex Cone shrinks to zero only at a boundary vertex. That holds only
// for a patch the build proves is the Cone it publishes, so a band whose
// patch carries a skew or a non-zero stamped departure leaves the survey
// undecided (modify-general Table DG's DG8). A fillet band that fills a
// concave corner adds its tube radius exactly; one that removes material is
// convex in its tube direction and adds none (loop-fillet Table DF's DF8).
func brepMinRadius(b *topo.Body, bp breprecord.Payload) (radiusOutcome, bool) {
	if bp.SectionDelta() != 0 {
		return radiusOutcome{}, false
	}
	if len(bp.LoopPatches) != len(bp.LoopBands) {
		return radiusOutcome{}, false
	}
	roles := facesByRole(b)
	for _, patches := range bp.LoopPatches {
		for _, patch := range patches {
			f := roles[patch.Role]
			if f == nil || capblend.PatchWindowSkew(patch.Geom) > 0 || f.NormalBound != 0 {
				return radiusOutcome{}, false
			}
		}
	}
	fillets := make([]radiussurvey.BrepFilletRadius, 0, len(bp.LoopBands))
	// A boss or pocket shell's band adds no radius. Its patches keep distance
	// t from a source edge the cavity wraps: the boss's root edge, or the
	// blind pocket's floor edges and corners and the counterbore's shoulder
	// rim. The solid lies inside each cylinder, torus or sphere about that
	// edge, so every patch is convex in its tube direction. A boss torus's
	// azimuthal radius is never below the inner cavity wall's r − t, which
	// that wall already supplies; a counterbore torus faces away from its
	// axis, so it curves the convex way around the axis too.
	for _, band := range bp.LoopBands {
		if band.Kind == breprecord.BandFillet && band.Sigma > 0 {
			// The tag's Minor or Radius, which a placement leaves unchanged,
			// within the radius's unit conversion.
			fillets = append(fillets, radiussurvey.BrepFilletRadius{
				Radius: band.Setback.Dc, Bound: band.Setback.DcDelta,
			})
		}
	}
	walls := make([]sectionrecord.CurveSegment, 0, len(bp.Faces))
	for _, f := range bp.Faces {
		if !f.Planar() {
			walls = append(walls, f.Wall)
		}
	}
	reading := radiussurvey.Brep(walls, fillets, bp.SectionDelta())
	return radiusOutcome{reading: reading.Reading, bound: reading.Bound, ok: reading.OK}, reading.OK
}

// The undercut readers below fold a root body's own face handles into an
// undercut result (agent-docs/verification-design.md §6). Analytic prism, cup, and
// cap-blend surveys read the exact three-valued wall and cap decisions in
// internal/survey2d/survey_undercut.go.
//
// A receiver face's normal-component range is decided over the rationals, not
// as a bare float: a genuinely perpendicular or antiparallel face need not
// land exactly on 0 or -1 in floating point. Both carve-outs are exactly
// decidable from held numbers alone: a straight wall's plane-local tangent, a
// circular wall's swept angle, and the placed frame's own exact directions
// (survey2d.NewPlacedFrameMap, capblend_normal.go).
//
// The caller's ORIGINAL pull is used throughout, never its normalized form,
// because normalizing rounds. Squaring both sides of every comparison against
// -1 (or 0) lets the reader decide without taking a square root of an
// irrational quantity: a component c = num / sqrt(scale2 * pull2) satisfies
// c >= 0 iff num >= 0, and (for num <= 0) c <= -1 iff num^2 >= scale2*pull2,
// an exact rational test either way.
//
// analyticundercut.Revolve reads through revolveangle.Extremes, a different
// reader, and does not use this decision.
