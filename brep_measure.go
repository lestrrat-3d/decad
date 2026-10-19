package decad

import (
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/capblendsurvey"
	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/radiussurvey"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// This file holds the undercut and minimum-radius surveys over a brepPayload's
// faces (docs/general-boolean-design.md §4.5). internal/breprecord's
// measure.go measures the body (§4.3).

// brepUndercuts surveys a brep body's faces against the pull
// (docs/general-boolean-design.md §4.5's undercut row, DX7's reading): a
// planar face carries one normal, its frame's N turned outward, and a swept
// face sweeps its wall walk's normal range exactly as a prism's side does.
// Both readings go through the exact three-valued decisions analyticundercut.Prism
// uses (survey2d.CapNormalDecision, survey2d.WallNormalDecision) over each
// face's own placed frame, so a straddling face sets undecided without
// discarding a face already proven to oppose. Every outward normal maps
// through the placement's linear part, which a reflection maps correctly. A
// fillet band's pipe patches are read by brepFilletUndercuts (Table DF's DF7).
// The reading is a normal-direction membership, unaffected by a face's
// displacements, as a prism's is (docs/prism-boolean-design.md §12).
func brepUndercuts(budget *proofbound.WorkBudget, b *Body, bp brepPayload, pull r3.Vec) undercutOutcome {
	if bp.BossShell != nil || bp.PocketShell != nil {
		return undercutOutcome{reason: surveyPayloadStaged}
	}
	p, ok := pull.Normalize()
	if !ok {
		return undercutOutcome{}
	}
	roles := facesByRole(b)
	faces := []*Face{}
	undecided := false
	work := freeform.NewFreeformWork()
	for _, f := range bp.Faces {
		face := roles[f.Role]
		if face == nil {
			return undercutOutcome{}
		}
		verdict, ok := brepgeom.PullDecision(brepgeom.PullFace{
			Frame: f.Frame, Transform: bp.Xform, Wall: f.Wall,
			Planar: f.Planar(), Outward: f.Outward,
		}, pull, work)
		if !listVerdict(&faces, &undecided, face, verdict, ok) {
			return undercutOutcome{}
		}
	}
	// A route L body's band patches are no face of the record: each is read
	// through its own built Face.NormalAt in its band's face frame, by the
	// reader a cap-blend body's patches take (modify-general Table DG's DG7).
	nodeRoles := make(map[string]*topo.Face, len(roles))
	for role, face := range roles {
		nodeRoles[role] = faceNode(face)
	}
	for bi, band := range bp.LoopBands {
		if bi >= len(bp.LoopPatches) {
			return undercutOutcome{}
		}
		if band.Kind == breprecord.BandFillet {
			if !brepFilletUndercuts(budget, roles, bp, band, pull, &faces, &undecided, work) {
				return undercutOutcome{}
			}
			continue
		}
		pl := bp.Faces[band.Face].View(bp.Xform)
		var patchFaces []*topo.Face
		if !capblendsurvey.PatchUndercuts(nodeRoles, pl, bp.LoopPatches[bi], p, &patchFaces, &undecided) {
			return undercutOutcome{}
		}
		for _, face := range patchFaces {
			faces = append(faces, faceHandle(face))
		}
	}
	if undecided && len(faces) == 0 {
		// Keep an entirely undecided result distinct from a proven all-clear.
		faces = nil
	}
	return undercutOutcome{faces: faces, ok: true, undecided: undecided}
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
func brepMinRadius(b *Body, bp brepPayload) (radiusOutcome, bool) {
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
			if f == nil || capblend.PatchWindowSkew(patch.Geom) > 0 || f.n.NormalBound != 0 {
				return radiusOutcome{}, false
			}
		}
	}
	fillets := make([]radiussurvey.BrepFilletRadius, 0, len(bp.LoopBands))
	if bp.BossShell != nil {
		fillets = append(fillets, radiussurvey.BrepFilletRadius{Radius: bp.BossShell.T})
	}
	if bp.PocketShell != nil {
		fillets = append(fillets, radiussurvey.BrepFilletRadius{Radius: bp.PocketShell.T})
	}
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
// undercut result (docs/verification-design.md §6). Analytic prism, cup, and
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

// listVerdict folds one public face's verdict into the running faces list and
// undecided flag used by the root BRep undercut readers. Analytic
// undercut packages have their own node-typed fold. Here ok=false (a
// non-finite input or a failed enclosure) propagates as a total refusal to
// the caller, matching every other refusal path these surveys already take.
func listVerdict(faces *[]*Face, undecided *bool, f *Face, verdict survey2d.PullVerdict, ok bool) bool {
	if !ok {
		return false
	}
	switch verdict {
	case survey2d.PullOpposes:
		*faces = append(*faces, f)
	case survey2d.PullUndecided:
		*undecided = true
	}
	return true
}

// brepFilletUndercuts folds DF7's reading of band's pipe patches into the
// running faces list and undecided flag (loop-fillet Table DF's DF7). Patch
// p of the band is read through the closed-form range of cos φ·A + sin φ·B(θ)
// of its outward normal along the pull: A from F's frame normal, B from the
// inward normal of F's region at the patch's walk (survey2d.InwardRange) or
// through the turn of a reflex corner (survey2d.CornerRange), decided by the
// three-valued rule every undercut reading uses. The patch tags are the built
// surfaces (their stamped departure is zero), so no allowance beyond that
// arithmetic arises. It reports false, a total refusal, when a patch's face is
// missing or a range cannot be enclosed.
func brepFilletUndercuts(budget *proofbound.WorkBudget, roles map[string]*Face, bp brepPayload, band brepLoopBand, pull r3.Vec,
	faces *[]*Face, undecided *bool, work *freeform.FreeformWork) bool {
	f := bp.Faces[band.Face]
	view := f.View(bp.Xform)
	m, ok := survey2d.NewPlacedFrameMap(view.Frame, view.Xform)
	if !ok {
		return false
	}
	fr, err := breprecord.FilletLoopOf(budget, band.Orig, band.Setback.Dc, f.Role, work)
	if err != nil {
		return false
	}
	sign := int64(-1)
	if f.Outward {
		sign = 1
	}
	sigma := int64(band.Sigma)
	n := len(fr.Walks)
	p := 0
	patch := func(rng survey2d.PullBracket, ok bool) bool {
		if !ok {
			return false
		}
		face := roles[band.PatchRole(p)]
		p++
		if face == nil {
			return false
		}
		verdict, ok := survey2d.PatchPullVerdict(m, pull, sign, rng)
		return listVerdict(faces, undecided, face, verdict, ok)
	}
	for i, w := range fr.Walks {
		if !patch(survey2d.InwardRange(w, m, pull, sigma)) {
			return false
		}
		if ni := (i + 1) % n; !fr.Loop.WholeTurn() && fr.Loop.Corners[ni] == filletband.Reflex {
			if !patch(survey2d.CornerRange(w, fr.Walks[ni], m, pull, sigma)) {
				return false
			}
		}
	}
	return p == len(fr.Pieces)
}
