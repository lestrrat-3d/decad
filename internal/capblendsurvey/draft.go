package capblendsurvey

import (
	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// This file is a draft body's undercut survey (docs/draft-design.md Table DD
// row DD7). Every wall of a draft body is a patch of its band view
// (draftPayload.Band), so the survey reads each wall the way DX7 reads a
// chamfer patch (PatchUndercuts, undercut.go): the wall's own
// Face.NormalAt, enclosed over its window by PatchNormalRange, then decided
// by survey2d.DecidePull at the allowance that reading carries. The allowance
// is never zero on a draft wall, because the taper's tangent is an enclosure
// of positive width (§8), so a wall is listed only where every point of it
// provably opposes the pull and cleared only where every point provably does
// not. The two caps are planes the frame alone orients, read exactly by
// survey2d.CapNormalDecision as a prism's are.
//
// A draft body has no unchanged receiver wall, so CapBlendUndercuts' receiver
// loop has nothing to read here: it would read a drafted wall as the vertical
// wall of the recorded section.

// DraftUndercuts surveys a draft body's walls and caps against the pull. A
// straddling wall sets undecided without discarding a face already proven to
// oppose, and an entirely undecided survey keeps a nil listing, distinct from
// the empty proven all-clear (docs/verification-design.md §6).
func DraftUndercuts(b *topo.Body, dp draftrecord.Payload, pull r3.Vec) reportvocab.UndercutSurvey[*topo.Face] {
	p, ok := pull.Normalize()
	if !ok {
		return undercutOutcome{}
	}
	pl := dp.Band().PrismLike(0, 0)
	m, okM := survey2d.NewPlacedFrameMap(pl.Frame, pl.Xform)
	if !okM {
		return undercutOutcome{}
	}
	roles := topobuild.FacesByRole(b)
	faces := []*topo.Face{}
	undecided := false
	if !PatchUndercuts(roles, pl, dp.Patches, p, &faces, &undecided) {
		return undercutOutcome{}
	}
	for _, cap := range []struct {
		role string
		sign float64
	}{{role: provenance.RoleCapStart, sign: -1}, {role: provenance.RoleCapEnd, sign: 1}} {
		f := roles[cap.role]
		if f == nil {
			return undercutOutcome{}
		}
		verdict, ok := survey2d.CapNormalDecision(m, pull, cap.sign)
		if !listVerdict(&faces, &undecided, f, verdict, ok) {
			return undercutOutcome{}
		}
	}
	if undecided && len(faces) == 0 {
		faces = nil
	}
	return undercutOutcome{Faces: faces, OK: true, Undecided: undecided}
}
