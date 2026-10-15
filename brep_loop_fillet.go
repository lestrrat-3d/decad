package decad

import (
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
)

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
