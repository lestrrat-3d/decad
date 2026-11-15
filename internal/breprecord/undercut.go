package breprecord

import (
	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// ListVerdict folds one face's verdict into the running faces list and
// undecided flag used by the BRep undercut readers. ok=false (a
// non-finite input or a failed enclosure) propagates as a total refusal to
// the caller, matching every other refusal path these surveys already take.
func ListVerdict(faces *[]*topo.Face, undecided *bool, f *topo.Face, verdict survey2d.PullVerdict, ok bool) bool {
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

// FilletUndercuts folds DF7's reading of band's pipe patches into the
// running faces list and undecided flag (loop-fillet Table DF's DF7). Patch
// p of the band is read through the closed-form range of cos φ·A + sin φ·B(θ)
// of its outward normal along the pull: A from F's frame normal, B from the
// inward normal of F's region at the patch's walk (survey2d.InwardRange) or
// through the turn of a reflex corner (survey2d.CornerRange), decided by the
// three-valued rule every undercut reading uses. The patch tags are the built
// surfaces (their stamped departure is zero), so no allowance beyond that
// arithmetic arises, except beside an LF5′ corner or a radial step: there the
// true band's normals stray from the patches' by at most
// filletband.Loop.NormalTurn, and each patch's bracket widens by it
// (loop-fillet §5.5, §5.7). It reports false, a total refusal, when a patch's face is
// missing or a range cannot be enclosed.
func FilletUndercuts(budget *proofbound.WorkBudget, roles map[string]*topo.Face, bp Payload, band LoopBand, pull r3.Vec,
	faces *[]*topo.Face, undecided *bool, work *freeform.FreeformWork) bool {
	f := bp.Faces[band.Face]
	view := f.View(bp.Xform)
	m, ok := survey2d.NewPlacedFrameMap(view.Frame, view.Xform)
	if !ok {
		return false
	}
	fr, err := FilletLoopOf(budget, band.Orig, band.Setback.Dc, band.Setback.DcDelta, f.Role, work)
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
		return ListVerdict(faces, undecided, face, verdict, ok)
	}
	rIv, err := filletband.RadiusInterval(band.Setback.Dc, band.Setback.DcDelta)
	if err != nil {
		return false
	}
	for i, w := range fr.Walks {
		rng, ok := survey2d.InwardRange(w, m, pull, sigma)
		if ok && !fr.Loop.WholeTurn() {
			for _, k := range [2]int{i, (i + 1) % n} {
				turn, okT := fr.Loop.NormalTurn(fr.Walks, k, rIv)
				if !okT {
					return false
				}
				if turn.Sign() == 0 {
					continue
				}
				if rng, ok = survey2d.WidenPullBracket(rng, m, pull, turn); !ok {
					return false
				}
			}
		}
		if !patch(rng, ok) {
			return false
		}
		if ni := (i + 1) % n; !fr.Loop.WholeTurn() && fr.Loop.Corners[ni] == filletband.Reflex {
			rng, ok := survey2d.CornerRange(w, fr.Walks[ni], m, pull, sigma)
			turn, okT := fr.Loop.NormalTurn(fr.Walks, ni, rIv)
			if !okT {
				return false
			}
			if ok && turn.Sign() > 0 {
				rng, ok = survey2d.WidenPullBracket(rng, m, pull, turn)
			}
			if !patch(rng, ok) {
				return false
			}
		}
	}
	return p == len(fr.Pieces)
}
