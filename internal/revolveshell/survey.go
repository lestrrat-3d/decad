package revolveshell

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/shellsurvey"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/wallsurvey"
)

// Inradius is S10's reading of the effective meridian. A meridian
// off the axis is its own effective meridian, read as a prism section is. One
// with an on-axis walk is read united with its mirror image across the axis,
// in axis coordinates (z, ρ) where the mirror is ρ ↦ −ρ exactly: a solid
// cylinder of radius R keeps a cavity for every thickness below min(R, H/2),
// not merely below the recorded half-section's own inradius.
func Inradius(budget *proofbound.WorkBudget, profile momentinput.Profile, ax revolveaxis.Frame, onAxis bool, tmm, tDelta, tol float64) (float64, bool, error) {
	if !onAxis {
		return shellsurvey.SectionInradius(budget, boundarywalk.Profile(profile), tmm, tDelta, tol)
	}
	loops, err := wallsurvey.RevolveLoops(budget, profile, ax)
	if err != nil {
		if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
			return 0, false, err
		}
		return 0, false, fmt.Errorf(`%w: this evaluator cannot read the shell meridian: %v`, decaderr.ErrUnsupported, err)
	}
	loop := loops[0]
	n := len(loop)
	var elems []survey2d.SurveyElem
	var verts [][2]float64
	for i, w := range loop {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return 0, false, err
		}
		if ax.IsAxis(w.SegmentWalk) {
			continue
		}
		el, ok := survey2d.WalkElem(w.SegmentWalk)
		if !ok {
			return 0, false, fmt.Errorf(`%w: this evaluator cannot survey the shell meridian's curve type`, decaderr.ErrUnsupported)
		}
		elems = append(elems, el, survey2d.MirrorElem(el))
		ends := [][2]float64{{w.StartU, w.StartV}}
		if ax.IsAxis(loop[(i+1)%n].SegmentWalk) {
			// The walk's end on the axis is a corner of the mirror union too.
			ends = append(ends, [2]float64{w.EndU, w.EndV})
		}
		for _, p := range ends {
			verts = append(verts, p)
			if p[1] > 0 {
				verts = append(verts, [2]float64{p[0], -p[1]})
			}
		}
	}
	if err := shellsurvey.RequireWallSurveyWork(budget, len(elems), len(verts)); err != nil {
		return 0, false, err
	}
	inradius, err := shellsurvey.WallSurveyInradius(budget, elems, verts)
	return inradius, false, err
}

// RemovedWalks maps the removed side faces' segments onto the
// meridian's coalesced walks and checks reach §9.2's run rule: the removed
// walks are one proper connected run, and what remains is one kept chain. On
// a meridian with an on-axis walk the kept chain runs from one axis end, so
// the run must touch the axis walk at one end of the chain: a run between two
// kept pieces would leave two wall regions, which one revolve record does not
// hold. A walk the faces cover only in part is a coalescing this evaluator
// does not match. Each refusal is SX8, decaderr.ErrUnsupported.
func RemovedWalks(walks []survey2d.SideWalk, axisAt int, segs map[int]struct{}) (map[int]struct{}, error) {
	n := len(walks)
	removed := map[int]struct{}{}
	for i, w := range walks {
		hit := 0
		for _, si := range w.Segs {
			if _, ok := segs[si]; ok {
				hit++
			}
		}
		switch {
		case hit == 0:
		case hit == len(w.Segs) && i != axisAt:
			removed[i] = struct{}{}
		default:
			return nil, fmt.Errorf(`%w: a removed side face covers only part of a meridian walk (modify-reach SX8)`, decaderr.ErrUnsupported)
		}
	}
	sides := n
	if axisAt >= 0 {
		sides--
	}
	if len(removed) == 0 || len(removed) >= sides {
		return nil, fmt.Errorf(`%w: a side opening must remove a proper run of the revolve's generated side faces, not all of them (modify-reach SX8)`, decaderr.ErrUnsupported)
	}
	// Count the run's boundaries around the cycle of side walks (the axis
	// walk is not a side walk and closes the cycle).
	var order []int
	for k := 1; k <= n; k++ {
		i := (max(axisAt, 0) + k) % n
		if i != axisAt {
			order = append(order, i)
		}
	}
	in := func(k int) bool { _, ok := removed[order[k]]; return ok }
	runs := 0
	for k := range order {
		if in(k) && (k == 0 && axisAt >= 0 || !in((k+len(order)-1)%len(order))) {
			runs++
		}
	}
	if runs != 1 {
		return nil, fmt.Errorf(`%w: a side opening's removed faces must be one connected run (modify-reach SX8)`, decaderr.ErrUnsupported)
	}
	if axisAt >= 0 && !in(0) && !in(len(order)-1) {
		return nil, fmt.Errorf(`%w: a side opening between two kept walks of a meridian on the axis leaves two wall regions, which one revolve does not hold (modify-reach SX8)`, decaderr.ErrUnsupported)
	}
	return removed, nil
}
