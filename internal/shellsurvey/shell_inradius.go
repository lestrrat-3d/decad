package shellsurvey

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/units"
)

// shellInradiusWorkLimit is S18's hard ceiling over one inward shell's
// candidate generation and whole-boundary validation. Candidate-family visits
// are checked against it before the wall kernel starts; every generated and
// validation visit then charges the same counter.
const shellInradiusWorkLimit uint64 = 1 << 20

// SectionInradius proves the requested thickness fits, or returns the largest
// inscribed disk of a recorded section from internal/survey2d/wall_kernel.go
// (docs/modify-design.md §8, the reading that answers Wall.Minimum). S18
// checks the candidate-family count before entering the kernel and shares one
// fixed work budget across its streamed generation and validation. An
// undecided or over-budget build-time gate is ErrUnsupported: it has no
// Suspect result to fall back on.
//
// The kernel publishes that inradius as an interval (survey2d.WallSurveyOut), and this
// gate reads its midpoint: the caller's own accept boundary already sits a
// scale-relative tol below the limit — 1e-9 of the section's own size,
// decades above the aggregate's own half-width — so the interval cannot reach
// across a decision the margin has not already made.
func SectionInradius(budget *proofbound.WorkBudget, profile boundarywalk.Profile, thickness, thicknessDelta, tol float64) (float64, bool, error) {
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return 0, false, err
	}
	loops, err := boundarywalk.SurveyLoopsBudget(budget, boundarywalk.Profile(profile))
	if err != nil {
		if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
			return 0, false, err
		}
		return 0, false, fmt.Errorf(`%w: this evaluator cannot read the shell section: %v`, decaderr.ErrUnsupported, err)
	}
	var elems []survey2d.SurveyElem
	var verts [][2]float64
	for _, loop := range loops {
		single := len(loop) == 1 && loop[0].Closed
		for _, w := range loop {
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return 0, false, err
			}
			el, ok := survey2d.WalkElem(w.SegmentWalk)
			if !ok {
				return 0, false, fmt.Errorf(`%w: this evaluator cannot survey the shell section's curve type`, decaderr.ErrUnsupported)
			}
			elems = append(elems, el)
			if single {
				continue
			}
			verts = append(verts, [2]float64{w.StartU, w.StartV})
		}
	}
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return 0, false, err
	}
	if err := RequireWallSurveyWork(budget, len(elems), len(verts)); err != nil {
		return 0, false, err
	}
	// A contained disk can prove only the success side of S10. Failure and all
	// diagnostics still use the full inradius survey. The S18 count above runs
	// first even when this shorter proof succeeds.
	enough, err := RectangleCircleWitness(budget, profile, loops, thickness, thicknessDelta, tol)
	if err != nil {
		return 0, false, err
	}
	if enough {
		return 0, true, nil
	}
	inradius, err := WallSurveyInradius(budget, elems, verts)
	return inradius, false, err
}

// RequireWallSurveyWork is S18's preflight: the inward section survey's
// candidate-family visits, counted under checked arithmetic before the wall
// kernel starts, must stay within shellInradiusWorkLimit.
func RequireWallSurveyWork(budget *proofbound.WorkBudget, elems, verts int) error {
	candidateWork, ok := proofbound.WallCandidateWork(elems, verts, false)
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return err
	}
	if !ok {
		return fmt.Errorf(`%w: inward shell section survey candidate count overflows the checked work counter (fixed work budget %d)`, decaderr.ErrUnsupported, shellInradiusWorkLimit)
	}
	if candidateWork > shellInradiusWorkLimit {
		return fmt.Errorf(`%w: inward shell section survey needs %d candidate-family visits, above the fixed work budget of %d`, decaderr.ErrUnsupported, candidateWork, shellInradiusWorkLimit)
	}
	return nil
}

// WallSurveyInradius runs internal/survey2d/wall_kernel.go over a section's
// survey elements and returns its inradius, charging generation and validation
// to S18's one shared work budget. An over-budget or undecided survey is
// ErrUnsupported: a build-time gate has no Suspect result to fall back on.
func WallSurveyInradius(budget *proofbound.WorkBudget, elems []survey2d.SurveyElem, verts [][2]float64) (float64, error) {
	// fitMax is +Inf: the inradius is a property of the section alone, with no
	// height constraint (that constraint only bears on spanning, not the
	// largest inscribed disk).
	k, err := survey2d.NewWallKernelBudget(budget, elems, nil, verts, 0, proofbound.ExactScalar(0), false, math.Inf(1))
	if err != nil {
		return 0, err
	}
	out, err := k.RunBudget(proofbound.NewWallWorkBudgetWithOperation(shellInradiusWorkLimit, budget))
	if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
		return 0, err
	}
	if errors.Is(err, proofbound.ErrWallWorkBudget) {
		return 0, fmt.Errorf(`%w: inward shell section survey exceeded the fixed work budget of %d during candidate generation or validation`, decaderr.ErrUnsupported, shellInradiusWorkLimit)
	}
	if err != nil {
		return 0, fmt.Errorf(`%w: inward shell section survey failed: %v`, decaderr.ErrUnsupported, err)
	}
	if !out.Ok {
		return 0, fmt.Errorf(`%w: this evaluator cannot prove the eroded section non-empty`, decaderr.ErrUnsupported)
	}
	return out.Inradius, nil
}

// RectangleCircleWitness passes the shell's recorded holes and unit conversion
// to the exact rectangular-section witness.
func RectangleCircleWitness(budget *proofbound.WorkBudget, profile boundarywalk.Profile, loops [][]survey2d.SideWalk, thickness, thicknessDelta, tol float64) (bool, error) {
	return survey2d.RectangleCircleWitness(budget, profile.Holes, loops, thickness, thicknessDelta, tol,
		func(radius units.Value) (float64, float64, error) {
			return extent.MagnitudeInBounded(radius, units.Length, units.Millimeter, "the hole radius")
		})
}
