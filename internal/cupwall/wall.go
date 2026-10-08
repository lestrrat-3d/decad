package cupwall

import (
	"context"
	"errors"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// Input is the cup morphology and thickness read by the wall theorem.
type Input struct {
	Outer, Cavity       momentinput.Profile
	ZOpen, ZOuter, ZCav float64
	Thickness           float64
	ThicknessDelta      float64
	Inward, Outward     bool
}

// Operations connects the theorem to the recorded section's offset and audit.
// Each function consumes the same work budget as the theorem.
type Operations struct {
	Offset  func(*proofbound.WorkBudget, momentinput.Profile, float64, float64) (momentinput.Profile, error)
	Equal   func(*proofbound.WorkBudget, momentinput.Profile, momentinput.Profile) (bool, error)
	Audit   func(*proofbound.WorkBudget, momentinput.Profile, momentinput.Profile) error
	Reverse func(*proofbound.WorkBudget, sectionrecord.LoopRecord) (sectionrecord.LoopRecord, error)
}

// Outcome is a certified wall reading or an undecided morphology.
type Outcome struct {
	Reading *float64
	Bound   float64
	OK      bool
}

// Evaluate rechecks the cup's offset morphology before reading its wall.
// A material junction inside alpha returns exact zero; otherwise the wall is
// the held thickness with its millimetre-conversion displacement.
func Evaluate(budget *proofbound.WorkBudget, cup Input, alpha float64, ops Operations) (Outcome, error) {
	finite := func(v float64) bool { return !math.IsNaN(v) && !math.IsInf(v, 0) }
	isCancellation := func(err error) bool {
		return errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded)
	}
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return Outcome{}, err
	}
	t := cup.Thickness
	if !finite(t) || t <= 0 || cup.Inward == cup.Outward {
		return Outcome{}, nil
	}
	dOuter := cup.ZOpen - cup.ZOuter
	dCavity := cup.ZOpen - cup.ZCav
	if !finite(dOuter) || !finite(dCavity) || dOuter == 0 || dCavity == 0 ||
		math.Signbit(dOuter) != math.Signbit(dCavity) || math.Abs(dCavity) >= math.Abs(dOuter) {
		return Outcome{}, nil
	}
	openDir := 1.0
	if dOuter < 0 {
		openDir = -1
	}
	if cup.Inward && cup.ZCav != cup.ZOuter+openDir*t {
		return Outcome{}, nil
	}
	if cup.Outward && cup.ZOuter != cup.ZCav-openDir*t {
		return Outcome{}, nil
	}

	oLoops := append([]sectionrecord.LoopRecord{cup.Outer.Outer}, cup.Outer.Holes...)
	cLoops := append([]sectionrecord.LoopRecord{cup.Cavity.Outer}, cup.Cavity.Holes...)
	if len(oLoops) != len(cLoops) {
		return Outcome{}, nil
	}
	if oi, err := cup.Outer.IntegralsBudget(budget); err != nil {
		if isCancellation(err) {
			return Outcome{}, err
		}
		return Outcome{}, nil
	} else if oi.Area <= 0 || !finite(oi.Area) {
		return Outcome{}, nil
	}
	if ci, err := cup.Cavity.IntegralsBudget(budget); err != nil {
		if isCancellation(err) {
			return Outcome{}, err
		}
		return Outcome{}, nil
	} else if ci.Area <= 0 || !finite(ci.Area) {
		return Outcome{}, nil
	}

	// Structural equality, rather than a residual, is the offset claim.
	offsetMatches := func(orig, want momentinput.Profile, sense float64) (bool, error) {
		got, err := ops.Offset(budget, orig, sense, t)
		if err != nil {
			if isCancellation(err) {
				return false, err
			}
			return false, nil
		}
		same, err := ops.Equal(budget, got, want)
		if err != nil {
			if isCancellation(err) {
				return false, err
			}
			return false, nil
		}
		if !same {
			return false, nil
		}
		if err := ops.Audit(budget, orig, got); err != nil {
			if isCancellation(err) {
				return false, err
			}
			return false, nil
		}
		return true, nil
	}
	matches, err := offsetMatches(cup.Outer, cup.Cavity, 1)
	if err != nil {
		return Outcome{}, err
	}
	if cup.Outward {
		matches, err = offsetMatches(cup.Cavity, cup.Outer, -1)
		if err != nil {
			return Outcome{}, err
		}
	}
	if !matches {
		return Outcome{}, nil
	}

	hasPinch := func(loops [][]survey2d.SideWalk) (bool, bool, error) {
		for _, loop := range loops {
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return false, false, err
			}
			if len(loop) == 0 {
				return false, false, nil
			}
			if len(loop) == 1 && loop[0].Closed {
				continue
			}
			for i, walk := range loop {
				if err := survey2d.WallBudgetStep(budget); err != nil {
					return false, false, err
				}
				prev := loop[(i+len(loop)-1)%len(loop)]
				if survey2d.JunctionPinch(prev.TanOutU, prev.TanOutV, walk.TanInU, walk.TanInV, alpha) {
					return true, true, nil
				}
			}
		}
		return false, true, nil
	}

	outerWalks, err := boundarywalk.SurveyLoopsBudget(budget, boundarywalk.Profile(cup.Outer))
	if err != nil {
		return Outcome{}, err
	}
	pinch, ok, err := hasPinch(outerWalks)
	if err != nil {
		return Outcome{}, err
	}
	if !ok {
		return Outcome{}, nil
	}
	if pinch {
		zero := 0.0
		return Outcome{Reading: &zero, OK: true}, nil
	}

	// The cavity boundary is a void skin; reverse its recorded loops to
	// restore the material-left convention of JunctionPinch.
	var cavityWalks [][]survey2d.SideWalk
	for _, loop := range cLoops {
		reversed, err := ops.Reverse(budget, loop)
		if err != nil {
			return Outcome{}, err
		}
		walks, err := boundarywalk.SurveyLoopsBudget(budget, boundarywalk.Profile{Outer: reversed})
		if err != nil {
			return Outcome{}, err
		}
		cavityWalks = append(cavityWalks, walks[0])
	}
	pinch, ok, err = hasPinch(cavityWalks)
	if err != nil {
		return Outcome{}, err
	}
	if !ok {
		return Outcome{}, nil
	}
	if pinch {
		zero := 0.0
		return Outcome{Reading: &zero, OK: true}, nil
	}

	return Outcome{Reading: &t, Bound: cup.ThicknessDelta, OK: true}, nil
}
