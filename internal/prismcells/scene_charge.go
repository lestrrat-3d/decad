package prismcells

import (
	"context"
	"errors"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
)

// SceneDelta records each operand's largest walked-endpoint charge and the
// additional displacement caused by arranged crossings and shared spans.
// The caller composes A and B with their own section displacements; B also
// includes the coordinate map's charge before either crossing is considered.
type SceneDelta struct {
	A, B      float64
	Crossing  float64
	Amplified bool

	shared          CoincidentReading
	sharedWidth     float64
	sharedDisplaced bool
}

// Incoming returns the two displacements carried into the arrangement.
func (d SceneDelta) Incoming(sectionA, sectionB, reexpression float64) (float64, float64) {
	return proofbound.AbsSumUpper(sectionA, d.A), proofbound.AbsSumUpper(sectionB, d.B, reexpression)
}

// ChargeCrossings bounds every cut that can move when an input is displaced.
// It also charges the gap across a coincident span that the result keeps as
// boundary. A pair whose charge cannot be proved returns ok=false for the
// caller to route through the mesh path; a non-nil error is cancellation.
func (d *SceneDelta) ChargeCrossings(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin,
	profiles []*sketch.Profile, sectionA, sectionB, reexpression float64) (bool, error) {
	inA, inB := d.Incoming(sectionA, sectionB, reexpression)
	coincident, ok, err := CoincidentEdges(budget, tags, profiles)
	if err != nil || !ok {
		return false, err
	}
	d.shared = coincident
	if len(coincident.Spans) > 0 {
		d.sharedWidth = proofbound.AbsSumUpper(coincident.Gap(), inA, inB)
		d.sharedDisplaced = inA > 0 || inB > 0
		d.Crossing = max(d.Crossing, d.sharedWidth)
	}
	if inA == 0 && inB == 0 {
		return true, nil
	}
	split, err := HasSplitBoundary(budget, profiles)
	if err != nil {
		return false, err
	}
	if !split {
		return true, nil
	}
	d.Amplified = true
	crossing, ok, err := CrossingCharge(budget, tags, profiles, inA, inB)
	if err != nil || !ok {
		return false, err
	}
	d.Crossing = max(d.Crossing, crossing)
	return true, nil
}

// SharedSpansBounded reports whether every coincident span can be covered by
// the result's boundary displacement. A displaced span inside or outside the
// selected region could leave an unrecorded sliver, so it takes the mesh path.
func (d SceneDelta) SharedSpansBounded(budget *proofbound.WorkBudget, selected []*sketch.Profile) (bool, error) {
	if !d.sharedDisplaced {
		return true, nil
	}
	return d.shared.OnBoundary(budget, selected)
}

// Merged composes both inputs, the crossing charge, and the cut parameter's
// own rounding into the merged section displacement.
func (d SceneDelta) Merged(sectionA, sectionB, reexpression, cutDelta float64) float64 {
	inA, inB := d.Incoming(sectionA, sectionB, reexpression)
	return proofbound.AbsSumUpper(max(inA, inB, d.Crossing), cutDelta)
}

// AmplifiedFallback routes an analytic error after a displaced crossing to
// the mesh path. Cancellation always propagates to the caller.
func AmplifiedFallback(amplified bool, err error) (bool, error) {
	if err == nil {
		return false, nil
	}
	if !amplified || errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
		return false, err
	}
	return true, nil
}
