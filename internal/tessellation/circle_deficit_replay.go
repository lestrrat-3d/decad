package tessellation

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/units"
)

// CircleScheduleReplay checks a recorded full-circle walk and selects a
// chord count and area deficits under one budget. The caller must separately
// authenticate the source record, its walk and the placement.
func CircleScheduleReplay(ctx context.Context, seg sectionrecord.CircleSeg, walk survey2d.SegmentWalk,
	chord, height float64, budget *proofbound.ReplayBudget) (int, float64, float64, float64, error) {
	if budget == nil {
		return 0, 0, 0, 0, fmt.Errorf("%w: missing circular schedule budget", decaderr.ErrUnsupported)
	}
	if err := budget.SpendWork(ctx, 128); err != nil {
		return 0, 0, 0, 0, err
	}
	if err := budget.ReserveRecords(ctx, 1); err != nil {
		return 0, 0, 0, 0, err
	}
	if err := budget.ReserveBytes(ctx, 256); err != nil {
		return 0, 0, 0, 0, err
	}
	if seg.Radius.Kind() != units.Length || proofbound.IsNonFinite(seg.Radius.Base()) {
		return 0, 0, 0, 0, fmt.Errorf("%w: invalid circle radius", decaderr.ErrUnsupported)
	}
	radius, err := seg.Radius.In(units.Millimeter)
	if err != nil || !seg.CCW || seg.TStart != 0 || seg.TEnd != 1 || walk.Kind != survey2d.WalkCircular || !walk.Closed ||
		walk.Th0 != 0 || walk.Th1 != 2*math.Pi || math.Float64bits(walk.CU) != math.Float64bits(seg.Center.U) ||
		math.Float64bits(walk.CV) != math.Float64bits(seg.Center.V) || math.Float64bits(walk.Radius) != math.Float64bits(radius) ||
		!(radius >= math.Ldexp(1, -100)) || radius > math.Ldexp(1, 100) || !(chord > 0) || proofbound.IsNonFinite(chord) {
		return 0, 0, 0, 0, fmt.Errorf("%w: unsupported recorded circle", decaderr.ErrUnsupported)
	}
	sweep := proofbound.TwoPiUpper()
	if err := budget.SpendWork(ctx, 64); err != nil {
		return 0, 0, 0, 0, err
	}
	sag := ChordSagitta(radius, sweep, 3)
	count := 3
	if sag > chord {
		maxAngle := math.Pi
		if chord < radius {
			maxAngle = 4 * math.Asin(math.Sqrt(chord/(2*radius)))
		}
		if maxAngle == 0 || sweep/maxAngle > freeform.MaxChordsPerWalk {
			return 0, 0, 0, 0, freeform.ErrTooManyChords
		}
		count = max(int(math.Ceil(sweep/maxAngle)), 3)
		if count > freeform.MaxChordsPerWalk {
			return 0, 0, 0, 0, freeform.ErrTooManyChords
		}
		for {
			if err := budget.SpendWork(ctx, 64); err != nil {
				return 0, 0, 0, 0, err
			}
			sag = ChordSagitta(radius, sweep, count)
			if sag <= chord {
				break
			}
			if count == freeform.MaxChordsPerWalk {
				return 0, 0, 0, 0, freeform.ErrTooManyChords
			}
			count++
		}
	}
	capArea, wallArea, err := CircleDeficitReplay(ctx, radius, count, height, budget)
	if err != nil {
		return 0, 0, 0, 0, err
	}
	return count, sag, capArea, wallArea, nil
}

// CircleDeficitReplay bounds the area omitted by a full-circle polygon's cap
// and wall. It uses x-sin(x) <= x³/6 and a rounded-up height. The radius and
// height are in millimetres; the returned values are square millimetres.
func CircleDeficitReplay(ctx context.Context, radius float64, chords int, height float64, budget *proofbound.ReplayBudget) (float64, float64, error) {
	if budget == nil {
		return 0, 0, fmt.Errorf("%w: missing circular deficit budget", decaderr.ErrUnsupported)
	}
	if err := budget.SpendWork(ctx, 64); err != nil {
		return 0, 0, err
	}
	if err := budget.ReserveRecords(ctx, 1); err != nil {
		return 0, 0, err
	}
	if err := budget.ReserveBytes(ctx, 256); err != nil {
		return 0, 0, err
	}
	if chords < 3 || chords > freeform.MaxChordsPerWalk || !(radius >= math.Ldexp(1, -100)) || radius > math.Ldexp(1, 100) || proofbound.IsNonFinite(radius) || proofbound.IsNonFinite(height) {
		return 0, 0, fmt.Errorf("%w: circular deficit is outside its bounded range", decaderr.ErrUnsupported)
	}
	theta := proofbound.TwoPiUpper()
	cube := proofbound.ProductUpper(theta, proofbound.ProductUpper(theta, theta))
	// chords <= 2^14, so 12*chords² and 24*chords² are exact binary64 integers.
	square := float64(chords) * float64(chords)
	capArea := proofbound.DivUpper(proofbound.ProductUpper(proofbound.ProductUpper(radius, radius), cube), 12*square)
	h := math.Abs(height)
	if h != 0 {
		h = math.Nextafter(h, math.Inf(1))
	}
	wallArea := proofbound.ProductUpper(proofbound.DivUpper(proofbound.ProductUpper(radius, cube), 24*square), h)
	if !(capArea > 0) || (height != 0 && !(wallArea > 0)) || proofbound.IsNonFinite(capArea) || proofbound.IsNonFinite(wallArea) {
		return 0, 0, fmt.Errorf("%w: circular deficit cannot be bounded", decaderr.ErrUnsupported)
	}
	return capArea, wallArea, nil
}
