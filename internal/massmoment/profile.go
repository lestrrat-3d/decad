package massmoment

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// PrismSectionMoments reads a profile's area, first and second moments as
// rational intervals. Exact fields stay exact; other fields carry their
// individual published bounds.
func PrismSectionMoments(ctx context.Context, profile momentinput.Profile) ([6]proofbound.RatInterval, error) {
	ig, err := profile.EvaluatorIntegralsContext(ctx, freeform.MomentSecondOrder, nil)
	if err != nil {
		return [6]proofbound.RatInterval{}, err
	}
	section, err := SectionIntervals(SectionInputsOf(ig))
	if err != nil {
		return [6]proofbound.RatInterval{}, err
	}
	return section, nil
}

// PrismProfileMoments integrates the section over its axial interval about
// the recorded mid level. Section and endpoint displacements widen its
// occupied volume and every first and second moment.
func PrismProfileMoments(ctx context.Context, profile momentinput.Profile, z0, z1,
	sectionDelta, z0Delta, z1Delta float64) (Moments, error) {
	if !NonNegativeFinite(sectionDelta) || !NonNegativeFinite(z0Delta) || !NonNegativeFinite(z1Delta) {
		return Moments{}, fmt.Errorf("%w: prism displacement has no finite bound", decaderr.ErrUnsupported)
	}
	section, err := PrismSectionMoments(ctx, profile)
	if err != nil {
		return Moments{}, err
	}
	return PrismVolumeMoments(ctx, section, z0, z1,
		sectionDelta > 0 || z0Delta > 0 || z1Delta > 0,
		func(area proofbound.RatInterval, h *big.Rat) (*big.Rat, *big.Rat, error) {
			return prismProfileOccupiedError(ctx, profile, area, h, sectionDelta, z0Delta, z1Delta)
		})
}

// prismProfileOccupiedError bounds the symmetric difference between the
// recorded prism and the one its construction denotes, and a coordinate
// radius R about its mid level. The section tube covers changed boundaries;
// the two end slabs cover changed levels.
func prismProfileOccupiedError(ctx context.Context, profile momentinput.Profile,
	area proofbound.RatInterval, h *big.Rat, sectionDelta, z0Delta, z1Delta float64) (*big.Rat, *big.Rat, error) {
	work := freeform.NewFreeformWork()
	walks, err := momentinput.ResolveProfileWalks(profile, work)
	if err != nil {
		return nil, nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, nil, err
	}
	count := 0
	perimeter := 0.0
	for _, loop := range append([][]survey2d.SegmentWalk{walks.Outer}, walks.Holes...) {
		for _, w := range loop {
			count++
			perimeter = proofbound.AbsSumUpper(perimeter, w.Length, w.LengthBound)
		}
	}
	coordUpper, err := momentinput.CoordinateEnvelope(profile, work, walks)
	if err != nil {
		return nil, nil, err
	}
	return PrismOccupiedError(area, h, sectionDelta, z0Delta, z1Delta, count, perimeter, coordUpper)
}

// RevolveSectionMoments reads a profile's plane-origin moments through third
// order as rational enclosures. The first six retain their exact values when
// the section evaluator recorded them.
func RevolveSectionMoments(ctx context.Context, profile momentinput.Profile) ([4][4]proofbound.RatInterval, error) {
	ig, err := profile.EvaluatorIntegralsContext(ctx, freeform.MomentThirdOrder, nil)
	if err != nil {
		return [4][4]proofbound.RatInterval{}, err
	}
	var m [4][4]proofbound.RatInterval
	slots := [6]*proofbound.RatInterval{&m[0][0], &m[1][0], &m[0][1], &m[2][0], &m[1][1], &m[0][2]}
	section, err := SectionIntervals(SectionInputsOf(ig))
	for i, value := range section {
		*slots[i] = value
	}
	if err != nil {
		return m, err
	}
	third, ok := ig.ThirdMoments()
	if !ok {
		return m, fmt.Errorf("%w: revolve section has no third-order moment enclosure", decaderr.ErrUnsupported)
	}
	m[3][0], m[2][1], m[1][2], m[0][3] = third[0], third[1], third[2], third[3]
	return m, nil
}

// RevolveSweepFactors reads the sweep's certified angular width and endpoint
// sine and cosine intervals.
func RevolveSweepFactors(den revolveangle.Sweep, phi0, phi1 float64) (RevolveAngular, bool) {
	width, ok := den.WidthInterval()
	if !ok {
		return RevolveAngular{}, false
	}
	s0, c0, ok0 := den.Phi0.SinCosFor(phi0)
	s1, c1, ok1 := den.Phi1.SinCosFor(phi1)
	if !ok0 || !ok1 {
		return RevolveAngular{}, false
	}
	return AngularFactors(width, s0, c0, s1, c1), true
}
