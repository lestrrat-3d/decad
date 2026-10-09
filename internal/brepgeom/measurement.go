package brepgeom

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// Region is a planar face's area enclosure, published area and displacement.
type Region struct {
	Area         proofbound.RatInterval
	Published    proofbound.BoundedScalar
	Upper        float64
	Displacement float64
}

// RegionOf integrates a planar face's region and charges its section
// displacement against the published area.
func RegionOf(ctx context.Context, profile *momentinput.Profile, delta float64,
	walks [][]survey2d.SegmentWalk) (Region, error) {
	ig, err := profile.EvaluatorIntegralsContext(ctx, freeform.MomentFirstOrder, freeform.NewFreeformWork())
	if err != nil {
		return Region{}, err
	}
	area, err := Enclosure(ig.Area, ig.AreaBound, ig.ExactArea())
	if err != nil {
		return Region{}, err
	}
	perimeter := proofbound.BoundedScalar{}
	count := 0
	for _, loop := range walks {
		for _, w := range loop {
			perimeter = proofbound.BoundedAdd(perimeter, proofbound.MeasuredScalar(w.Length,
				proofbound.AbsSumUpper(w.LengthBound, proofbound.SectionDisplacementLength(delta, 1))))
			count++
		}
	}
	displacement := proofbound.SectionDisplacementArea(delta, count,
		proofbound.AbsSumUpper(perimeter.Value, perimeter.Bound))
	return Region{
		Area: area, Published: proofbound.MeasuredScalar(ig.Area, proofbound.AbsSumUpper(ig.AreaBound, displacement)),
		Displacement: displacement, Upper: proofbound.AbsSumUpper(ig.Area, ig.AreaBound),
	}, nil
}

// RestoredRegion resolves a restored planar contour before integrating it.
func RestoredRegion(ctx context.Context, profile *momentinput.Profile, delta float64) (Region, error) {
	work := freeform.NewFreeformWork()
	var walks [][]survey2d.SegmentWalk
	for _, loop := range append([]sectionrecord.LoopRecord{profile.Outer}, profile.Holes...) {
		var ws []survey2d.SegmentWalk
		for _, seg := range loop.Segments {
			w, err := boundarywalk.WalkOf(seg, work)
			if err != nil {
				return Region{}, err
			}
			ws = append(ws, w)
		}
		walks = append(walks, ws)
	}
	return RegionOf(ctx, profile, delta, walks)
}

// Enclosure reads an exact value or widens a held float by its proven bound.
func Enclosure(value, bound float64, exact *big.Rat) (proofbound.RatInterval, error) {
	if exact != nil {
		return proofbound.PointInterval(exact), nil
	}
	v, b := proofarith.FloatRat(value), proofarith.FloatRat(bound)
	if v == nil || b == nil {
		return proofbound.RatInterval{}, fmt.Errorf(`%w: a brep face's integral is not finite`, decaderr.ErrNotFinite)
	}
	return proofbound.IntervalWiden(proofbound.PointInterval(v), b), nil
}

// SegmentIntegrals encloses a wall segment's Green's-theorem contributions:
// area, and the first moments in the segment's frame coordinates.
func SegmentIntegrals(seg sectionrecord.CurveSegment) ([3]proofbound.RatInterval, error) {
	var ig momentinput.Integrals
	if err := ig.AddFor(seg, momentinput.Plan{}, sectionrecord.Point2{}, freeform.MomentFirstOrder); err != nil {
		return [3]proofbound.RatInterval{}, err
	}
	var exact [3]*big.Rat
	if !ig.ExactDead && ig.Exact.Complete() {
		exact = [3]*big.Rat{ig.Exact.Area, ig.Exact.Mu, ig.Exact.Mv}
	}
	var out [3]proofbound.RatInterval
	for i, field := range [3][2]float64{{ig.Area, ig.AreaBound}, {ig.Mu, ig.MuBound}, {ig.Mv, ig.MvBound}} {
		iv, err := Enclosure(field[0], field[1], exact[i])
		if err != nil {
			return [3]proofbound.RatInterval{}, err
		}
		out[i] = iv
	}
	return out, nil
}

// Held publishes an enclosure's point or midpoint rounded once to float.
func Held(iv proofbound.RatInterval) float64 {
	if iv.Lo.Cmp(iv.Hi) == 0 {
		held, _ := iv.Lo.Float64()
		return held
	}
	mid := new(big.Rat).Add(iv.Lo, iv.Hi)
	held, _ := mid.Quo(mid, big.NewRat(2, 1)).Float64()
	return held
}

// WallArea multiplies a swept segment's bounded length by its bounded height.
func WallArea(delta, z0, z1, z0Delta, z1Delta float64, w survey2d.SegmentWalk) proofbound.BoundedScalar {
	length := proofbound.MeasuredScalar(w.Length,
		proofbound.AbsSumUpper(w.LengthBound, proofbound.SectionDisplacementLength(delta, 1)))
	height := proofbound.BoundedSub(proofbound.MeasuredScalar(z1, z1Delta), proofbound.MeasuredScalar(z0, z0Delta))
	return proofbound.BoundedMul(length, height)
}
