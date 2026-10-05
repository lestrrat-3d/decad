package decad

import (
	"context"
	"fmt"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// SweptBox encloses every point a body occupies at any time of one pair path:
// the body's Bounds() at its current placement, inflated by its Bound and
// mapped exactly through the path's From, then expanded on every axis by the
// path's whole-duration travel bound (docs/contact-sweep-design.md §4.1). A
// path that only translates takes the exact hull of its start and end boxes,
// which holds every intermediate translate and lies inside that expansion.
// Its extremes are exact dyadic rationals; docs/multibody-dynamics-design.md
// §4.2 owns it.
//
// The zero value encloses nothing proven: it is never StrictlyDisjoint from
// any box, and its Box is the zero Box.
type SweptBox struct {
	lo, hi [3]proofarith.Dyadic
	body   *Body
}

// SweptBox returns the box b sweeps along path. It validates b and path as
// SweepPair does, and returns ErrUnsupported when b's bounds or the path's
// travel bound are not finite. It changes neither b nor the document.
// b, path and ctx must be non-nil.
func (d *Document) SweptBox(ctx context.Context, b *Body, path PairPath) (SweptBox, error) {
	if d == nil || ctx == nil {
		return SweptBox{}, fmt.Errorf("%w: nil document or context", ErrDegenerate)
	}
	if err := d.requireLive(b); err != nil {
		return SweptBox{}, err
	}
	p, err := validatePairPath(path)
	if err != nil {
		return SweptBox{}, err
	}
	if err := ctx.Err(); err != nil {
		return SweptBox{}, err
	}
	swept, ok := sweptBoxOf(b, p)
	if !ok {
		return SweptBox{}, fmt.Errorf("%w: body bounds or path travel bound are not finite", ErrUnsupported)
	}
	return swept, nil
}

// StrictlyDisjoint reports a positive gap between s and o on at least one
// axis, compared exactly. Boxes that merely meet are not disjoint.
func (s SweptBox) StrictlyDisjoint(o SweptBox) bool {
	if s.body == nil || o.body == nil {
		return false
	}
	for axis := range 3 {
		if proofarith.DyCmp(s.hi[axis], o.lo[axis]) < 0 || proofarith.DyCmp(o.hi[axis], s.lo[axis]) < 0 {
			return true
		}
	}
	return false
}

// Box converts the exact extremes outward: Min rounds down and Max rounds up,
// so the float box contains the exact one. Bound is the largest conversion
// step, and the box is Exact only when every extreme is a float.
func (s SweptBox) Box() Box {
	if s.body == nil {
		return Box{}
	}
	var lo, hi [3]float64
	bound := 0.0
	for axis := range 3 {
		lo[axis] = proofarith.DyFloatDown(s.lo[axis])
		hi[axis] = proofarith.DyFloatUp(s.hi[axis])
		bound = max(bound, proofarith.DyadicFloatError(s.lo[axis], lo[axis]),
			proofarith.DyadicFloatError(s.hi[axis], hi[axis]))
	}
	return Box{Min: r3.Vec{X: lo[0], Y: lo[1], Z: lo[2]}, Max: r3.Vec{X: hi[0], Y: hi[1], Z: hi[2]},
		Exactness: exactnessFromBound(bound), Bound: units.Millimeters(bound)}
}

// sweptBoxOf is the one swept-box implementation: Document.SweptBox and the
// sweeps that certify clearance by a box gap all read it. It reports false
// when b's bounds or the path's travel bound are not finite.
func sweptBoxOf(b *Body, p affinePairPath) (SweptBox, bool) {
	corners, ok := inflatedBoundsCorners(b)
	if !ok {
		return SweptBox{}, false
	}
	out := SweptBox{body: b}
	for i, corner := range corners {
		mapped := exactContactTransform(p.from, corner)
		for axis := range 3 {
			if i == 0 || proofarith.DyCmp(mapped[axis], out.lo[axis]) < 0 {
				out.lo[axis] = mapped[axis]
			}
			if i == 0 || proofarith.DyCmp(mapped[axis], out.hi[axis]) > 0 {
				out.hi[axis] = mapped[axis]
			}
		}
	}
	var travel *big.Rat
	switch {
	case p.drift != nil:
		travel, ok = driftTravel(b, p)
	case p.read != nil:
		travel, ok = screwTravel(b, p)
	default:
		for axis := range 3 {
			out.lo[axis] = dyMin(out.lo[axis], proofarith.DyAdd(out.lo[axis], p.delta[axis]))
			out.hi[axis] = dyMax(out.hi[axis], proofarith.DyAdd(out.hi[axis], p.delta[axis]))
		}
		return out, true
	}
	if !ok {
		return SweptBox{}, false
	}
	up := ratFloatUp(travel)
	if !finiteMeasurementValues(up) {
		return SweptBox{}, false
	}
	grow := proofarith.MustDyOf(up)
	for axis := range 3 {
		out.lo[axis] = proofarith.DySubScalar(out.lo[axis], grow)
		out.hi[axis] = proofarith.DyAdd(out.hi[axis], grow)
	}
	return out, true
}

// driftTravel is contact-sweep §4.1's τ_drift over the whole duration:
// (V + ρΩ)·Duration, with ρ the largest distance of the From-mapped bounds
// corners from the rotation line through Center.
func driftTravel(b *Body, p affinePairPath) (*big.Rat, bool) {
	drift := p.drift
	linear := [3]units.Value{drift.LinearVelocity.X, drift.LinearVelocity.Y, drift.LinearVelocity.Z}
	angular := [3]units.Value{drift.AngularVelocity.X, drift.AngularVelocity.Y, drift.AngularVelocity.Z}
	var omega ratVec
	vSquared, omegaSquared := new(big.Rat), new(big.Rat)
	for axis := range 3 {
		v, okV := exactBaseValue(linear[axis])
		w, okW := exactBaseValue(angular[axis])
		if !okV || !okW {
			return nil, false
		}
		omega[axis] = w
		vSquared.Add(vSquared, new(big.Rat).Mul(v, v))
		omegaSquared.Add(omegaSquared, new(big.Rat).Mul(w, w))
	}
	speed, spin := ratSqrtUp(vSquared), ratSqrtUp(omegaSquared)
	if !finiteMeasurementValues(speed, spin) {
		return nil, false
	}
	radius, ok := rotationalSweepRadius(b, p.from, drift.Center, omega)
	if !ok {
		return nil, false
	}
	rate := new(big.Rat).Add(proofarith.FloatRat(speed), new(big.Rat).Mul(radius, proofarith.FloatRat(spin)))
	return rate.Mul(rate, p.duration), true
}

// screwTravel is contact-sweep §4.1's τ_screw over the whole path: ρ|θ| + |d|,
// with ρ the largest distance of the From-mapped bounds corners from the
// read screw's axis line.
func screwTravel(b *Body, p affinePairPath) (*big.Rat, bool) {
	screw := p.read
	angle, okAngle := exactBaseValue(screw.Angle)
	slide := proofarith.FloatRat(screw.Slide)
	if !okAngle || slide == nil {
		return nil, false
	}
	travel := new(big.Rat).Abs(slide)
	if angle.Sign() == 0 {
		return travel, true
	}
	axis, ok := ratVecOf(screw.Axis)
	if !ok {
		return nil, false
	}
	radius, ok := rotationalSweepRadius(b, p.from, screw.Point, axis)
	if !ok {
		return nil, false
	}
	return travel.Add(travel, new(big.Rat).Mul(radius, angle.Abs(angle))), true
}

// inflatedBoundsCorners returns the eight corners of b's Bounds() at its
// current placement, each coordinate pushed outward by the box's Bound.
func inflatedBoundsCorners(b *Body) ([8]proofarith.DyV3, bool) {
	var corners [8]proofarith.DyV3
	box, err := b.Bounds()
	if err != nil || box.Bound.Kind() != units.Length ||
		!finiteMeasurementValues(box.Bound.Base(), box.Min.X, box.Min.Y, box.Min.Z,
			box.Max.X, box.Max.Y, box.Max.Z) || box.Bound.Base() < 0 {
		return corners, false
	}
	minimum := [3]float64{box.Min.X, box.Min.Y, box.Min.Z}
	maximum := [3]float64{box.Max.X, box.Max.Y, box.Max.Z}
	var extremes [3][2]proofarith.Dyadic
	bound := proofarith.MustDyOf(box.Bound.Base())
	for axis := range 3 {
		if minimum[axis] > maximum[axis] {
			return corners, false
		}
		extremes[axis] = [2]proofarith.Dyadic{proofarith.DySubScalar(proofarith.MustDyOf(minimum[axis]), bound),
			proofarith.DyAdd(proofarith.MustDyOf(maximum[axis]), bound)}
	}
	for index := range corners {
		for axis := range 3 {
			corners[index][axis] = extremes[axis][(index>>axis)&1]
		}
	}
	return corners, true
}
