package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/sweeppath"

	"github.com/lestrrat-3d/decad/internal/motionbound"

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
	p, err := sweeppath.Validate(path)
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
	bounds, ok := sweeppath.SweepBounds(corners, p, exactContactTransform,
		func(from r3.Transform, center r3.Vec, axis motionbound.RatVec) (*big.Rat, bool) {
			return rotationalSweepRadius(b, from, center, axis)
		})
	if !ok {
		return SweptBox{}, false
	}
	return SweptBox{lo: bounds.Lo, hi: bounds.Hi, body: b}, true
}

// inflatedBoundsCorners returns the eight corners of b's Bounds() at its
// current placement, each coordinate pushed outward by the box's Bound.
func inflatedBoundsCorners(b *Body) ([8]proofarith.DyV3, bool) {
	box, err := b.Bounds()
	if err != nil || box.Bound.Kind() != units.Length ||
		!finiteMeasurementValues(box.Bound.Base(), box.Min.X, box.Min.Y, box.Min.Z,
			box.Max.X, box.Max.Y, box.Max.Z) || box.Bound.Base() < 0 {
		return [8]proofarith.DyV3{}, false
	}
	return sweeppath.InflatedBoundsCorners(box.Min, box.Max, box.Bound.Base())
}
