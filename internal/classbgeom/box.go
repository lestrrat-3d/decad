package classbgeom

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// Box2 is a segment's outward box in its record plane, as exact rationals.
type Box2 struct{ Lo, Hi [2]*big.Rat }

// SegmentBox bounds one natural-range segment in its own plane: a line by
// its endpoints, a circle or arc by its whole circle, with the computed arc
// radius rounded outward by its proven bound.
func SegmentBox(seg sectionrecord.CurveSegment) (Box2, error) {
	rat := proofarith.FloatRat
	switch s := seg.(type) {
	case sectionrecord.LineSeg:
		b := Box2{}
		for i, pair := range [2][2]float64{{s.Start.U, s.End.U}, {s.Start.V, s.End.V}} {
			lo, hi := pair[0], pair[1]
			if lo > hi {
				lo, hi = hi, lo
			}
			b.Lo[i], b.Hi[i] = rat(lo), rat(hi)
		}
		return b, nil
	default:
		w, err := boundarywalk.WalkOf(seg, freeform.NewFreeformWork())
		if err != nil {
			return Box2{}, err
		}
		r := rat(proofbound.UpRound(proofbound.AbsSumUpper(w.Radius, w.RadiusBound)))
		if r == nil {
			return Box2{}, fmt.Errorf(`%w: a circular segment states no finite radius`, decaderr.ErrNotFinite)
		}
		c := [2]*big.Rat{rat(w.CU), rat(w.CV)}
		b := Box2{}
		for i := range c {
			b.Lo[i] = new(big.Rat).Sub(c[i], r)
			b.Hi[i] = new(big.Rat).Add(c[i], r)
		}
		return b, nil
	}
}

// Apart reports whether two closed intervals are separated exactly.
func Apart(alo, ahi, blo, bhi *big.Rat) bool {
	return ahi.Cmp(blo) < 0 || bhi.Cmp(alo) < 0
}

// Box3 is an exact axis-aligned box in reference coordinates.
type Box3 struct{ Lo, Hi [3]*big.Rat }

// Apart reports whether two boxes are separated along some axis.
func (a Box3) Apart(b Box3) bool {
	for k := range 3 {
		if Apart(a.Lo[k], a.Hi[k], b.Lo[k], b.Hi[k]) {
			return true
		}
	}
	return false
}

// Widened returns the box grown by w on every side of every axis. A zero w
// returns the box itself.
func (a Box3) Widened(w *big.Rat) Box3 {
	if w.Sign() == 0 {
		return a
	}
	var out Box3
	for k := range 3 {
		out.Lo[k] = new(big.Rat).Sub(a.Lo[k], w)
		out.Hi[k] = new(big.Rat).Add(a.Hi[k], w)
	}
	return out
}

// Place lifts a plane box and level interval through a signed axis map.
func Place(b Box2, zlo, zhi *big.Rat, axis [3]int, sign [3]float64) Box3 {
	lo := [3]*big.Rat{b.Lo[0], b.Lo[1], zlo}
	hi := [3]*big.Rat{b.Hi[0], b.Hi[1], zhi}
	var placed Box3
	for i := range 3 {
		k := axis[i]
		if sign[i] > 0 {
			placed.Lo[k], placed.Hi[k] = lo[i], hi[i]
			continue
		}
		placed.Lo[k], placed.Hi[k] = new(big.Rat).Neg(hi[i]), new(big.Rat).Neg(lo[i])
	}
	return placed
}

// Union is the smallest plane box holding both inputs.
func Union(a, b Box2) Box2 {
	out := a
	for i := range 2 {
		if b.Lo[i].Cmp(out.Lo[i]) < 0 {
			out.Lo[i] = b.Lo[i]
		}
		if b.Hi[i].Cmp(out.Hi[i]) > 0 {
			out.Hi[i] = b.Hi[i]
		}
	}
	return out
}

// FaceBox bounds a face's segments over its displaced axial interval.
func FaceBox(segs []sectionrecord.CurveSegment, z0, z1, z0Delta, z1Delta float64,
	axis [3]int, sign [3]float64) (Box3, error) {
	var plane Box2
	for i, seg := range segs {
		b, err := SegmentBox(seg)
		if err != nil {
			return Box3{}, err
		}
		if i == 0 {
			plane = b
			continue
		}
		plane = Union(plane, b)
	}
	rat := proofarith.FloatRat
	zlo := new(big.Rat).Sub(rat(z0), rat(z0Delta))
	zhi := new(big.Rat).Add(rat(z1), rat(z1Delta))
	return Place(plane, zlo, zhi, axis, sign), nil
}

// Plane is a planar carrier in reference coordinates.
type Plane struct {
	Axis  int
	Level *big.Rat
}

// RecordPlanes lists straight-wall planes in the given signed axis map.
func RecordPlanes(loops []sectionrecord.LoopRecord, axis [3]int, sign [3]float64) []Plane {
	var out []Plane
	for _, loop := range loops {
		for _, seg := range loop.Segments {
			line, ok := seg.(sectionrecord.LineSeg)
			if !ok {
				continue
			}
			if line.Start.U == line.End.U {
				out = append(out, Plane{Axis: axis[0], Level: proofarith.FloatRat(sign[0]*line.Start.U + 0)})
				continue
			}
			out = append(out, Plane{Axis: axis[1], Level: proofarith.FloatRat(sign[1]*line.Start.V + 0)})
		}
	}
	return out
}
