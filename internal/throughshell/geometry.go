package throughshell

import (
	"fmt"
	"math/big"
	"reflect"
	"slices"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/classbgeom"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// StripInput states TC7's tool and receiver sections in their reference frames.
type StripInput struct {
	Tool, Dilated, Receiver           momentinput.Profile
	ToolFrame, ReceiverFrame          brepgeom.Embed
	SweepAxis, PiercedAxis            int
	Lo, Hi, Thickness, ThicknessDelta float64
}

// StripsClear rejects a receiver segment whose outward box may meet either
// material strip beyond the wall pierced by the dilated tool.
func StripsClear(in StripInput) (bool, error) {
	rat := proofarith.FloatRat
	m := 3 - in.SweepAxis - in.PiercedAxis
	reach := new(big.Rat).Add(rat(in.Thickness), rat(in.ThicknessDelta))
	mLo, mHi, err := extent(in.Tool, in.ToolFrame, m)
	if err != nil {
		return false, err
	}
	dLo, dHi, err := extent(in.Dilated, in.ToolFrame, m)
	if err != nil {
		return false, err
	}
	stripMLo := ratMin(new(big.Rat).Sub(mLo, reach), dLo)
	stripMHi := ratMax(new(big.Rat).Add(mHi, reach), dHi)
	lo, hi := rat(in.Lo), rat(in.Hi)
	strip0Lo := new(big.Rat).Sub(lo, reach)
	strip1Hi := new(big.Rat).Add(hi, reach)
	for _, loop := range append([]sectionrecord.LoopRecord{in.Receiver.Outer}, in.Receiver.Holes...) {
		for _, seg := range loop.Segments {
			box, err := classbgeom.SegmentBox(seg)
			if err != nil {
				return false, err
			}
			jLo, jHi := boxAxis(box, in.ReceiverFrame, in.PiercedAxis)
			sLo, sHi := boxAxis(box, in.ReceiverFrame, m)
			mApart := sHi.Cmp(stripMLo) < 0 || sLo.Cmp(stripMHi) > 0
			apart0 := mApart || jHi.Cmp(strip0Lo) < 0 || jLo.Cmp(lo) >= 0
			apart1 := mApart || jHi.Cmp(hi) <= 0 || jLo.Cmp(strip1Hi) > 0
			if !apart0 || !apart1 {
				return false, nil
			}
		}
	}
	return true, nil
}

// extent is the exact union of a section's segment boxes along one reference axis.
func extent(p momentinput.Profile, e brepgeom.Embed, axis int) (*big.Rat, *big.Rat, error) {
	var lo, hi *big.Rat
	for _, loop := range append([]sectionrecord.LoopRecord{p.Outer}, p.Holes...) {
		for _, seg := range loop.Segments {
			box, err := classbgeom.SegmentBox(seg)
			if err != nil {
				return nil, nil, err
			}
			l, h := boxAxis(box, e, axis)
			if lo == nil {
				lo, hi = l, h
				continue
			}
			lo, hi = ratMin(lo, l), ratMax(hi, h)
		}
	}
	if lo == nil {
		return nil, nil, fmt.Errorf(`%w: a tool section holds no segment`, decaderr.ErrDegenerate)
	}
	return lo, hi, nil
}

func boxAxis(box classbgeom.Box2, e brepgeom.Embed, axis int) (*big.Rat, *big.Rat) {
	for i := range 2 {
		if e.Axis[i] != axis {
			continue
		}
		if e.Sign[i] > 0 {
			return box.Lo[i], box.Hi[i]
		}
		return new(big.Rat).Neg(box.Hi[i]), new(big.Rat).Neg(box.Lo[i])
	}
	panic("throughshell: the axis is not in the frame's plane")
}

func ratMin(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) <= 0 {
		return a
	}
	return b
}

func ratMax(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) >= 0 {
		return a
	}
	return b
}

// RimWall is a straight receiver wall restated as a planar rim carrier.
type RimWall struct {
	Frame  r3.Frame
	Embed  brepgeom.Embed
	Axis   int
	Level  float64
	Region momentinput.Profile
}

// RestateWall states a straight wall along sweepAxis in a planar frame.
// A failed reading returns its face-specific reason to the caller.
func RestateWall(wall sectionrecord.CurveSegment, e brepgeom.Embed, ref r3.Frame,
	sweepAxis int, zlo, zhi float64) (RimWall, string) {
	from, to, ok := brepgeom.NaturalLine(wall)
	if !ok {
		return RimWall{}, "its wall is no straight line over its natural range"
	}
	a, b := e.Canon(from.U, from.V, 0), e.Canon(to.U, to.V, 0)
	j := -1
	for i := range 3 {
		if i != sweepAxis && a[i] == b[i] {
			j = i
		}
	}
	if j < 0 {
		return RimWall{}, "its wall lies along no section axis"
	}
	sign := 1.0
	if normal := e.Canon(to.V-from.V, from.U-to.U, 0); normal[j] < 0 {
		sign = -1
	}
	frame, embed, err := brepgeom.PlanarFrame(ref, j, sign)
	if err != nil {
		return RimWall{}, "its plane has no exact frame"
	}
	level := a[j]
	return RimWall{Frame: frame, Embed: embed, Axis: j, Level: level,
		Region: momentinput.Profile{Outer: RimRect(embed, a, b, sweepAxis, zlo, zhi)}}, ""
}

// SweptTrace states a straight cavity wall on a rim plane as a rectangle.
func SweptTrace(wall sectionrecord.CurveSegment, z0, z1 float64, eQ, eR brepgeom.Embed,
	axis int, level float64, sweepAxis int) (sectionrecord.LoopRecord, bool) {
	from, to, ok := brepgeom.NaturalLine(wall)
	if !ok {
		return sectionrecord.LoopRecord{}, false
	}
	a, b := eQ.Canon(from.U, from.V, 0), eQ.Canon(to.U, to.V, 0)
	if a[axis] != level || b[axis] != level {
		return sectionrecord.LoopRecord{}, false
	}
	l0, l1 := eQ.Sign[2]*z0+0, eQ.Sign[2]*z1+0
	return RimRect(eR, a, b, sweepAxis, min(l0, l1), max(l0, l1)), true
}

// RimRect is a swept segment's counter-clockwise rectangle in an embedded frame.
func RimRect(e brepgeom.Embed, a, b [3]float64, axis int, lo, hi float64) sectionrecord.LoopRecord {
	corners := [4][3]float64{a, b, b, a}
	corners[0][axis], corners[1][axis], corners[2][axis], corners[3][axis] = lo, lo, hi, hi
	pts := make([]sectionrecord.Point2, 4)
	for i, c := range corners {
		l := e.Local(c)
		pts[i] = sectionrecord.Point2{U: l[0], V: l[1]}
	}
	area := 0.0
	for i := range pts {
		j := (i + 1) % len(pts)
		area += pts[i].U*pts[j].V - pts[j].U*pts[i].V
	}
	if area < 0 {
		slices.Reverse(pts)
	}
	segs := make([]sectionrecord.CurveSegment, len(pts))
	for i := range pts {
		segs[i] = sectionrecord.LineSeg{Start: pts[i], End: pts[(i+1)%len(pts)], TStart: 0, TEnd: 1}
	}
	return sectionrecord.LoopRecord{Segments: segs}
}

// LoopsSame compares walked line ends regardless of the recorded line range.
// Other segments must match their recorded representation.
func LoopsSame(a, b sectionrecord.LoopRecord) bool {
	n := len(a.Segments)
	if n != len(b.Segments) || n == 0 {
		return false
	}
	same := func(x, y sectionrecord.CurveSegment) bool {
		xf, xt, xok := brepgeom.NaturalLine(x)
		yf, yt, yok := brepgeom.NaturalLine(y)
		if xok && yok {
			return xf == yf && xt == yt
		}
		return reflect.DeepEqual(x, y)
	}
	for r := range n {
		ok := true
		for i := range n {
			if !same(a.Segments[i], b.Segments[(i+r)%n]) {
				ok = false
				break
			}
		}
		if ok {
			return true
		}
	}
	return false
}
