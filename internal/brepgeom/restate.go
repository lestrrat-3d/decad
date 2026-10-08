package brepgeom

import (
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// ErrRestate marks a swept face that does not restate as a planar face.
var ErrRestate = errors.New("the swept face does not restate as a plane")

// Restate states a swept straight wall as the planar rectangle it sweeps
// (docs/brep-modify-design.md §5.2). ref is the record's reference frame.
//
// The wall must be a LineSeg over its natural range (0→1 or 1→0) with
// exactly one of its walked run's two plane-local components zero, its side
// lines unsplit, and neither sweep level displaced: a level of the swept face
// becomes a section coordinate of the planar one, and the planar record
// carries no displacement for one. Any other face is ErrRestate, naming what
// fails.
//
// The rectangle's frame is PlanarFrame on the wall's constant reference axis,
// its normal the wall's outward normal: the right-hand normal of its walk in
// its own frame, the wall walking with the material on its left. Its region
// is the one loop through the corners (start, z0), (end, z0), (end, z1),
// (start, z1), each moved into the new frame by signed permutations alone,
// and that loop must turn counter-clockwise in the new frame (a reject-only
// check on the orientation reading). The returned record is planar at the
// wall's constant coordinate; its level and section displacements are the
// wall's section displacement, its role the wall's, and the caller records it
// with its frame normal outward. The returned Embed maps the new frame onto
// ref.
func Restate(face FaceRecord, ref r3.Frame) (FaceRecord, Embed, error) {
	switch {
	case face.Region != nil:
		return FaceRecord{}, Embed{}, fmt.Errorf(`%w: the face is already planar`, ErrRestate)
	case len(face.Side0) != 0 || len(face.Side1) != 0:
		return FaceRecord{}, Embed{}, fmt.Errorf(`%w: its side line is split`, ErrRestate)
	case face.Z0Delta != 0 || face.Z1Delta != 0:
		return FaceRecord{}, Embed{}, fmt.Errorf(`%w: a sweep level carries a displacement`, ErrRestate)
	}
	line, ok := face.Wall.(sectionrecord.LineSeg)
	if !ok {
		return FaceRecord{}, Embed{}, fmt.Errorf(`%w: its wall is a %T, not a straight line`, ErrRestate, face.Wall)
	}
	var from, to sectionrecord.Point2
	switch {
	case line.TStart == 0 && line.TEnd == 1:
		from, to = line.Start, line.End
	case line.TStart == 1 && line.TEnd == 0:
		from, to = line.End, line.Start
	default:
		return FaceRecord{}, Embed{}, fmt.Errorf(`%w: its wall runs over a narrowed range`, ErrRestate)
	}
	du, dv := to.U-from.U, to.V-from.V
	if (du == 0) == (dv == 0) {
		return FaceRecord{}, Embed{}, fmt.Errorf(`%w: its wall is oblique in its frame, or has no length`, ErrRestate)
	}
	embeds, err := Embeds([]r3.Frame{ref, face.Frame}, ErrRestate)
	if err != nil {
		return FaceRecord{}, Embed{}, err
	}
	e := embeds[1]

	// The outward normal is the walk's right-hand normal, (dv, −du) by sign;
	// one component is zero, so it lands on one reference axis.
	normal := e.Canon(signOf(dv), signOf(-du), 0)
	axis := -1
	for i, c := range normal {
		if c != 0 {
			axis = i
		}
	}
	frame, pe, err := PlanarFrame(ref, axis, normal[axis])
	if err != nil {
		return FaceRecord{}, Embed{}, fmt.Errorf(`%w: %w`, ErrRestate, err)
	}

	corners := [4][3]float64{
		e.Canon(from.U, from.V, face.Z0), e.Canon(to.U, to.V, face.Z0),
		e.Canon(to.U, to.V, face.Z1), e.Canon(from.U, from.V, face.Z1),
	}
	var pts [4]sectionrecord.Point2
	level := pe.Local(corners[0])[2]
	for i, c := range corners {
		l := pe.Local(c)
		if l[2] != level {
			return FaceRecord{}, Embed{}, fmt.Errorf(`%w: its corners do not share one level in the new frame`, ErrRestate)
		}
		pts[i] = sectionrecord.Point2{U: l[0], V: l[1]}
	}
	// A rectangle turns one way at every corner; each run has one nonzero
	// component, so the first corner's cross product is one exact product.
	ax, ay := pts[1].U-pts[0].U, pts[1].V-pts[0].V
	bx, by := pts[2].U-pts[1].U, pts[2].V-pts[1].V
	if !(ax*by-ay*bx > 0) {
		return FaceRecord{}, Embed{}, fmt.Errorf(`%w: its rectangle turns clockwise about its outward normal`, ErrRestate)
	}
	var loop sectionrecord.LoopRecord
	for i, p := range pts {
		loop.Segments = append(loop.Segments, sectionrecord.LineSeg{Start: p, End: pts[(i+1)%len(pts)], TStart: 0, TEnd: 1})
	}
	return FaceRecord{
		Frame: frame, Region: &Profile{Outer: loop},
		Z0: level, Z1: level, Z0Delta: face.Delta, Z1Delta: face.Delta,
		Delta: face.Delta, Role: face.Role,
	}, pe, nil
}

// signOf is −1, 0 or +1 by the sign of x.
func signOf(x float64) float64 {
	switch {
	case x > 0:
		return 1
	case x < 0:
		return -1
	default:
		return 0
	}
}
