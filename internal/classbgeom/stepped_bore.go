package classbgeom

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// cutSteppedBore reads a through-tool along the axis of a stepped turned
// shaft. It admits only a closed line-and-arc tool outline whose exact segment
// boxes fit strictly inside every step. Sketch authenticates the section cut;
// the resulting B-rep keeps each outer cylinder, bore wall, cap and shoulder.
func cutSteppedBore(ctx context.Context, steps []prismrecord.Payload, payload topo.Payload) (topo.Payload, bool, error) {
	tool, ok := payload.(prismrecord.Payload)
	if !ok || len(steps) < 2 || len(tool.Profile.Holes) != 0 || tool.SurfaceResult ||
		tool.SectionDelta != 0 || tool.Z0Delta != 0 || tool.Z1Delta != 0 || !(tool.Z0 < tool.Z1) ||
		!NaturalRecord([]sectionrecord.LoopRecord{tool.Profile.Outer}) {
		return nil, false, nil
	}
	if len(tool.Profile.Outer.Segments) == 0 {
		return nil, false, nil
	}
	for _, seg := range tool.Profile.Outer.Segments {
		switch seg.(type) {
		case sectionrecord.LineSeg, sectionrecord.ArcSeg, sectionrecord.CircleSeg:
		default:
			return nil, false, nil
		}
	}

	first := steps[0]
	last := steps[len(steps)-1]
	relative, err := prismplacement.Compose(first.PlacementOperand(), tool.PlacementOperand())
	if err != nil {
		return nil, false, err
	}
	loop, z0, z1, exact, err := ReexpressAxialBoreTool(ctx, tool.Profile.Outer, relative,
		first.PlacementOperand(), tool.PlacementOperand())
	if err != nil {
		return nil, false, err
	}
	if !exact || z0 > first.Z0 || z1 < last.Z1 {
		return nil, false, nil
	}
	tool.Profile.Outer = loop
	tool.Frame, tool.Xform, tool.Z0, tool.Z1 = first.Frame, first.Xform, z0, z1
	center := steppedCenter(first)
	minRadius := 0.0
	for i, step := range steps {
		circle, ok := steppedCircle(step)
		if !ok || step.Frame != first.Frame || step.Xform != first.Xform ||
			step.Z0Delta != 0 || step.Z1Delta != 0 || step.SectionDelta != 0 ||
			step.SurfaceResult || circle.Center != center || !(step.Z0 < step.Z1) ||
			(i > 0 && steps[i-1].Z1 != step.Z0) {
			return nil, false, nil
		}
		radius, err := circle.Radius.In(units.Millimeter)
		if err != nil {
			return nil, false, err
		}
		if radius <= 0 {
			return nil, false, nil
		}
		if i == 0 || radius < minRadius {
			minRadius = radius
		}
	}
	if !outlineBoxInsideCircle(tool.Profile.Outer, center, minRadius) {
		return nil, false, nil
	}

	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, false, err
	}
	reexpress, err := prismcells.NewReexpression(first.PlacementOperand(), tool.PlacementOperand())
	if err != nil || !reexpress.Identity {
		return nil, false, err
	}
	profiles := make([]momentinput.Profile, len(steps))
	for i, step := range steps {
		_, within, err := prismcells.RegionsWithinWorkCap(budget, step.Profile, tool.Profile)
		if err != nil || !within {
			return nil, false, err
		}
		scene, match, tags, delta, resolved, err := prismcells.ResolveCutWithTags(ctx, budget, step.Profile, tool.Profile, reexpress)
		if err != nil || !resolved || delta.A != 0 || delta.B != 0 {
			return nil, false, err
		}
		profile, err := prismcells.RecordProfileContext(ctx, scene, match)
		if err != nil {
			return nil, false, err
		}
		profile, err = prismcells.RecordArrangedProfileContext(ctx, match)
		if err != nil {
			return nil, false, err
		}
		profile, err = prismcells.CanonicalStackedCut(budget, step.Profile, match, tags, profile)
		if err != nil {
			return nil, false, err
		}
		if len(profile.Holes) != 1 {
			return nil, false, nil
		}
		profiles[i] = profile
	}

	if err := budget.Err(); err != nil {
		return nil, false, err
	}
	bp := breprecord.Payload{Xform: first.Xform}
	for i, step := range steps {
		circle, _ := steppedCircle(step)
		bp.Faces = append(bp.Faces, breprecord.Face{
			Frame: first.Frame, Wall: circle, Z0: step.Z0, Z1: step.Z1,
		})
		if i == 0 {
			bottom := profiles[i]
			bp.Faces = append(bp.Faces, breprecord.Face{
				Frame: first.Frame, Region: &bottom, Z0: first.Z0, Z1: first.Z0,
			})
		}
		if i == len(steps)-1 {
			top := profiles[i]
			bp.Faces = append(bp.Faces, breprecord.Face{
				Frame: first.Frame, Region: &top, Outward: true, Z0: last.Z1, Z1: last.Z1,
			})
		}
		if i == 0 {
			continue
		}
		prev, _ := steppedCircle(steps[i-1])
		wide, narrow := circle, prev
		outward := false
		prevRadius, _ := prev.Radius.In(units.Millimeter)
		circleRadius, _ := circle.Radius.In(units.Millimeter)
		if prevRadius > circleRadius {
			wide, narrow, outward = prev, circle, true
		}
		narrowLoop, err := offset2d.ReverseLoopRecordContext(ctx, sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{narrow}})
		if err != nil {
			return nil, false, err
		}
		region := momentinput.Profile{
			Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{wide}},
			Holes: []sectionrecord.LoopRecord{narrowLoop},
		}
		bp.Faces = append(bp.Faces, breprecord.Face{
			Frame: first.Frame, Region: &region, Outward: outward,
			Z0: step.Z0, Z1: step.Z0,
		})
	}
	for _, seg := range profiles[0].Holes[0].Segments {
		bp.Faces = append(bp.Faces, breprecord.Face{
			Frame: first.Frame, Wall: seg, Z0: first.Z0, Z1: last.Z1,
		})
	}
	bp.AssignRoles()
	return bp, true, nil
}

// ReexpressAxialBoreTool maps a line/arc/circle outline and sweep interval
// into the target's section frame. Signed axis changes are exact; an opposite
// sweep normal reverses the interval and the section winding before the cut.
func ReexpressAxialBoreTool(ctx context.Context, loop sectionrecord.LoopRecord, relative prismplacement.Relative,
	target, tool prismplacement.Operand) (sectionrecord.LoopRecord, float64, float64, bool, error) {
	var matrix [2][2]int
	axialSign := 1
	determinant := 1
	translation := r3.Vec{}
	if relative.Identity {
		matrix[0][0], matrix[1][1] = 1, 1
	} else {
		basis := relative.Map.Basis()
		for col, axis := range [2]r3.Vec{basis.EX, basis.EY} {
			row, sign, ok := signedUnitAxis2(axis)
			if !ok || row > 1 || matrix[row][0] != 0 || matrix[row][1] != 0 {
				return sectionrecord.LoopRecord{}, 0, 0, false, nil
			}
			matrix[row][col] = sign
		}
		_, sign, ok := signedUnitAxis2(basis.EZ)
		if !ok || basis.EZ.X != 0 || basis.EZ.Y != 0 {
			return sectionrecord.LoopRecord{}, 0, 0, false, nil
		}
		axialSign = sign
		translation = relative.Map.Translation()
		determinant = matrix[0][0]*matrix[1][1] - matrix[0][1]*matrix[1][0]
		if determinant != axialSign {
			return sectionrecord.LoopRecord{}, 0, 0, false, nil
		}
	}
	if proofarith.FloatRat(translation.X) == nil || proofarith.FloatRat(translation.Y) == nil ||
		proofarith.FloatRat(translation.Z) == nil {
		return sectionrecord.LoopRecord{}, 0, 0, false, nil
	}
	mapValue := func(a, b float64, signA, signB int, offset float64) (float64, bool) {
		ra, rb, ro := proofarith.FloatRat(a), proofarith.FloatRat(b), proofarith.FloatRat(offset)
		if ra == nil || rb == nil || ro == nil {
			return 0, false
		}
		exact := new(big.Rat).Set(ro)
		if signA != 0 {
			exact.Add(exact, new(big.Rat).Mul(ra, big.NewRat(int64(signA), 1)))
		}
		if signB != 0 {
			exact.Add(exact, new(big.Rat).Mul(rb, big.NewRat(int64(signB), 1)))
		}
		held, _ := exact.Float64()
		return held, proofarith.RationalFloatError(exact, held) == 0
	}
	mapPoint := func(point sectionrecord.Point2) sectionrecord.Point2 {
		u, _ := mapValue(point.U, point.V, matrix[0][0], matrix[0][1], translation.X)
		v, _ := mapValue(point.U, point.V, matrix[1][0], matrix[1][1], translation.Y)
		return sectionrecord.Point2{U: u, V: v}
	}
	pointExact := func(point sectionrecord.Point2) bool {
		_, exactU := mapValue(point.U, point.V, matrix[0][0], matrix[0][1], translation.X)
		_, exactV := mapValue(point.U, point.V, matrix[1][0], matrix[1][1], translation.Y)
		return exactU && exactV
	}
	mapped := sectionrecord.LoopRecord{}
	for _, segment := range loop.Segments {
		switch s := segment.(type) {
		case sectionrecord.LineSeg:
			if !pointExact(s.Start) || !pointExact(s.End) {
				return sectionrecord.LoopRecord{}, 0, 0, false, nil
			}
			s.Start, s.End = mapPoint(s.Start), mapPoint(s.End)
			mapped.Segments = append(mapped.Segments, s)
		case sectionrecord.ArcSeg:
			if !pointExact(s.Center) || !pointExact(s.Start) || !pointExact(s.End) {
				return sectionrecord.LoopRecord{}, 0, 0, false, nil
			}
			s.Center, s.Start, s.End = mapPoint(s.Center), mapPoint(s.Start), mapPoint(s.End)
			if determinant < 0 {
				s.Start, s.End = s.End, s.Start
				s.TStart, s.TEnd = 1-s.TStart, 1-s.TEnd
			}
			mapped.Segments = append(mapped.Segments, s)
		case sectionrecord.CircleSeg:
			if !pointExact(s.Center) {
				return sectionrecord.LoopRecord{}, 0, 0, false, nil
			}
			s.Center = mapPoint(s.Center)
			if determinant < 0 {
				s.CCW = !s.CCW
			}
			mapped.Segments = append(mapped.Segments, s)
		default:
			return sectionrecord.LoopRecord{}, 0, 0, false, nil
		}
	}
	if determinant < 0 {
		reversed, err := offset2d.ReverseLoopRecordContext(ctx, mapped)
		if err != nil {
			return sectionrecord.LoopRecord{}, 0, 0, false, err
		}
		mapped = reversed
	}
	var z0, z1 float64
	var exact0, exact1 bool
	if relative.Identity {
		lo, hi, ok := prismplacement.ShiftedInterval(target, tool)
		if !ok {
			return sectionrecord.LoopRecord{}, 0, 0, false, nil
		}
		z0, exact0 = ratFloatExact(lo)
		z1, exact1 = ratFloatExact(hi)
	} else {
		za, exactA := mapValue(tool.Z0, 0, axialSign, 0, translation.Z)
		zb, exactB := mapValue(tool.Z1, 0, axialSign, 0, translation.Z)
		exact0, exact1 = exactA, exactB
		if axialSign < 0 {
			z0, z1 = zb, za
		} else {
			z0, z1 = za, zb
		}
	}
	if !exact0 || !exact1 {
		return sectionrecord.LoopRecord{}, 0, 0, false, nil
	}
	return mapped, z0, z1, true, nil
}

func ratFloatExact(value *big.Rat) (float64, bool) {
	if value == nil {
		return 0, false
	}
	held, _ := value.Float64()
	return held, proofarith.RationalFloatError(value, held) == 0
}

func signedUnitAxis2(axis r3.Vec) (index, sign int, ok bool) {
	switch axis {
	case r3.Vec{X: 1}:
		return 0, 1, true
	case r3.Vec{X: -1}:
		return 0, -1, true
	case r3.Vec{Y: 1}:
		return 1, 1, true
	case r3.Vec{Y: -1}:
		return 1, -1, true
	case r3.Vec{Z: 1}:
		return 2, 1, true
	case r3.Vec{Z: -1}:
		return 2, -1, true
	default:
		return 0, 0, false
	}
}

// outlineBoxInsideCircle is a conservative exact containment proof: every
// segment's outward box must fit strictly inside the smallest target circle.
func outlineBoxInsideCircle(loop sectionrecord.LoopRecord, center sectionrecord.Point2, radius float64) bool {
	var bounds Box2
	for i, seg := range loop.Segments {
		box, err := SegmentBox(seg)
		if err != nil {
			return false
		}
		if i == 0 {
			bounds = box
		} else {
			bounds = Union(bounds, box)
		}
	}
	if len(loop.Segments) == 0 {
		return false
	}
	rat := proofarith.FloatRat
	dx := maxAbs(new(big.Rat).Sub(bounds.Lo[0], rat(center.U)), new(big.Rat).Sub(bounds.Hi[0], rat(center.U)))
	dy := maxAbs(new(big.Rat).Sub(bounds.Lo[1], rat(center.V)), new(big.Rat).Sub(bounds.Hi[1], rat(center.V)))
	distance2 := proofbound.RatAdd(proofbound.RatMul(dx, dx), proofbound.RatMul(dy, dy))
	r := rat(radius)
	r.Mul(r, r)
	return distance2.Cmp(r) < 0
}

func maxAbs(a, b *big.Rat) *big.Rat {
	a.Abs(a)
	b.Abs(b)
	if a.Cmp(b) > 0 {
		return a
	}
	return b
}
