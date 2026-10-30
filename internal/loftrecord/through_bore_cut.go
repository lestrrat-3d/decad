package loftrecord

import (
	"context"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftfillet"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// ThroughBoreCut keeps a matching axial loft as a loft when a whole circular,
// D-shaped, or closed line-and-arc-profile through-tool becomes one new hole
// in each section. A conservative circle certificate proves the hole lies
// inside the existing outer loop. Every other pair keeps the general boolean
// path.
func ThroughBoreCut(ctx context.Context, env topo.Env, a, b topo.Payload) (*topo.Body, bool, error) {
	pl, ok := a.(Payload)
	if !ok || pl.SurfaceResult || len(pl.Profile0.Holes) != 0 ||
		!sectionrecord.IdenticalRecord(pl.Profile0, pl.Profile1) ||
		pl.Frame0.U() != pl.Frame1.U() || pl.Frame0.V() != pl.Frame1.V() ||
		pl.Xform != r3.Identity() {
		return nil, false, nil
	}
	for _, off := range pl.Alignment {
		if off != 0 {
			return nil, false, nil
		}
	}
	tool, ok := b.(prismrecord.Payload)
	if !ok || tool.SurfaceResult || tool.Xform != r3.Identity() ||
		proofbound.IsNonFinite(tool.SectionDelta) || tool.SectionDelta < 0 ||
		tool.Z0Delta != 0 || tool.Z1Delta != 0 ||
		tool.Frame.U() != pl.Frame0.U() || tool.Frame.V() != pl.Frame0.V() ||
		len(tool.Profile.Holes) != 0 {
		return nil, false, nil
	}
	circle, wholeCircle, ok, err := throughBoreToolCircle(tool.Profile.Outer, tool.SectionDelta)
	profileLoop := false
	if err != nil {
		return nil, false, err
	}
	if !ok {
		circle, ok, err = throughBoreProfileCircle(tool.Profile.Outer)
		if err != nil {
			return nil, false, err
		}
		if !ok {
			return nil, false, nil
		}
		profileLoop = true
	}
	if wholeCircle && tool.SectionDelta != 0 {
		return nil, false, nil
	}
	centerWorld := tool.Point(circle.Center.U, circle.Center.V, 0)
	center := pl.Frame0.ToLocal(centerWorld)
	if !loftToolCenterExact(tool, pl.Frame0, circle.Center.U, circle.Center.V, center) {
		return nil, false, nil
	}
	end := pl.Frame0.ToLocal(pl.Frame1.Origin())
	if end.Z <= 0 || !loftToolSpansExactCaps(tool, pl.Frame0, pl.Frame1) {
		return nil, false, nil
	}
	profile := pl.Profile0
	var hole sectionrecord.LoopRecord
	if wholeCircle {
		circle.Center = sectionrecord.Point2{U: center.X, V: center.Y}
		circle.CCW, circle.TStart, circle.TEnd = false, 1, 0
		hole = sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{circle}}
	} else if profileLoop {
		hole, ok = mapProfileToolHole(tool.Profile.Outer, tool.Frame, pl.Frame0, center.Z)
		if !ok {
			return nil, false, nil
		}
	} else {
		hole, ok = mapDToolHole(tool.Profile.Outer, tool.Frame, pl.Frame0, center.Z)
		if !ok {
			return nil, false, nil
		}
	}
	profile.Holes = []sectionrecord.LoopRecord{hole}
	budget := proofbound.NewWorkBudget(ctx)
	work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
	pl.RaiseReconstructionLimit(work0, work1)
	limit := pl.RewriteWorkLimit(uint64(len(profile.Outer.Segments) + 1))
	work0.RaiseLimit(limit)
	work1.RaiseLimit(limit)
	container := circle
	container.Center = sectionrecord.Point2{U: center.X, V: center.Y}
	container.Radius = units.Millimeters(proofbound.AbsSumUpper(
		container.Radius.Base(), tool.SectionDelta))
	targetDelta := proofbound.AbsSumUpper(pl.SectionDelta, pl.JoinDelta, pl.ConstructionDelta)
	contained, err := loftfillet.CircleHoleInside(budget, pl.Profile0.Outer, container, targetDelta, work0)
	if err != nil {
		if ctx.Err() != nil {
			return nil, false, ctx.Err()
		}
		return nil, false, nil
	}
	if !contained {
		return nil, false, nil
	}
	ig, err := profile.EvaluatorIntegralsContext(ctx, freeform.MomentAreaOrder, work0)
	if err != nil {
		return nil, false, err
	}
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}
	next := pl
	next.Profile0, next.Profile1 = profile, profile
	next.ThroughBoreCut = true
	next.SectionDelta = max(pl.SectionDelta, tool.SectionDelta)
	next.RecordArea = [2]float64{ig.Area, ig.Area}
	// The analytic wall carries the normal departure a rounded center, a
	// non-isometric frame map or a tilted held axis leaves on its tag. A bore
	// whose departure has no bound keeps the Loft's tessellated topology.
	_, bounded := BoreCylinderNormalBound(pl.Frame0, pl.Xform, container.Center.U, container.Center.V,
		end.Z, container.Radius.Base())
	next.AnalyticBore = wholeCircle && BoreHasExactAxis(pl.Frame0, pl.Frame1, container.Center.U, container.Center.V, end.Z) && bounded
	out, err := Build(ctx, env, next, budget, work0, work1)
	if err != nil {
		return nil, false, err
	}
	return out, true, nil
}

// throughBoreProfileCircle encloses a validated closed line and circular-arc
// loop in a conservative circle. Each line lies between its endpoints, and
// each arc lies inside its outward-rounded supporting circle.
func throughBoreProfileCircle(loop sectionrecord.LoopRecord) (sectionrecord.CircleSeg, bool, error) {
	if len(loop.Segments) < 2 {
		return sectionrecord.CircleSeg{}, false, nil
	}
	type point struct{ u, v *big.Rat }
	starts, ends := make([]point, len(loop.Segments)), make([]point, len(loop.Segments))
	work := freeform.NewFreeformWork()
	minU, maxU, minV, maxV := math.Inf(1), math.Inf(-1), math.Inf(1), math.Inf(-1)
	for i, segment := range loop.Segments {
		switch current := segment.(type) {
		case sectionrecord.LineSeg:
			if !wholeSectionSegment(current) {
				return sectionrecord.CircleSeg{}, false, nil
			}
		case sectionrecord.ArcSeg:
			if !wholeSectionSegment(current) {
				return sectionrecord.CircleSeg{}, false, nil
			}
		case sectionrecord.CircleSeg:
			// The single whole-circle case was handled before this helper.
		default:
			return sectionrecord.CircleSeg{}, false, nil
		}
		walk, err := boundarywalk.WalkOf(segment, work)
		if err != nil {
			return sectionrecord.CircleSeg{}, false, err
		}
		if walk.Closed || walk.StartU == walk.EndU && walk.StartV == walk.EndV && !walk.IsCircular() {
			return sectionrecord.CircleSeg{}, false, nil
		}
		starts[i] = point{proofarith.FloatRat(walk.StartU), proofarith.FloatRat(walk.StartV)}
		ends[i] = point{proofarith.FloatRat(walk.EndU), proofarith.FloatRat(walk.EndV)}
		if starts[i].u == nil || starts[i].v == nil || ends[i].u == nil || ends[i].v == nil {
			return sectionrecord.CircleSeg{}, false, nil
		}
		if i > 0 && (ends[i-1].u.Cmp(starts[i].u) != 0 || ends[i-1].v.Cmp(starts[i].v) != 0) {
			return sectionrecord.CircleSeg{}, false, nil
		}
		if walk.IsCircular() {
			radius := proofbound.AbsSumUpper(walk.Radius, walk.RadiusBound,
				proofbound.WalkEndBoundAllow(walk.StartBound), proofbound.WalkEndBoundAllow(walk.EndBound))
			if proofbound.IsNonFinite(radius) || radius <= 0 {
				return sectionrecord.CircleSeg{}, false, nil
			}
			radiusRat := proofarith.FloatRat(radius)
			centerURat, centerVRat := proofarith.FloatRat(walk.CU), proofarith.FloatRat(walk.CV)
			if radiusRat == nil || centerURat == nil || centerVRat == nil {
				return sectionrecord.CircleSeg{}, false, nil
			}
			loU := proofbound.RatFloatDown(new(big.Rat).Sub(centerURat, radiusRat))
			hiU := proofbound.RatFloatUp(new(big.Rat).Add(centerURat, radiusRat))
			loV := proofbound.RatFloatDown(new(big.Rat).Sub(centerVRat, radiusRat))
			hiV := proofbound.RatFloatUp(new(big.Rat).Add(centerVRat, radiusRat))
			minU, maxU = math.Min(minU, loU), math.Max(maxU, hiU)
			minV, maxV = math.Min(minV, loV), math.Max(maxV, hiV)
		}
		minU, maxU = math.Min(minU, math.Min(walk.StartU, walk.EndU)), math.Max(maxU, math.Max(walk.StartU, walk.EndU))
		minV, maxV = math.Min(minV, math.Min(walk.StartV, walk.EndV)), math.Max(maxV, math.Max(walk.StartV, walk.EndV))
	}
	if ends[len(ends)-1].u.Cmp(starts[0].u) != 0 || ends[len(ends)-1].v.Cmp(starts[0].v) != 0 {
		return sectionrecord.CircleSeg{}, false, nil
	}
	centerU, centerV := (minU+maxU)/2, (minV+maxV)/2
	centerURat, centerVRat := proofarith.FloatRat(centerU), proofarith.FloatRat(centerV)
	if centerURat == nil || centerVRat == nil {
		return sectionrecord.CircleSeg{}, false, nil
	}
	radiusSq := new(big.Rat)
	for _, p := range [][2]float64{{minU, minV}, {minU, maxV}, {maxU, minV}, {maxU, maxV}} {
		u, v := proofarith.FloatRat(p[0]), proofarith.FloatRat(p[1])
		if u == nil || v == nil {
			return sectionrecord.CircleSeg{}, false, nil
		}
		du, dv := new(big.Rat).Sub(u, centerURat), new(big.Rat).Sub(v, centerVRat)
		distanceSq := new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))
		if distanceSq.Cmp(radiusSq) > 0 {
			radiusSq.Set(distanceSq)
		}
	}
	radius := proofbound.RatSqrtUp(radiusSq)
	if proofbound.IsNonFinite(radius) || radius <= 0 {
		return sectionrecord.CircleSeg{}, false, nil
	}
	return sectionrecord.CircleSeg{
		Center: sectionrecord.Point2{U: centerU, V: centerV},
		Radius: units.Millimeters(radius), CCW: true, TStart: 0, TEnd: 1,
	}, true, nil
}

func mapProfileToolHole(loop sectionrecord.LoopRecord, toolFrame, loftFrame r3.Frame, level float64) (sectionrecord.LoopRecord, bool) {
	segments := make([]sectionrecord.CurveSegment, 0, len(loop.Segments))
	work := freeform.NewFreeformWork()
	for _, source := range slices.Backward(loop.Segments) {
		walk, err := boundarywalk.WalkOf(source, work)
		if err != nil || walk.Closed {
			return sectionrecord.LoopRecord{}, false
		}
		start, ok := mapToolPoint(sectionrecord.Point2{U: walk.EndU, V: walk.EndV}, toolFrame, loftFrame, level)
		if !ok {
			return sectionrecord.LoopRecord{}, false
		}
		end, ok := mapToolPoint(sectionrecord.Point2{U: walk.StartU, V: walk.StartV}, toolFrame, loftFrame, level)
		if !ok {
			return sectionrecord.LoopRecord{}, false
		}
		if walk.IsCircular() {
			center, ok := mapToolPoint(sectionrecord.Point2{U: walk.CU, V: walk.CV}, toolFrame, loftFrame, level)
			if !ok {
				return sectionrecord.LoopRecord{}, false
			}
			segments = append(segments, offset2d.ArcSegment(center, start, end, !(walk.Th1 > walk.Th0)))
			continue
		}
		segments = append(segments, sectionrecord.LineSeg{Start: start, End: end, TStart: 0, TEnd: 1})
	}
	return sectionrecord.LoopRecord{Segments: segments}, len(segments) >= 2
}

// throughBoreToolCircle returns the circle that contains a circular or
// D-shaped through-hole tool. A D-shaped tool is bounded by one circular arc
// and its chord, so its entire filled section lies inside the supporting circle.
func throughBoreToolCircle(loop sectionrecord.LoopRecord, sectionDelta float64) (sectionrecord.CircleSeg, bool, bool, error) {
	if len(loop.Segments) == 1 {
		circle, ok := loop.Segments[0].(sectionrecord.CircleSeg)
		if !ok || !wholeSectionSegment(circle) {
			return sectionrecord.CircleSeg{}, false, false, nil
		}
		return circle, true, true, nil
	}
	if len(loop.Segments) != 2 {
		return sectionrecord.CircleSeg{}, false, false, nil
	}
	var arc sectionrecord.CircleSeg
	var line sectionrecord.LineSeg
	hasArc, hasLine := false, false
	for _, segment := range loop.Segments {
		switch segment := segment.(type) {
		case sectionrecord.CircleSeg:
			if hasArc {
				return sectionrecord.CircleSeg{}, false, false, nil
			}
			arc, hasArc = segment, true
		case sectionrecord.LineSeg:
			if hasLine {
				return sectionrecord.CircleSeg{}, false, false, nil
			}
			line, hasLine = segment, true
		default:
			return sectionrecord.CircleSeg{}, false, false, nil
		}
	}
	if !hasArc || !hasLine {
		return sectionrecord.CircleSeg{}, false, false, nil
	}
	arcWalk, err := boundarywalk.WalkOf(arc, freeform.NewFreeformWork())
	if err != nil || !arcWalk.IsCircular() || arcWalk.Closed {
		return sectionrecord.CircleSeg{}, false, false, err
	}
	lineWalk, err := boundarywalk.WalkOf(line, freeform.NewFreeformWork())
	if err != nil || lineWalk.Closed || lineWalk.StartU == lineWalk.EndU && lineWalk.StartV == lineWalk.EndV {
		return sectionrecord.CircleSeg{}, false, false, err
	}
	endpointsClose := func(au, av float64, ab proofbound.WalkEndBound, bu, bv float64, bb proofbound.WalkEndBound) bool {
		distance := math.Hypot(au-bu, av-bv)
		allow := proofbound.AbsSumUpper(proofbound.WalkEndBoundAllow(ab),
			proofbound.WalkEndBoundAllow(bb), sectionDelta)
		return !proofbound.IsNonFinite(distance) && !proofbound.IsNonFinite(allow) && distance <= allow
	}
	if !(endpointsClose(arcWalk.StartU, arcWalk.StartV, arcWalk.StartBound,
		lineWalk.EndU, lineWalk.EndV, lineWalk.EndBound) &&
		endpointsClose(arcWalk.EndU, arcWalk.EndV, arcWalk.EndBound,
			lineWalk.StartU, lineWalk.StartV, lineWalk.StartBound)) &&
		!(endpointsClose(arcWalk.StartU, arcWalk.StartV, arcWalk.StartBound,
			lineWalk.StartU, lineWalk.StartV, lineWalk.StartBound) &&
			endpointsClose(arcWalk.EndU, arcWalk.EndV, arcWalk.EndBound,
				lineWalk.EndU, lineWalk.EndV, lineWalk.EndBound)) {
		return sectionrecord.CircleSeg{}, false, false, nil
	}
	radius := proofbound.AbsSumUpper(arcWalk.Radius, arcWalk.RadiusBound)
	if proofbound.IsNonFinite(radius) || radius <= 0 {
		return sectionrecord.CircleSeg{}, false, false, nil
	}
	return sectionrecord.CircleSeg{
		Center: arc.Center, Radius: units.Millimeters(radius), CCW: true, TStart: 0, TEnd: 1,
	}, false, true, nil
}

func wholeSectionSegment(segment sectionrecord.CurveSegment) bool {
	var start, end float64
	switch segment := segment.(type) {
	case sectionrecord.LineSeg:
		start, end = segment.TStart, segment.TEnd
	case sectionrecord.CircleSeg:
		start, end = segment.TStart, segment.TEnd
	case sectionrecord.ArcSeg:
		start, end = segment.TStart, segment.TEnd
	default:
		return false
	}
	return start == 0 && end == 1 || start == 1 && end == 0
}

// mapDToolHole maps a clipped circle and its chord to a normalized exact loop.
// The chord endpoints define both records, so Sketch can certify their shared
// junctions without relying on independently rounded trigonometric endpoints.
func mapDToolHole(loop sectionrecord.LoopRecord, toolFrame, loftFrame r3.Frame, level float64) (sectionrecord.LoopRecord, bool) {
	if len(loop.Segments) != 2 {
		return sectionrecord.LoopRecord{}, false
	}
	var circle sectionrecord.CircleSeg
	var line sectionrecord.LineSeg
	hasCircle, hasLine := false, false
	for _, segment := range loop.Segments {
		switch segment := segment.(type) {
		case sectionrecord.CircleSeg:
			circle, hasCircle = segment, true
		case sectionrecord.LineSeg:
			line, hasLine = segment, true
		default:
			return sectionrecord.LoopRecord{}, false
		}
	}
	if !hasCircle || !hasLine {
		return sectionrecord.LoopRecord{}, false
	}
	circleWalk, err := boundarywalk.WalkOf(circle, freeform.NewFreeformWork())
	if err != nil || !circleWalk.IsCircular() || circleWalk.Closed {
		return sectionrecord.LoopRecord{}, false
	}
	lineWalk, err := boundarywalk.WalkOf(line, freeform.NewFreeformWork())
	if err != nil || lineWalk.Closed {
		return sectionrecord.LoopRecord{}, false
	}
	center, ok := mapToolPoint(circle.Center, toolFrame, loftFrame, level)
	if !ok {
		return sectionrecord.LoopRecord{}, false
	}
	lineStart, ok := mapToolPoint(sectionrecord.Point2{U: lineWalk.StartU, V: lineWalk.StartV},
		toolFrame, loftFrame, level)
	if !ok {
		return sectionrecord.LoopRecord{}, false
	}
	lineEnd, ok := mapToolPoint(sectionrecord.Point2{U: lineWalk.EndU, V: lineWalk.EndV},
		toolFrame, loftFrame, level)
	if !ok {
		return sectionrecord.LoopRecord{}, false
	}
	// The source loop walks the chord from circle.End to circle.Start, then the
	// CCW circle arc from Start to End. Reverse it for a hole.
	allow := proofbound.AbsSumUpper(proofbound.WalkEndBoundAllow(lineWalk.StartBound),
		proofbound.WalkEndBoundAllow(lineWalk.EndBound), proofbound.WalkEndBoundAllow(circleWalk.StartBound),
		proofbound.WalkEndBoundAllow(circleWalk.EndBound), proofbound.UlpOf(circleWalk.Radius))
	if proofbound.IsNonFinite(allow) ||
		math.Hypot(lineWalk.StartU-circleWalk.EndU, lineWalk.StartV-circleWalk.EndV) > allow ||
		math.Hypot(lineWalk.EndU-circleWalk.StartU, lineWalk.EndV-circleWalk.StartV) > allow {
		return sectionrecord.LoopRecord{}, false
	}
	return sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.ArcSeg{Center: center, Start: lineEnd, End: lineStart, TStart: 1, TEnd: 0},
		sectionrecord.LineSeg{Start: lineEnd, End: lineStart, TStart: 0, TEnd: 1},
	}}, true
}

func mapToolPoint(p sectionrecord.Point2, toolFrame, loftFrame r3.Frame, level float64) (sectionrecord.Point2, bool) {
	world := toolFrame.ToWorldUV(p.U, p.V)
	local := loftFrame.ToLocal(world)
	if local.Z != level || !loftToolPointExact(toolFrame, loftFrame, p.U, p.V, local) {
		return sectionrecord.Point2{}, false
	}
	return sectionrecord.Point2{U: local.X, V: local.Y}, true
}

// The candidate local coordinates are rounded readings. Compare their exact
// lifts with the recorded tool center before reusing them in a hole record.
func loftToolCenterExact(tool prismrecord.Payload, frame r3.Frame, u, v float64, candidate r3.Vec) bool {
	return loftToolPointExact(tool.Frame, frame, u, v, candidate)
}

func loftToolPointExact(toolFrame, frame r3.Frame, u, v float64, candidate r3.Vec) bool {
	toolCenter, err := massmoment.RevolveAnchor(toolFrame, u, v)
	if err != nil {
		return false
	}
	loftCenter, err := massmoment.RevolveAnchor(frame, candidate.X, candidate.Y)
	if err != nil {
		return false
	}
	axis, err := massmoment.PlaneMap(frame, r3.Identity())
	if err != nil {
		return false
	}
	z := proofarith.FloatRat(candidate.Z)
	if z == nil {
		return false
	}
	for i := range toolCenter {
		lift := new(big.Rat).Add(loftCenter[i], new(big.Rat).Mul(axis[i][2], z))
		if toolCenter[i].Cmp(lift) != 0 {
			return false
		}
	}
	return true
}

// The tool's exact endpoints must lie on or beyond the cap planes. A dot
// with the lower frame's exact normal decides each side without a rounded
// local-coordinate comparison.
func loftToolSpansExactCaps(tool prismrecord.Payload, lower, upper r3.Frame) bool {
	toolOrigin, err := massmoment.RevolveAnchor(tool.Frame, 0, 0)
	if err != nil {
		return false
	}
	bottom, err := massmoment.RevolveAnchor(lower, 0, 0)
	if err != nil {
		return false
	}
	top, err := massmoment.RevolveAnchor(upper, 0, 0)
	if err != nil {
		return false
	}
	toolMap, err := massmoment.PlaneMap(tool.Frame, r3.Identity())
	if err != nil {
		return false
	}
	loftMap, err := massmoment.PlaneMap(lower, r3.Identity())
	if err != nil {
		return false
	}
	for end, endCap := range [2][3]*big.Rat{bottom, top} {
		z := proofarith.FloatRat(tool.Z0)
		if end == 1 {
			z = proofarith.FloatRat(tool.Z1)
		}
		if z == nil {
			return false
		}
		side := new(big.Rat)
		for i := range toolOrigin {
			point := new(big.Rat).Add(toolOrigin[i], new(big.Rat).Mul(toolMap[i][2], z))
			delta := new(big.Rat).Sub(point, endCap[i])
			side.Add(side, new(big.Rat).Mul(delta, loftMap[i][2]))
		}
		if (end == 0 && side.Sign() > 0) || (end == 1 && side.Sign() < 0) {
			return false
		}
	}
	return true
}
