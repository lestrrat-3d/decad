package brepmodify

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/featureoption"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/rimoffset"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// FreeformRimChamfer chamfers the complete outer cap loop of a straight prism whose section holds fit
// splines, the receiver the cap-loop chamfer refuses (agent-docs/modify-reach-design.md §8.5). Decad
// cannot offset a fit spline exactly, so each selected rim is cut by a band-shaped loft cutter. The band's
// inner wall runs between two offsets of the recorded outline: the outline moved into the material by the setback
// plus an overshoot, at a level that overshoot beyond the cap, and the outline moved away from the material by
// the overshoot, at a level that overshoot beyond the bevel's far edge. That wall is the 45° bevel, and it crosses
// the receiver's side wall at the setback. The offsets are fit splines through points of the exact offset.
//
// The result is a Boolean Cut of the receiver, so every reading it publishes is the cut's own. The bevel is
// an approximation of the ideal chamfer to within rimoffset.MaxDeviationRatio of the setback, which the offset
// measures and refuses beyond; nothing certifies that the body matches the ideal chamfer.
//
// It reports false, leaving the ordinary cap-loop route in charge, for any receiver or selection it does not
// recognise: a payload that is not a plain prism, a section with no fit spline, an unequal chamfer, or a
// selection that is not the whole outer loop of a cap.
func FreeformRimChamfer(ctx context.Context, env topo.Env, hooks Hooks, body *topo.Body, edges []*topo.Edge,
	d, dDelta float64, asym *Asymmetric) (Result, bool, error) {
	pp, ok := body.Payload.(prismrecord.Payload)
	if !ok || asym != nil || dDelta != 0 || pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0 ||
		pp.Xform != r3.Identity() || !hasFitSpline(pp.Profile.Outer) {
		return Result{}, false, nil
	}
	rims, ok := selectedRims(body, edges)
	if !ok {
		return Result{}, false, nil
	}
	height := pp.Z1 - pp.Z0
	if d >= height || (len(rims) == 2 && 2*d >= height) {
		return Result{}, true, fmt.Errorf(`%w: the rim chamfer of %.6g mm reaches the far end of a %.6g mm sweep`,
			decaderr.ErrUnsupported, d, height)
	}
	if err := ctx.Err(); err != nil {
		return Result{}, true, err
	}
	span := 0
	var last error
	for _, attempt := range rimAttempts {
		out, used, retry, err := cutRims(ctx, env, hooks, body, pp, rims, d, attempt, span)
		span += used
		if err == nil {
			return Result{Body: out, Span: span}, true, nil
		}
		last = err
		if !retry || ctx.Err() != nil || !errors.Is(err, decaderr.ErrUnsupported) {
			break
		}
	}
	return Result{}, true, last
}

// rimAttempt is one choice of the cutter's free dimensions. Overshoot is how far, as a fraction of the setback,
// the cutter reaches past the cap and past the bevel's far edge. Scale is how many times the farthest point of
// the outline the cutter's outer circle lies from the middle of the section.
type rimAttempt struct{ overshoot, scale float64 }

// rimAttempts are the choices a chamfer tries in turn, while its cutter or its Cut is refused as unsupported.
//
// The Boolean meshes both operands at a chord tolerance that follows the pair's size, so the smaller the
// cutter's circle, the finer the chamfered body is held. A circle close to the outline gives the loft a thin ring
// whose chord target is fine and whose station cap and work ceiling a long outline can exceed. The Boolean can
// also refuse a cutter whose crossings fall within rounding of the receiver's mesh edges, and which cutters do
// depends on the overshoot. The first attempt holds the body finest; the last is the one most likely to build.
var rimAttempts = []rimAttempt{{0.5, 1.5}, {0.35, 1.5}, {0.5, 2}, {0.5, 3}}

// cutRims cuts every rim from body with cutters that reach factor·d past the cap and past the bevel's far
// edge, under producer identities from env.Ref+first on. It returns the body, how many identities it used, and
// whether an error came from the Boolean.
func cutRims(ctx context.Context, env topo.Env, hooks Hooks, body *topo.Body, pp prismrecord.Payload, rims []rim,
	d float64, attempt rimAttempt, first int) (*topo.Body, int, bool, error) {
	over := attempt.overshoot * d
	tol := rimoffset.MaxDeviationRatio * d
	into, err := rimoffset.OffsetLoop(pp.Profile.Outer, d+over, tol)
	if err != nil {
		return nil, 0, false, rimRefusal(err)
	}
	away, err := rimoffset.OffsetLoop(pp.Profile.Outer, -over, tol)
	if err != nil {
		return nil, 0, false, rimRefusal(err)
	}
	outer := rimBoundary(pp.Profile.Outer, attempt.scale)
	used := 0
	next := func() topo.Env {
		step := topo.Env{Doc: env.Doc, Ref: env.Ref + provenance.Producer(first+used), Alloc: env.Alloc}
		used++
		return step
	}
	out := body
	for _, r := range rims {
		// The cutter runs from the level beyond the cap, where its hole is the deepest offset, to the level
		// beyond the bevel's far edge, where its hole is the outline pushed outward.
		capLevel, bevelLevel := pp.Z0-over, pp.Z0+d+over
		if !r.start {
			capLevel, bevelLevel = pp.Z1+over, pp.Z1-d-over
		}
		cutter, err := rimCutter(ctx, next(), pp, capLevel, bevelLevel, outer, into.Segments, away.Segments)
		if err != nil {
			return nil, used, true, err
		}
		if out, err = hooks.Boolean(ctx, meshbool.OpCut, next(), out, cutter); err != nil {
			return nil, used, true, err
		}
	}
	return out, used, false, ctx.Err()
}

// rimRefusal words an offset refusal as the chamfer's own, keeping its sentinel.
func rimRefusal(err error) error {
	return fmt.Errorf("the free-form rim chamfer: %w", err)
}

// rim is one selected cap loop.
type rim struct{ start bool }

// selectedRims reports which caps' outer loops edges covers whole, and false when the selection is anything
// else.
func selectedRims(body *topo.Body, edges []*topo.Edge) ([]rim, bool) {
	caps := CapsOf(body)
	selected := make(map[*topo.Edge]struct{}, len(edges))
	for _, e := range edges {
		selected[e] = struct{}{}
	}
	var rims []rim
	for _, c := range []struct {
		face  *topo.Face
		start bool
	}{{caps.Start, true}, {caps.End, false}} {
		if c.face == nil || len(c.face.Loops) == 0 || !c.face.Loops[0].Outer {
			return nil, false
		}
		loopEdges := c.face.Loops[0].CoEdges
		covered := 0
		for _, ce := range loopEdges {
			if _, ok := selected[ce.Edge]; ok {
				covered++
			}
		}
		switch covered {
		case 0:
		case len(loopEdges):
			rims = append(rims, rim{start: c.start})
			for _, ce := range loopEdges {
				delete(selected, ce.Edge)
			}
		default:
			return nil, false
		}
	}
	if len(rims) == 0 || len(selected) != 0 {
		return nil, false
	}
	return rims, true
}

func hasFitSpline(loop sectionrecord.LoopRecord) bool {
	for _, segment := range loop.Segments {
		if _, ok := segment.(sectionrecord.FitSplineSeg); ok {
			return true
		}
	}
	return false
}

// rimBoundary returns the circle that bounds every cutter ring, as a closed loop of its own: a circle about
// the middle of the outline's box whose radius is scale times the outline's farthest point from that middle.
func rimBoundary(loop sectionrecord.LoopRecord, scale float64) []sectionrecord.CurveSegment {
	var pts []sectionrecord.Point2
	for _, segment := range loop.Segments {
		switch s := segment.(type) {
		case sectionrecord.FitSplineSeg:
			pts = append(pts, s.Fit...)
		case sectionrecord.LineSeg:
			pts = append(pts, s.Start, s.End)
		}
	}
	lo, hi := pts[0], pts[0]
	for _, q := range pts {
		lo.U, lo.V = math.Min(lo.U, q.U), math.Min(lo.V, q.V)
		hi.U, hi.V = math.Max(hi.U, q.U), math.Max(hi.V, q.V)
	}
	center := sectionrecord.Point2{U: (lo.U + hi.U) / 2, V: (lo.V + hi.V) / 2}
	farthest := 0.0
	for _, q := range pts {
		farthest = math.Max(farthest, math.Hypot(q.U-center.U, q.V-center.V))
	}
	return []sectionrecord.CurveSegment{sectionrecord.CircleSeg{
		Center: center, Radius: units.Millimeters(scale * farthest), CCW: true, TStart: 0, TEnd: 2 * math.Pi,
	}}
}

// rimCutter lofts the band between two private sketches: outer around hole0 at level z0 and around hole1 at
// level z1, each drawn in the receiver's frame.
func rimCutter(ctx context.Context, env topo.Env, pp prismrecord.Payload, z0, z1 float64,
	outer, hole0, hole1 []sectionrecord.CurveSegment) (*topo.Body, error) {
	s0, p0, err := rimRing(ctx, pp, z0, outer, hole0)
	if err != nil {
		return nil, err
	}
	s1, p1, err := rimRing(ctx, pp, z1, outer, hole1)
	if err != nil {
		return nil, err
	}
	cutter, err := loftrecord.FromProfiles(ctx, env, s0, p0, s1, p1, featureoption.LoftConfig{})
	if err != nil {
		return nil, fmt.Errorf("the free-form rim chamfer's cutter: %w", err)
	}
	return cutter, nil
}

// rimRing sketches the outer loop and the hole at level z of the receiver's frame, and returns the one
// profile that is the band between them.
func rimRing(ctx context.Context, pp prismrecord.Payload, z float64, outer, hole []sectionrecord.CurveSegment) (
	*sketch.Sketch, *sketch.Profile, error) {
	frame, err := r3.NewFrame(pp.Frame.Origin().Add(pp.Frame.N().Scale(z)), pp.Frame.U(), pp.Frame.V())
	if err != nil {
		return nil, nil, fmt.Errorf("%w: the free-form rim chamfer has no cutter frame: %v", decaderr.ErrUnsupported, err)
	}
	world := sketch.NewWorld()
	plane, err := world.CreatePlaneFromFrame(frame)
	if err != nil {
		return nil, nil, fmt.Errorf("%w: the free-form rim chamfer has no cutter plane: %v", decaderr.ErrUnsupported, err)
	}
	s, err := world.CreateSketch(plane)
	if err != nil {
		return nil, nil, fmt.Errorf("%w: the free-form rim chamfer has no cutter sketch: %v", decaderr.ErrUnsupported, err)
	}
	for _, loop := range [][]sectionrecord.CurveSegment{outer, hole} {
		if err := drawRimLoop(s, loop); err != nil {
			return nil, nil, err
		}
	}
	if _, err := s.Solve(ctx); err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return nil, nil, ctxErr
		}
		return nil, nil, fmt.Errorf("%w: the free-form rim chamfer's cutter sketch does not solve: %v", decaderr.ErrUnsupported, err)
	}
	for _, candidate := range s.Profiles() {
		if candidate.Valid && len(candidate.Holes) == 1 {
			return s, candidate, nil
		}
	}
	return nil, nil, fmt.Errorf("%w: the free-form rim chamfer's cutter ring is not one valid region", decaderr.ErrUnsupported)
}

// drawRimLoop draws a circle, or a closed chain of fit splines and lines, in s, every point fixed, each segment ending at the
// next one's start and the last at the first's.
func drawRimLoop(s *sketch.Sketch, loop []sectionrecord.CurveSegment) error {
	joint := func(q sectionrecord.Point2) *sketch.Point {
		p := s.CreatePoint(q.U, q.V)
		s.Fix(p)
		return p
	}
	var first, shared *sketch.Point
	for i, segment := range loop {
		if circle, ok := segment.(sectionrecord.CircleSeg); ok {
			center := joint(circle.Center)
			s.CreateCircle(center, circle.Radius.Base())
			continue
		}
		last := i == len(loop)-1
		var from, to *sketch.Point
		switch seg := segment.(type) {
		case sectionrecord.FitSplineSeg:
			pts := make([]*sketch.Point, len(seg.Fit))
			for k, q := range seg.Fit {
				switch {
				case k == 0 && shared != nil:
					pts[k] = shared
				case k == len(seg.Fit)-1 && last:
					pts[k] = first
				default:
					pts[k] = joint(q)
				}
			}
			if _, err := s.CreateFitSpline(pts...); err != nil {
				return fmt.Errorf("%w: the free-form rim chamfer's cutter spline: %v", decaderr.ErrUnsupported, err)
			}
			from, to = pts[0], pts[len(pts)-1]
		case sectionrecord.LineSeg:
			from, to = shared, first
			if from == nil {
				from = joint(seg.Start)
			}
			if !last {
				to = joint(seg.End)
			}
			s.CreateLine(from, to)
		}
		if first == nil {
			first = from
		}
		shared = to
	}
	return nil
}
