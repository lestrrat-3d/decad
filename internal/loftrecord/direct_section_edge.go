package loftrecord

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

type directRatPoint [3]*big.Rat

func (p Payload) directSectionLine(ctx context.Context, body *topo.Body, section, segment int,
	budget *proofbound.WorkBudget) (directRatPoint, directRatPoint, bool, error) {
	var zero directRatPoint
	if ctx == nil || budget == nil {
		return zero, zero, false, fmt.Errorf("direct section line: %w", decaderr.ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return zero, zero, false, err
	}
	if body == nil || p.SourceIdentity == nil || p.SourceIdentity.Origin != body.Origin.Producer ||
		!p.MatchesDirectSections(body) || section < 0 || section > 1 {
		return zero, zero, false, nil
	}
	profile, frame := p.Profile0, p.Frame0
	if section == 1 {
		profile, frame = p.Profile1, p.Frame1
	}
	if !frame.IsValid() || segment < 0 || segment >= len(profile.Outer.Segments) {
		return zero, zero, false, nil
	}
	line, ok := profile.Outer.Segments[segment].(sectionrecord.LineSeg)
	if !ok || !((line.TStart == 0 && line.TEnd == 1) || (line.TStart == 1 && line.TEnd == 0)) {
		return zero, zero, false, nil
	}
	if err := budget.Step(); err != nil {
		return zero, zero, false, err
	}
	a, err := massmoment.RevolveAnchor(frame, line.Start.U, line.Start.V)
	if err != nil {
		return zero, zero, false, err
	}
	b, err := massmoment.RevolveAnchor(frame, line.End.U, line.End.V)
	if err != nil {
		return zero, zero, false, err
	}
	if !distinctDirectPoints(a, b) {
		return zero, zero, false, nil
	}
	return a, b, true, nil
}

func distinctDirectPoints(a, b directRatPoint) bool {
	return a[0].Cmp(b[0]) != 0 || a[1].Cmp(b[1]) != 0 || a[2].Cmp(b[2]) != 0
}

func directPoint(v r3.Vec) directRatPoint {
	return directRatPoint{new(big.Rat).SetFloat64(v.X), new(big.Rat).SetFloat64(v.Y), new(big.Rat).SetFloat64(v.Z)}
}

func directPointWithinSegment(a, b, p directRatPoint) bool {
	var d, q directRatPoint
	length, dot := new(big.Rat), new(big.Rat)
	for i := range 3 {
		d[i] = new(big.Rat).Sub(b[i], a[i])
		q[i] = new(big.Rat).Sub(p[i], a[i])
		length.Add(length, new(big.Rat).Mul(d[i], d[i]))
		dot.Add(dot, new(big.Rat).Mul(d[i], q[i]))
	}
	if length.Sign() == 0 || dot.Sign() < 0 || dot.Cmp(length) > 0 {
		return false
	}
	for i := range 3 {
		j, k := (i+1)%3, (i+2)%3
		cross := new(big.Rat).Sub(new(big.Rat).Mul(d[j], q[k]), new(big.Rat).Mul(d[k], q[j]))
		if cross.Sign() != 0 {
			return false
		}
	}
	return true
}

// DirectStartWallTriangleExact binds a start-edge wall facet to the exact
// endpoints of its paired whole source lines. It refuses any rounded lift.
func (p Payload) DirectStartWallTriangleExact(ctx context.Context, body *topo.Body, mesh *topo.Mesh,
	facet, cell, side int, prepared [3]r3.Vec, budget *proofbound.WorkBudget) (bool, error) {
	if side != 0 || len(p.Profile0.Outer.Segments) != len(p.Profile1.Outer.Segments) {
		return false, nil
	}
	segment, ok := p.DirectWallFacetSourceSegment(body, mesh, facet, cell, side, prepared)
	if !ok {
		return false, nil
	}
	offset := 0
	if len(p.Alignment) == 1 {
		offset = p.Alignment[0]
	}
	n := len(p.Profile0.Outer.Segments)
	if n == 0 || offset < 0 || offset >= n {
		return false, nil
	}
	upper := (segment + offset) % n
	for section, index := range [2]int{segment, upper} {
		profile := p.Profile0
		if section == 1 {
			profile = p.Profile1
		}
		line, ok := profile.Outer.Segments[index].(sectionrecord.LineSeg)
		if !ok || line.TStart != 0 || line.TEnd != 1 {
			return false, nil
		}
		var next sectionrecord.Point2
		switch s := profile.Outer.Segments[(index+1)%n].(type) {
		case sectionrecord.LineSeg:
			if s.TStart != 0 || s.TEnd != 1 {
				return false, nil
			}
			next = s.Start
		case sectionrecord.ArcSeg:
			if s.TStart != 0 || s.TEnd != 1 {
				return false, nil
			}
			next = s.Start
		default:
			return false, nil
		}
		if line.End != next {
			return false, nil
		}
	}
	a, b, ok, err := p.directSectionLine(ctx, body, 0, segment, budget)
	if err != nil || !ok {
		return false, err
	}
	_, c, ok, err := p.directSectionLine(ctx, body, 1, upper, budget)
	if err != nil || !ok {
		return false, err
	}
	want := [3]directRatPoint{a, b, c}
	var used [3]bool
	for _, point := range prepared {
		if !proofbound.FiniteVec(point) {
			return false, nil
		}
		actual := directPoint(point)
		found := false
		for j := range want {
			if !used[j] && !distinctDirectPoints(actual, want[j]) {
				used[j], found = true, true
				break
			}
		}
		if !found {
			return false, nil
		}
	}
	return true, budget.Err()
}

// DirectSectionLineWithinEdge checks exact containment of an authenticated
// source line in a retained topological edge. It does not classify contact.
func (p Payload) DirectSectionLineWithinEdge(ctx context.Context, body *topo.Body, section, segment int,
	edge *topo.Edge, budget *proofbound.WorkBudget) (bool, error) {
	a, b, ok, err := p.directSectionLine(ctx, body, section, segment, budget)
	if err != nil || !ok {
		return false, err
	}
	if edge == nil || edge.Start == nil || edge.End == nil || !edge.RetainedCurve {
		return false, nil
	}
	if _, ok := edge.Curve.(surfacegeom.Line3); !ok {
		return false, nil
	}
	if edge.Start.Bound.Kind() != units.Length || edge.End.Bound.Kind() != units.Length ||
		edge.Start.Bound.Base() != 0 || edge.End.Bound.Base() != 0 ||
		!proofbound.FiniteVec(edge.Start.Position) || !proofbound.FiniteVec(edge.End.Position) {
		return false, nil
	}
	start, end := directPoint(edge.Start.Position), directPoint(edge.End.Position)
	result := directPointWithinSegment(start, end, a) && directPointWithinSegment(start, end, b)
	if err := budget.Err(); err != nil {
		return false, err
	}
	return result, nil
}

// DirectSectionLineContainsPoints checks whether two distinct held points lie
// on an authenticated source line segment. It does not bind points to facets
// or classify Boolean contact.
func (p Payload) DirectSectionLineContainsPoints(ctx context.Context, body *topo.Body, section, segment int,
	points [2]r3.Vec, budget *proofbound.WorkBudget) (bool, error) {
	a, b, ok, err := p.directSectionLine(ctx, body, section, segment, budget)
	if err != nil || !ok {
		return false, err
	}
	if !proofbound.FiniteVec(points[0]) || !proofbound.FiniteVec(points[1]) {
		return false, nil
	}
	x, y := directPoint(points[0]), directPoint(points[1])
	result := distinctDirectPoints(x, y) && directPointWithinSegment(a, b, x) && directPointWithinSegment(a, b, y)
	if err := budget.Err(); err != nil {
		return false, err
	}
	return result, nil
}

// DirectSectionLineMatchesPoints checks exact endpoint equality, in either
// order, with a named authenticated whole source segment. It does not bind
// the points to a mesh or classify contact.
func (p Payload) DirectSectionLineMatchesPoints(ctx context.Context, body *topo.Body, section, segment int,
	points [2]r3.Vec, budget *proofbound.WorkBudget) (bool, error) {
	a, b, ok, err := p.directSectionLine(ctx, body, section, segment, budget)
	if err != nil || !ok {
		return false, err
	}
	if !proofbound.FiniteVec(points[0]) || !proofbound.FiniteVec(points[1]) {
		return false, nil
	}
	x, y := directPoint(points[0]), directPoint(points[1])
	equal := func(a, b directRatPoint) bool { return !distinctDirectPoints(a, b) }
	result := equal(a, x) && equal(b, y) || equal(a, y) && equal(b, x)
	if err := budget.Err(); err != nil {
		return false, err
	}
	return result, nil
}
