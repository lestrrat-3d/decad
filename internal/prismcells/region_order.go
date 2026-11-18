package prismcells

import (
	"math/big"
	"slices"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// orderPoint is one arranged vertex along a source segment: its parameter on
// the entity, its exact point and its two charges.
type orderPoint struct {
	t      float64
	p      [2]*big.Rat
	l, err float64
	moves  bool // L_v > 0
}

// Ordered is V3 (agent-docs/prism-boolean-design.md Part K §5.3): along every
// source segment that carries an arranged vertex with L_v > 0, each two
// consecutive arranged vertices v, w satisfy
// |P̂_v − P̂_w| > 4·(L_v + 2·err_v + L_w + 2·err_w + 2h), h the segment's own.
// A segment's own read ends count as vertices with L = err = 0. ok=false is
// a miss.
func (arr *Arrangement) Ordered(budget *proofbound.WorkBudget) (bool, error) {
	if !guardOn("V3") {
		return true, nil
	}
	type entityPoints struct {
		pts    []orderPoint
		src    sectionrecord.CurveSegment
		h      float64
		moving bool
	}
	byEntity := map[sketch.Entity]*entityPoints{}
	var order []sketch.Entity
	for i := range arr.edges {
		if err := budget.Step(); err != nil {
			return false, err
		}
		e := &arr.edges[i]
		ep, ok := byEntity[e.entity]
		if !ok {
			ep = &entityPoints{src: e.src, h: e.srcH}
			byEntity[e.entity] = ep
			order = append(order, e.entity)
		}
		for _, end := range []struct {
			at [2]float64
			t  float64
		}{{e.start, e.t0}, {e.end, e.t1}} {
			if vx, ok := arr.vr.Vertices[end.at]; ok {
				ep.pts = append(ep.pts, orderPoint{t: end.t, p: vx.Point, l: vx.L, err: vx.Err, moves: vx.L > 0})
				ep.moving = ep.moving || vx.L > 0
				continue
			}
			p, ok := pointRat(end.at)
			if c, circle := e.src.(sectionrecord.CircleSeg); circle && (end.t == 0 || end.t == 1) {
				// the seam is the exact point of the recorded circle at angle 0
				r, err := c.Radius.In(units.Millimeter)
				cu, cv, rr := proofarith.FloatRat(c.Center.U), proofarith.FloatRat(c.Center.V), proofarith.FloatRat(r)
				if err != nil || cu == nil || cv == nil || rr == nil {
					return false, nil //nolint:nilerr // an unreadable circle is a miss
				}
				p, ok = [2]*big.Rat{new(big.Rat).Add(cu, rr), cv}, true
			}
			if !ok {
				return false, nil
			}
			ep.pts = append(ep.pts, orderPoint{t: end.t, p: p, err: e.ownEndErr()})
		}
	}
	for _, ent := range order {
		ep := byEntity[ent]
		if !ep.moving {
			continue
		}
		pts := ep.pts
		cyclic := false
		switch s := ep.src.(type) {
		case sectionrecord.LineSeg:
			a, ok1 := pointRat([2]float64{s.Start.U, s.Start.V})
			b, ok2 := pointRat([2]float64{s.End.U, s.End.V})
			if !ok1 || !ok2 {
				return false, nil
			}
			pts = append(pts, orderPoint{t: 0, p: a}, orderPoint{t: 1, p: b})
		case sectionrecord.ArcSeg:
			a, ok1 := pointRat([2]float64{s.Start.U, s.Start.V})
			b, ok2 := pointRat([2]float64{s.End.U, s.End.V})
			if !ok1 || !ok2 {
				return false, nil
			}
			pts = append(pts, orderPoint{t: 0, p: a}, orderPoint{t: 1, p: b, err: arcResidual(s)})
		default:
			cyclic = true
			for i := range pts {
				if pts[i].t == 1 {
					pts[i].t = 0 // a circle's two bounds are one point
				}
			}
		}
		slices.SortStableFunc(pts, func(a, b orderPoint) int {
			switch {
			case a.t < b.t:
				return -1
			case a.t > b.t:
				return 1
			}
			return 0
		})
		pts = slices.CompactFunc(pts, func(a, b orderPoint) bool { return a.t == b.t })
		n := len(pts)
		pairs := n - 1
		if cyclic && guardOn("V3cyclic") {
			pairs = n // a whole circle's last vertex pairs with its first
		}
		for i := range pairs {
			v, w := pts[i], pts[(i+1)%n]
			if n < 2 {
				break
			}
			need := proofbound.ProductUpper(4, proofbound.AbsSumUpper(v.l, 2*v.err, w.l, 2*w.err, 2*ep.h))
			if !(ratPointDistLower(v.p, w.p) > need) {
				return false, nil
			}
		}
	}
	return true, nil
}

// ownEndErr is how far the point V3 reads for an edge's own end (the
// arranger's polyline point) can lie from the end the walk reads: the walk
// end's allowance, and for an ArcSeg the radial residual ρ, since its read
// end is Start's radius on End's bearing and can sit ρ from the recorded End.
func (e *arrangedEdge) ownEndErr() float64 {
	err := proofbound.AbsSumUpper(proofbound.WalkEndBoundAllow(e.walk.StartBound), proofbound.WalkEndBoundAllow(e.walk.EndBound))
	if arc, ok := e.src.(sectionrecord.ArcSeg); ok {
		err = proofbound.AbsSumUpper(err, arcResidual(arc))
	}
	return err
}

func pointRat(p [2]float64) ([2]*big.Rat, bool) {
	u, v := proofarith.FloatRat(p[0]), proofarith.FloatRat(p[1])
	return [2]*big.Rat{u, v}, u != nil && v != nil
}
