// Package occupancyleaf reads a Boolean operand's payload as an occupancy
// record (agent-docs/body-refine-design.md §4): a solid prism or stack whose
// section regions are whole circles or convex polygons with whole circular
// holes, a Revolve of one whole circle clear of its axis, a Sweep of one whole
// circle whose spans are lines and arcs, a Coil of one whole circle, or a
// faceted result that already carries a record for its own held mesh. Every
// other payload has none.
package occupancyleaf

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/coilshell"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/occupancy"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/sweeprecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Sides is the circumscribed polygon of every circle leaf's outer piece. Its
// widening of the radius, r·(1/cos(π/128) − 1) ≈ 3e-4·r, enters every
// certified bound as a lower limit.
const Sides = 128

// relativeSlack widens a leaf's piece past the record's floats, relative to
// the section's own magnitudes. The record denotes the exact composite of its
// stored frame and placement (prismrecord.Payload.Point), so the slack covers
// nothing the record leaves open; it keeps the containment robust to a later
// change in how the frame's normal is read.
const relativeSlack = 1e-12

// Of returns body's occupancy record for the operand mesh the Boolean used,
// or nil when the payload is outside the input class.
func Of(body *topo.Body, mesh *topo.Mesh) *occupancy.Record {
	if body == nil || mesh == nil || body.Kind != topo.BodySolid {
		return nil
	}
	switch p := body.Payload.(type) {
	case facetedrecord.Payload:
		return p.OccupancyRecord()
	case prismrecord.Payload:
		if p.SurfaceResult {
			return nil
		}
		if c, ok := wholeCircle(p.Profile); ok && len(p.Profile.Holes) == 0 {
			return leaf(p.Frame, p.Xform, c, p.Z0, p.Z1, p.SectionDelta, p.Z0Delta, p.Z1Delta, mesh)
		}
		return prismWithHoles(p, mesh.Bound)
	case revolveplan.Payload:
		l, ok := revolveLeaf(p, mesh.Bound)
		if !ok {
			return nil
		}
		return occupancy.LeafRecord(l)
	case sweeprecord.Payload:
		return sweepRecord(p, mesh)
	case coilshell.Payload:
		l, ok := coilLeaf(p, mesh.Bound)
		if !ok {
			return nil
		}
		return occupancy.LeafRecord(l)
	case stackedrecord.Payload:
		if p.ClosedShell || len(p.Slabs) == 0 {
			return nil
		}
		if rec := coaxialStack(p, mesh); rec != nil {
			return rec
		}
		return stackRegions(p, mesh.Bound)
	}
	return nil
}

// coaxialStack reads a stack whose every slab is one whole circle of the same
// centre and radius at contiguous levels as a single cylinder leaf.
func coaxialStack(p stackedrecord.Payload, mesh *topo.Mesh) *occupancy.Record {
	var c sectionrecord.CircleSeg
	dz0, dz1 := 0.0, 0.0
	for i, s := range p.Slabs {
		if len(s.Regions) != 1 || len(s.Regions[0].Holes) != 0 {
			return nil
		}
		ci, ok := wholeCircle(s.Regions[0])
		if !ok || i > 0 && (ci != c || p.Slabs[i-1].Z1 != s.Z0) {
			return nil
		}
		c = ci
		dz0, dz1 = max(dz0, s.Z0Delta), max(dz1, s.Z1Delta)
	}
	return leaf(p.Frame, p.Xform, c, p.Slabs[0].Z0, p.Slabs[len(p.Slabs)-1].Z1, p.SectionDelta, dz0, dz1, mesh)
}

// stackRegions reads a stack slab by slab, region by region: every region's
// outer loop a leaf and every hole a hole leaf through that slab's levels.
func stackRegions(p stackedrecord.Payload, delta float64) *occupancy.Record {
	var outers, holes []occupancy.Leaf
	for _, s := range p.Slabs {
		for _, region := range s.Regions {
			outer, hs, ok := regionLeaves(p.Frame, p.Xform, region, s.Z0, s.Z1, p.SectionDelta, s.Z0Delta, s.Z1Delta, delta)
			if !ok {
				return nil
			}
			outers = append(outers, outer)
			holes = append(holes, hs...)
		}
	}
	if len(outers) == 0 {
		return nil
	}
	return occupancy.PrismRecord(outers, holes)
}

func wholeCircle(pr momentinput.Profile) (sectionrecord.CircleSeg, bool) {
	if len(pr.Outer.Segments) != 1 {
		return sectionrecord.CircleSeg{}, false
	}
	c, ok := pr.Outer.Segments[0].(sectionrecord.CircleSeg)
	return c, ok && c.CCW && c.TStart == 0 && c.TEnd == 1
}

// wholeHoleCircle reads a hole loop of one whole circle, which a section
// records clockwise, from parameter 1 down to 0, as often as the other way.
func wholeHoleCircle(loop sectionrecord.LoopRecord) (sectionrecord.CircleSeg, bool) {
	if len(loop.Segments) != 1 {
		return sectionrecord.CircleSeg{}, false
	}
	c, ok := loop.Segments[0].(sectionrecord.CircleSeg)
	return c, ok && (c.CCW && c.TStart == 0 && c.TEnd == 1 || !c.CCW && c.TStart == 1 && c.TEnd == 0)
}

func leaf(frame r3.Frame, place r3.Transform, c sectionrecord.CircleSeg, z0, z1, sectionDelta, dz0, dz1 float64, mesh *topo.Mesh) *occupancy.Record {
	l, ok := prismLeaf(frame, place, c, z0, z1, sectionDelta, dz0, dz1, mesh.Bound)
	if !ok {
		return nil
	}
	return occupancy.LeafRecord(l)
}

func prismLeaf(frame r3.Frame, place r3.Transform, c sectionrecord.CircleSeg, z0, z1, sectionDelta, dz0, dz1, delta float64) (occupancy.Leaf, bool) {
	r, err := c.Radius.In(units.Millimeter)
	if err != nil || !(r > 0) {
		return occupancy.Leaf{}, false
	}
	u, v, n, o := frame.U(), frame.V(), frame.N(), frame.Origin()
	b, t := place.Basis(), place.Translation()
	chain := occupancy.Chain{
		{M: [3][3]float64{{u.X, v.X, n.X}, {u.Y, v.Y, n.Y}, {u.Z, v.Z, n.Z}}, T: [3]float64{o.X, o.Y, o.Z}},
		{M: [3][3]float64{{b.EX.X, b.EY.X, b.EZ.X}, {b.EX.Y, b.EY.Y, b.EZ.Y}, {b.EX.Z, b.EY.Z, b.EZ.Z}}, T: [3]float64{t.X, t.Y, t.Z}},
	}
	scale := 1 + math.Abs(c.Center.U) + math.Abs(c.Center.V) + r + math.Max(math.Abs(z0), math.Abs(z1))
	l, err := occupancy.CircleLeaf(chain, c.Center.U, c.Center.V, r, z0, z1, sectionDelta, dz0, dz1,
		relativeSlack*scale, Sides, delta)
	if err != nil {
		return occupancy.Leaf{}, false
	}
	return l, true
}

// prismWithHoles reads a straight prism whose outer loop is a whole circle or
// a convex polygon of whole line segments, and whose holes are whole circles,
// as its outer leaf less one circle leaf per hole.
func prismWithHoles(p prismrecord.Payload, delta float64) *occupancy.Record {
	outer, holes, ok := regionLeaves(p.Frame, p.Xform, p.Profile, p.Z0, p.Z1, p.SectionDelta, p.Z0Delta, p.Z1Delta, delta)
	if !ok {
		return nil
	}
	return occupancy.PrismRecord([]occupancy.Leaf{outer}, holes)
}

// regionLeaves reads one section region between two levels: its outer loop as
// a circle or convex polygon leaf, its holes as circle leaves.
func regionLeaves(frame r3.Frame, place r3.Transform, pr momentinput.Profile, z0, z1, sectionDelta, dz0, dz1, delta float64) (occupancy.Leaf, []occupancy.Leaf, bool) {
	var outer occupancy.Leaf
	if c, ok := wholeCircle(momentinput.Profile{Outer: pr.Outer}); ok {
		l, ok := prismLeaf(frame, place, c, z0, z1, sectionDelta, dz0, dz1, delta)
		if !ok {
			return occupancy.Leaf{}, nil, false
		}
		outer = l
	} else {
		segs := pr.Outer.Segments
		pts := make([][2]float64, 0, len(segs))
		scale := 1 + math.Max(math.Abs(z0), math.Abs(z1))
		for i, seg := range segs {
			l, ok := seg.(sectionrecord.LineSeg)
			if !ok || l.TStart != 0 || l.TEnd != 1 {
				return occupancy.Leaf{}, nil, false
			}
			// Each segment must end where the next one starts.
			next, ok := segs[(i+1)%len(segs)].(sectionrecord.LineSeg)
			if !ok || l.End != next.Start {
				return occupancy.Leaf{}, nil, false
			}
			pts = append(pts, [2]float64{l.Start.U, l.Start.V})
			scale = math.Max(scale, 1+math.Abs(l.Start.U)+math.Abs(l.Start.V))
		}
		l, err := occupancy.PolygonLeaf(prismChain(frame, place), pts, z0, z1, sectionDelta, dz0, dz1, relativeSlack*scale, delta)
		if err != nil {
			return occupancy.Leaf{}, nil, false
		}
		outer = l
	}
	holes := make([]occupancy.Leaf, 0, len(pr.Holes))
	for _, h := range pr.Holes {
		c, ok := wholeHoleCircle(h)
		if !ok {
			return occupancy.Leaf{}, nil, false
		}
		l, ok := prismLeaf(frame, place, c, z0, z1, sectionDelta, dz0, dz1, delta)
		if !ok {
			return occupancy.Leaf{}, nil, false
		}
		holes = append(holes, l)
	}
	return outer, holes, true
}

func prismChain(frame r3.Frame, place r3.Transform) occupancy.Chain {
	u, v, n, o := frame.U(), frame.V(), frame.N(), frame.Origin()
	b, t := place.Basis(), place.Translation()
	return occupancy.Chain{
		{M: [3][3]float64{{u.X, v.X, n.X}, {u.Y, v.Y, n.Y}, {u.Z, v.Z, n.Z}}, T: [3]float64{o.X, o.Y, o.Z}},
		{M: [3][3]float64{{b.EX.X, b.EY.X, b.EZ.X}, {b.EX.Y, b.EY.Y, b.EZ.Y}, {b.EX.Z, b.EY.Z, b.EZ.Z}}, T: [3]float64{t.X, t.Y, t.Z}},
	}
}

// TubeSides is the circumscribed polygon of every arc tube cell's piece, and
// cellSagitta the largest arc sagitta one cell may span.
const (
	TubeSides   = 64
	cellSagitta = 1e-4
)

// revolveLeaf reads a solid Revolve of one whole circle whose disc stays
// clear of the axis as an arc tube. The record denotes the point
// xform(A3 + W·z + (E0·cos φ + E1·sin φ)·ρ) of its plane-local section
// (revolvemesh.RevolveLift.Point); the disc's centre at axis coordinates
// (zc, rc) sweeps the spine circle. The record's section and axis
// displacements widen the leaf.
func revolveLeaf(p revolveplan.Payload, delta float64) (occupancy.Leaf, bool) {
	if p.SurfaceResult || len(p.Profile.Holes) != 0 {
		return occupancy.Leaf{}, false
	}
	c, ok := wholeCircle(p.Profile)
	if !ok {
		return occupancy.Leaf{}, false
	}
	rho, err := c.Radius.In(units.Millimeter)
	if err != nil || !(rho > 0) {
		return occupancy.Leaf{}, false
	}
	du, dv := c.Center.U-p.Ax.AU, c.Center.V-p.Ax.AV
	zc := du*p.Ax.DU + dv*p.Ax.DV
	rc := -du*p.Ax.DV + dv*p.Ax.DU
	b := p.Basis()
	e0, e1 := b.E0, b.E1
	if rc < 0 {
		rc, e0, e1 = -rc, e0.Scale(-1), e1.Scale(-1)
	}
	if !(rc > rho) {
		return occupancy.Leaf{}, false
	}
	o := p.Xform.Apply(b.A3.Add(b.W.Scale(zc)))
	w, we0, we1 := p.Xform.ApplyDir(b.W), p.Xform.ApplyDir(e0), p.Xform.ApplyDir(e1)
	reach := math.Abs(zc) + rc + rho
	slack := p.SectionDelta + p.Ax.AUBound + p.Ax.AVBound + (p.Ax.DUBound+p.Ax.DVBound)*reach + relativeSlack*(1+reach)
	l, err := occupancy.ArcTubeLeaf(o, w, we0, we1, rc, rho, p.Phi0, p.Phi1, p.Full, slack, cellSagitta, TubeSides, delta)
	if err != nil {
		return occupancy.Leaf{}, false
	}
	return l, true
}

// HelixSides is the circumscribed polygon of every helix cell's piece.
const HelixSides = 32

// coilLeaf reads a Coil of one whole circle as a helix tube: the record's
// axis line, side and plane frame give the screw motion's c, n, e_r and
// e_t = n × e_r (agent-docs/helix-design.md Table CP), then its placement.
// The axis displacements widen the leaf.
func coilLeaf(p coilshell.Payload, delta float64) (occupancy.Leaf, bool) {
	if len(p.Profile.Holes) != 0 || p.Side == 0 {
		return occupancy.Leaf{}, false
	}
	c, ok := wholeCircle(p.Profile)
	if !ok {
		return occupancy.Leaf{}, false
	}
	rho, err := c.Radius.In(units.Millimeter)
	if err != nil || !(rho > 0) {
		return occupancy.Leaf{}, false
	}
	side := float64(p.Side)
	du, dv := c.Center.U-p.Line.AU, c.Center.V-p.Line.AV
	rc := (dv*p.Line.DU - du*p.Line.DV) * side
	zc := du*p.Line.DU + dv*p.Line.DV
	u, v := p.Frame.U(), p.Frame.V()
	n := p.Xform.ApplyDir(u.Scale(p.Line.DU).Add(v.Scale(p.Line.DV)))
	er := p.Xform.ApplyDir(u.Scale(-p.Line.DV * side).Add(v.Scale(p.Line.DU * side)))
	o := p.Xform.Apply(p.Frame.ToWorldUV(p.Line.AU, p.Line.AV))
	sigma := 1.0
	if p.LeftHand {
		sigma = -1
	}
	k := p.Pitch / (2 * math.Pi)
	span := 2 * math.Pi * p.Turns
	reach := math.Abs(zc) + rc + rho + math.Abs(k)*span
	slack := p.Line.AUBound + p.Line.AVBound + (p.Line.DUBound+p.Line.DVBound)*reach + relativeSlack*(1+reach)
	l, err := occupancy.HelixTubeLeaf(o, n, er, n.Cross(er), sigma, k, rc, zc, rho, span, slack, cellSagitta, HelixSides, delta)
	if err != nil {
		return occupancy.Leaf{}, false
	}
	return l, true
}

// sweepRecord reads a solid Sweep of one whole circle: the union of its spans,
// each a straight prism or an arc Revolve reduction, all standing for the one
// held mesh.
func sweepRecord(p sweeprecord.Payload, mesh *topo.Mesh) *occupancy.Record {
	if p.SurfaceResult || p.ClosedCircle || p.ClosedComposite {
		return nil
	}
	type span struct {
		prism   prismrecord.Payload
		revolve revolveplan.Payload
		arc     bool
	}
	spans := []span{{p.Prism, p.Revolve, p.Arc}}
	if len(p.Spans) != 0 {
		spans = spans[:0]
		for _, s := range p.Spans {
			spans = append(spans, span{s.Prism, s.Revolve, s.Arc})
		}
	}
	var leaves []occupancy.Leaf
	for _, s := range spans {
		if s.arc {
			l, ok := revolveLeaf(s.revolve, mesh.Bound)
			if !ok {
				return nil
			}
			leaves = append(leaves, l)
			continue
		}
		pp := s.prism
		if pp.SurfaceResult || len(pp.Profile.Holes) != 0 {
			return nil
		}
		c, ok := wholeCircle(pp.Profile)
		if !ok {
			return nil
		}
		l, ok := prismLeaf(pp.Frame, pp.Xform, c, pp.Z0, pp.Z1, pp.SectionDelta, pp.Z0Delta, pp.Z1Delta, mesh.Bound)
		if !ok {
			return nil
		}
		leaves = append(leaves, l)
	}
	return occupancy.LeafRecord(leaves...)
}

// Bulges reads λ(t) for every facet of mesh under rec (occupancy.Record.Bulges),
// with each facet's own published bound: its corners' largest vertex bound
// when the mesh publishes a per-vertex record, else its source face's bound.
func Bulges(rec *occupancy.Record, mesh *topo.Mesh) []float64 {
	return rec.Bulges(mesh.Vertices, mesh.Triangles, Deltas(mesh))
}

// Deltas reads every facet's own published bound: its corners' largest vertex
// bound when the mesh publishes a per-vertex record, else its source face's
// bound.
func Deltas(mesh *topo.Mesh) []float64 {
	delta := make([]float64, len(mesh.Triangles))
	for t, tri := range mesh.Triangles {
		if mesh.VertexBound != nil {
			delta[t] = max(mesh.VertexBound[tri[0]], mesh.VertexBound[tri[1]], mesh.VertexBound[tri[2]])
			continue
		}
		d, ok := mesh.FaceBound[mesh.Source[t]]
		if !ok {
			d = mesh.Bound
		}
		delta[t] = d
	}
	return delta
}
