// Package occupancyleaf reads a Boolean operand's payload as an occupancy
// record (agent-docs/body-refine-design.md §4): a solid prism of one whole
// circle, a stacked prism whose slabs are one coaxial circle, or a faceted
// Union result that already carries a record for its own held mesh. Every
// other payload has none.
package occupancyleaf

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/occupancy"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
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
		if p.SurfaceResult || len(p.Profile.Holes) != 0 {
			return nil
		}
		c, ok := wholeCircle(p.Profile)
		if !ok {
			return nil
		}
		return leaf(p.Frame, p.Xform, c, p.Z0, p.Z1, p.SectionDelta, p.Z0Delta, p.Z1Delta, mesh)
	case stackedrecord.Payload:
		if p.ClosedShell || len(p.Slabs) == 0 {
			return nil
		}
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
	return nil
}

func wholeCircle(pr momentinput.Profile) (sectionrecord.CircleSeg, bool) {
	if len(pr.Outer.Segments) != 1 {
		return sectionrecord.CircleSeg{}, false
	}
	c, ok := pr.Outer.Segments[0].(sectionrecord.CircleSeg)
	return c, ok && c.CCW && c.TStart == 0 && c.TEnd == 1
}

func leaf(frame r3.Frame, place r3.Transform, c sectionrecord.CircleSeg, z0, z1, sectionDelta, dz0, dz1 float64, mesh *topo.Mesh) *occupancy.Record {
	r, err := c.Radius.In(units.Millimeter)
	if err != nil || !(r > 0) {
		return nil
	}
	u, v, n, o := frame.U(), frame.V(), frame.N(), frame.Origin()
	b, t := place.Basis(), place.Translation()
	chain := occupancy.Chain{
		{M: [3][3]float64{{u.X, v.X, n.X}, {u.Y, v.Y, n.Y}, {u.Z, v.Z, n.Z}}, T: [3]float64{o.X, o.Y, o.Z}},
		{M: [3][3]float64{{b.EX.X, b.EY.X, b.EZ.X}, {b.EX.Y, b.EY.Y, b.EZ.Y}, {b.EX.Z, b.EY.Z, b.EZ.Z}}, T: [3]float64{t.X, t.Y, t.Z}},
	}
	scale := 1 + math.Abs(c.Center.U) + math.Abs(c.Center.V) + r + math.Max(math.Abs(z0), math.Abs(z1))
	l, err := occupancy.CircleLeaf(chain, c.Center.U, c.Center.V, r, z0, z1, sectionDelta, dz0, dz1,
		relativeSlack*scale, Sides, mesh.Vertices, mesh.Bound)
	if err != nil {
		return nil
	}
	return occupancy.LeafRecord(l)
}
