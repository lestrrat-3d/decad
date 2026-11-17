package brepmodify

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// cornerRelief is the region near one concave end's corner where a relieved cutter's result may differ from the
// chamfer without the relief: the cylinder of that radius about the segment from top to bottom, the corner's vertical
// edge from the cap down to the depth the cutter's bevel can reach.
type cornerRelief struct {
	top, bottom r3.Vec
	radius      float64
	height      float64
}

// reliefPad widens a relief region and its height, in millimetres, for the rounding of the frame that places them.
const reliefPad = 1e-9

// reliefRegion is the region about the corner of the concave end e, whose relief has radius relief, for a cutter of
// setback d and overshoot over. Inside the material the cutter removes nothing deeper than its bevel, which leaves the
// wall d below the cap, and the bevel's facets lie within the cutter's bound of it, so d + over covers the depth.
func (l *chamferLoop) reliefRegion(e runEnd, d, over float64) cornerRelief {
	h := d + over
	return cornerRelief{top: l.to3(e.v, 0), bottom: l.to3(e.v, -h), radius: l.relief + reliefPad,
		height: proofbound.AbsSumUpper(h, reliefPad)}
}

// chargeCornerReliefs restates body, the Cut of cutters whose end clips took the corner reliefs, against the chamfer
// whose clips take none, and builds the restated body under env. The two results differ only inside each relief's
// region: the relieved one keeps material there that the other removes. That material lies within the relief's radius
// of the corner's vertical edge, and that edge lies on the boundary of both results, where the corner's empty wedge
// meets the material. So every point of either boundary inside a region lies within the radius of the other.
//
// The charge follows from that:
//   - Each held facet whose box, widened by its own bound δ(t), meets a region's box adds the radius to the bound of
//     each of its corners. A held point within δ(t) of a point of the relieved boundary inside the region is then
//     within δ(t) plus the radius of the other boundary, and a point of the other boundary inside the region is within
//     the radius of the corner's edge, which such a facet holds within δ(t). MeshBound becomes the largest facet bound
//     again.
//   - VolSymDiff adds each region's volume, pi radius^2 height, since the kept material lies inside it.
//   - AreaSlack adds eight times each region's surface area, 2 pi radius^2 + 2 pi radius height. The two boundaries
//     differ only on the boundary of the kept material, which lies in the region and is made of pieces of the region's
//     own boundary, of the cap, of the next wall and the run's wall, of the far cap where the prism is thinner than
//     the region, and of the cutter's bevel. Each piece is a plane, a cylinder about an axis parallel to the region's,
//     or the bevel, a surface that rises at 45 degrees over the cap, so none holds more area inside the region than
//     the region's own surface. Seven pieces at most leave a margin of one.
//
// The restated payload keeps the held mesh, its source groups and their tags, and the composed proof terms, and drops
// every record that certifies more of the unrelieved Boolean than its bounds: a group's centroid and the Boolean's
// similarity, occupancy, refinement and exact-source records. AssembleBody then reads every vertex, edge, face, area,
// volume, centroid and box bound from the charged payload. The body's operation warnings carry over.
func chargeCornerReliefs(ctx context.Context, env topo.Env, body *topo.Body, regions []cornerRelief) (*topo.Body,
	error) {
	fp, ok := body.Payload.(facetedrecord.Payload)
	if !ok {
		return nil, fmt.Errorf(`%w: the partial chamfer's relieved Cut holds no faceted mesh to charge`,
			decaderr.ErrUnsupported)
	}
	if len(fp.VertexBound) != len(fp.Verts) {
		return nil, fmt.Errorf(`%w: the partial chamfer's relieved Cut carries %d vertex bounds for %d vertices`,
			decaderr.ErrBooleanFailed, len(fp.VertexBound), len(fp.Verts))
	}
	bounds := append([]float64(nil), fp.VertexBound...)
	extra := make([]float64, len(fp.Verts))
	volume, area := fp.VolSymDiff, fp.AreaSlack
	for _, rg := range regions {
		lo, hi := segmentBox(rg.top, rg.bottom, rg.radius)
		for _, t := range fp.Tris {
			delta := max(fp.VertexBound[t[0]], fp.VertexBound[t[1]], fp.VertexBound[t[2]])
			flo, fhi := reliefFacetBox(fp.Verts, t, proofbound.AbsSumUpper(delta, reliefPad))
			if !boxesMeet(flo, fhi, lo, hi) {
				continue
			}
			for _, i := range t {
				extra[i] = max(extra[i], rg.radius)
			}
		}
		r2 := proofbound.ProductUpper(rg.radius, rg.radius)
		volume = proofbound.AbsSumUpper(volume, proofbound.ProductUpper(proofbound.ProductUpper(reliefPi, r2), rg.height))
		surface := proofbound.AbsSumUpper(proofbound.ProductUpper(2*reliefPi, r2),
			proofbound.ProductUpper(proofbound.ProductUpper(2*reliefPi, rg.radius), rg.height))
		area = proofbound.AbsSumUpper(area, proofbound.ProductUpper(8, surface))
	}
	meshBound := fp.MeshBound
	for i, e := range extra {
		if e > 0 {
			bounds[i] = proofbound.AbsSumUpper(bounds[i], e)
		}
	}
	for _, t := range fp.Tris {
		meshBound = max(meshBound, bounds[t[0]], bounds[t[1]], bounds[t[2]])
	}
	if proofbound.IsNonFinite(meshBound) || proofbound.IsNonFinite(volume) || proofbound.IsNonFinite(area) {
		return nil, fmt.Errorf(`%w: the partial chamfer's corner reliefs leave no finite bound`, decaderr.ErrUnsupported)
	}
	groups := append([]facetedrecord.Group(nil), fp.Groups...)
	for i := range groups {
		groups[i].HasCentroid = false
		groups[i].Centroid = measurement.VecMeasurement{}
	}
	charged := facetedrecord.Payload{
		Verts: fp.Verts, VertexBound: bounds, Tris: fp.Tris, Src: fp.Src, Groups: groups,
		AxialSectionJoin: fp.AxialSectionJoin,
		MeshBound:        meshBound, VolSymDiff: volume, AreaSlack: area, DPair: fp.DPair,
		Xform: fp.Xform,
	}
	out, err := facetedrecord.Build(ctx, env, charged)
	if err != nil {
		return nil, err
	}
	out.OperationWarnings = body.OperationWarnings
	return out, nil
}

// reliefPi is an upper bound on pi.
const reliefPi = 3.1415927

// segmentBox is the box of the segment from a to b widened by r on every side.
func segmentBox(a, b r3.Vec, r float64) (r3.Vec, r3.Vec) {
	lo := r3.Vec{X: math.Min(a.X, b.X) - r, Y: math.Min(a.Y, b.Y) - r, Z: math.Min(a.Z, b.Z) - r}
	hi := r3.Vec{X: math.Max(a.X, b.X) + r, Y: math.Max(a.Y, b.Y) + r, Z: math.Max(a.Z, b.Z) + r}
	return lo, hi
}

// reliefFacetBox is the box of facet t of verts widened by r on every side.
func reliefFacetBox(verts []r3.Vec, t [3]int, r float64) (r3.Vec, r3.Vec) {
	a, b, c := verts[t[0]], verts[t[1]], verts[t[2]]
	lo := r3.Vec{X: min(a.X, b.X, c.X) - r, Y: min(a.Y, b.Y, c.Y) - r, Z: min(a.Z, b.Z, c.Z) - r}
	hi := r3.Vec{X: max(a.X, b.X, c.X) + r, Y: max(a.Y, b.Y, c.Y) + r, Z: max(a.Z, b.Z, c.Z) + r}
	return lo, hi
}

// boxesMeet reports whether the boxes [alo, ahi] and [blo, bhi] share a point.
func boxesMeet(alo, ahi, blo, bhi r3.Vec) bool {
	return alo.X <= bhi.X && blo.X <= ahi.X && alo.Y <= bhi.Y && blo.Y <= ahi.Y && alo.Z <= bhi.Z && blo.Z <= ahi.Z
}
