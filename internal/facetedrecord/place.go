package facetedrecord

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfaceplace"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// PlacementDelta is the float map applied to the held mesh. Motion
// verification must charge this same map against its ideal pose.
func PlacementDelta(xform, composed r3.Transform) (r3.Transform, error) {
	inv, err := xform.Inverse()
	if err != nil {
		return r3.Transform{}, fmt.Errorf(`decad: inverting the accumulated placement failed: %w`, err)
	}
	delta, err := inv.Then(composed)
	if err != nil {
		return r3.Transform{}, fmt.Errorf(`decad: composing the placement failed: %w`, err)
	}
	return delta, nil
}

// SourceDiameterUpper bounds any pair of held or true boundary
// points. Every true point lies within departure of the held surface, and
// the held surface lies inside its vertices' coordinate box.
func SourceDiameterUpper(verts []r3.Vec, departure float64) float64 {
	lo, hi := [3]float64{math.Inf(1), math.Inf(1), math.Inf(1)},
		[3]float64{math.Inf(-1), math.Inf(-1), math.Inf(-1)}
	for _, vertex := range verts {
		coords := [3]float64{vertex.X, vertex.Y, vertex.Z}
		for i, v := range coords {
			lo[i] = min(lo[i], v)
			hi[i] = max(hi[i], v)
		}
	}
	return proofbound.AbsSumUpper(proofbound.DvLenUpper(proofbound.HeldDelta(
		r3.NewVec(hi[0], hi[1], hi[2]), r3.NewVec(lo[0], lo[1], lo[2]))),
		proofbound.ProductUpper(2, departure))
}

// ExactGroupPlaced restates one source group for a copy that denotes the exact
// rotation q. The cap centroid moves by q itself, which is exactly orthogonal,
// so the placed reading adds no stretch charge. Planar, Reversed and Origins
// describe the source face, which q maps rigidly, and stay.
//
// A retained Plane, Cylinder, Sphere or Torus group with no denoted patch keeps
// its tag. The tag becomes its float image under delta, the float placement
// that moved the held mesh, and the group's NormalBound becomes its own plus
// PlacedTagDepartureBy against q with defect 0 (retention design §3.4), where
// verts and reach are the held mesh and its reach before the placement. A
// retained Cone keeps the trivial bound 2, as under a float placement. Every
// other tag, a denoted patch and the CapRecord describe the float placement
// rather than q and are dropped, so those faces read as Faceted.
func ExactGroupPlaced(g *Group, q *exactrotation.Rotation, delta r3.Transform, verts []r3.Vec,
	reach float64) error {
	if g.HasCentroid {
		reading, err := PlacedCapCentroidBy(g.Centroid, q.Linear(), q.Translation())
		if err != nil {
			return err
		}
		g.Centroid = reading
	}
	source, sourceBound, denoted := g.Surface, g.NormalBound, g.Denoted != nil
	g.Surface, g.Denoted, g.NormalBound = nil, nil, 0
	g.CapRecord, g.AxialDelta, g.HasAxialDelta = nil, 0, false
	if !g.Retained || source == nil || denoted {
		g.Retained = false
		return nil
	}
	switch source.(type) {
	case surfacegeom.Plane, surfacegeom.Cylinder, surfacegeom.Cone, surfacegeom.Sphere, surfacegeom.Torus:
	default:
		g.Retained = false
		return nil
	}
	moved, err := surfaceplace.TransformSurface(source, delta)
	if err != nil {
		return err
	}
	g.Surface, g.NormalBound = moved, 2
	if add, stated := PlacedTagDepartureBy(source, moved, q.Linear(), q.Translation(), 0, verts, reach); stated {
		g.NormalBound = min(2, proofbound.AbsSumUpper(sourceBound, add))
	}
	return nil
}

// PlacedCapCentroid applies the recorded affine motion to the true cap
// centroid. Rational evaluation rounds the reported point once; the placement
// matrix's proved stretch maps the source radius to the new one.
func PlacedCapCentroid(source measurement.VecMeasurement, motion r3.Transform) (measurement.VecMeasurement, error) {
	linear, err := massmoment.PlacementRotation(motion)
	if err != nil {
		return measurement.VecMeasurement{}, err
	}
	tr := motion.Translation()
	t := [3]*big.Rat{ratOfPointPlane(tr.X), ratOfPointPlane(tr.Y), ratOfPointPlane(tr.Z)}
	return PlacedCapCentroidBy(source, linear, t)
}

// PlacedCapCentroidBy applies the exact affine map p -> linear*p + t to the
// true cap centroid. Rational evaluation rounds the reported point once; the
// map's proved stretch, zero for an exactly orthogonal linear part, carries the
// source radius to the new one.
func PlacedCapCentroidBy(source measurement.VecMeasurement, linear [3][3]*big.Rat, t [3]*big.Rat) (measurement.VecMeasurement, error) {
	charge, err := massmoment.MapChargeOf(linear)
	if err != nil {
		return measurement.VecMeasurement{}, err
	}
	p := [3]float64{source.Value.X, source.Value.Y, source.Value.Z}
	var exact [3]*big.Rat
	for i := range exact {
		exact[i] = new(big.Rat).Set(t[i])
		for j := range p {
			exact[i].Add(exact[i], pointPlaneMul(linear[i][j], ratOfPointPlane(p[j])))
		}
	}
	value, round, err := PointPlaneRound(exact)
	if err != nil {
		return measurement.VecMeasurement{}, err
	}
	bound := proofbound.AbsSumUpper(proofbound.ProductUpper(source.Bound.Base(),
		proofbound.AbsSumUpper(1, charge.Stretch)), round)
	return measurement.VecMeasurement{Value: value, Exactness: measurement.ExactnessOf(bound),
		Bound: units.Millimeters(bound)}, nil
}

// PointPlaneRound rounds an exact point to its nearest held coordinates and
// returns the 3D radius of the rounding gap. A coordinate that has no finite
// binary64 reading refuses.
func PointPlaneRound(q [3]*big.Rat) (r3.Vec, float64, error) {
	var value [3]float64
	gap := 0.0
	for i := range value {
		value[i], _ = q[i].Float64()
		if proofbound.IsNonFinite(value[i]) {
			return r3.Vec{}, 0, fmt.Errorf("%w: a planar split vertex is not representable", decaderr.ErrUnsupported)
		}
		gap = max(gap, proofbound.RatAbsDiff(q[i], value[i]))
	}
	return r3.NewVec(value[0], value[1], value[2]), proofbound.Radius3D(gap), nil
}
