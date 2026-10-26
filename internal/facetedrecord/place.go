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
	"github.com/lestrrat-3d/decad/internal/topo"
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
// A retained Plane group with no denoted patch, a zero NormalBound and a tag
// normal that q fixes exactly (PlaneFixedBy) keeps its tag, its CapRecord and
// its axial bound verbatim and with bound 0: q maps that plane onto itself, so
// the source's exact record still denotes the plane of the copy. When the group
// has a CapRecord, the record's plane must be fixed by q as well. Any other
// retained Plane, Cylinder, Sphere or Torus group with no denoted patch keeps
// its tag. The tag becomes its float image under delta, the float placement
// that moved the held mesh, and the group's NormalBound becomes its own plus
// PlacedTagDepartureBy against q with defect 0 (retention design §3.4), where
// verts and reach are the held mesh and its reach before the placement. A
// retained Cone keeps the trivial bound 2, as under a float placement. Every
// other tag, a denoted patch and every other CapRecord describe the float
// placement rather than q and are dropped, so those faces read as Faceted.
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
	record, axial, hasAxial := g.CapRecord, g.AxialDelta, g.HasAxialDelta
	g.Surface, g.Denoted, g.NormalBound = nil, nil, 0
	g.CapRecord, g.AxialDelta, g.HasAxialDelta = nil, 0, false
	if plane, ok := source.(surfacegeom.Plane); ok && g.Retained && !denoted && sourceBound == 0 &&
		planeGroupFixedBy(plane, record, q.Linear(), q.Translation()) {
		g.Surface, g.CapRecord, g.AxialDelta, g.HasAxialDelta = source, record, axial, hasAxial
		return nil
	}
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

// planeGroupFixedBy reports whether the exact rotation (linear, t) maps both
// the tag plane and, when there is one, the plane the CapRecord denotes onto
// themselves.
func planeGroupFixedBy(tag surfacegeom.Plane, record *topo.CapRecord, linear [3][3]*big.Rat, t [3]*big.Rat) bool {
	n, ok := ratVec3(tag.Frame.N())
	if !ok || !PlaneFixedBy(linear, t, n) {
		return false
	}
	if record == nil {
		return true
	}
	m, ok := capRecordNormal(record)
	return ok && PlaneFixedBy(linear, t, m)
}

// capRecordNormal is the exact normal m = (B*U) x (B*V) of the plane a
// CapRecord denotes, where B is the linear part of its Xform.
func capRecordNormal(rec *topo.CapRecord) ([3]*big.Rat, bool) {
	u, uok := ratVec3(rec.Frame.U())
	v, vok := ratVec3(rec.Frame.V())
	basis := rec.Xform.Basis()
	ex, xok := ratVec3(basis.EX)
	ey, yok := ratVec3(basis.EY)
	ez, zok := ratVec3(basis.EZ)
	if !uok || !vok || !xok || !yok || !zok {
		return [3]*big.Rat{}, false
	}
	apply := func(a [3]*big.Rat) [3]*big.Rat {
		var out [3]*big.Rat
		for i := range out {
			out[i] = pointPlaneAdd(pointPlaneAdd(pointPlaneMul(ex[i], a[0]), pointPlaneMul(ey[i], a[1])),
				pointPlaneMul(ez[i], a[2]))
		}
		return out
	}
	bu, bv := apply(u), apply(v)
	m := [3]*big.Rat{
		pointPlaneSub(pointPlaneMul(bu[1], bv[2]), pointPlaneMul(bu[2], bv[1])),
		pointPlaneSub(pointPlaneMul(bu[2], bv[0]), pointPlaneMul(bu[0], bv[2])),
		pointPlaneSub(pointPlaneMul(bu[0], bv[1]), pointPlaneMul(bu[1], bv[0])),
	}
	if m[0].Sign() == 0 && m[1].Sign() == 0 && m[2].Sign() == 0 {
		return m, false
	}
	return m, true
}

// PlaneFixedBy reports whether the exact map p -> linear*p + t sends every
// plane with normal direction m onto itself. It holds exactly when linear*m = m,
// linear^T*m = m and m.t = 0, all in rationals; a rotation about the direction
// m satisfies them whatever its center. A nonzero m is required. The check
// compares exact data and admits only when every equality holds exactly.
func PlaneFixedBy(linear [3][3]*big.Rat, t, m [3]*big.Rat) bool {
	if m[0].Sign() == 0 && m[1].Sign() == 0 && m[2].Sign() == 0 {
		return false
	}
	dot := new(big.Rat)
	for i := range m {
		dot.Add(dot, pointPlaneMul(m[i], t[i]))
		var forward, back = new(big.Rat), new(big.Rat)
		for j := range m {
			forward.Add(forward, pointPlaneMul(linear[i][j], m[j]))
			back.Add(back, pointPlaneMul(linear[j][i], m[j]))
		}
		if forward.Cmp(m[i]) != 0 || back.Cmp(m[i]) != 0 {
			return false
		}
	}
	return dot.Sign() == 0
}

func ratVec3(v r3.Vec) ([3]*big.Rat, bool) {
	out := [3]*big.Rat{ratOfPointPlane(v.X), ratOfPointPlane(v.Y), ratOfPointPlane(v.Z)}
	return out, out[0] != nil && out[1] != nil && out[2] != nil
}
