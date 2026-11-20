package retainedblend

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// roundedRimTorus encloses the exact offset of a cardinal plane/cylinder rim.
// The ideal rim center uses the plane's axial coordinate and the cylinder
// axis origin's other coordinates. Side and kappa only select the offset sign.
func roundedRimTorus(r rim, radius float64, tag surfacegeom.Torus) *topo.FilletCarrier {
	if r.charge != 0 || r.planeFace == nil || r.wallFace == nil || math.Abs(r.kappa) != 1 || math.Abs(r.side) != 1 {
		return nil
	}
	exactSource := func(f *topo.Face) bool {
		return f.RetainedTag && f.Denoted == nil && f.TagDeparture == 0 && f.AxialDelta == 0 && f.ExtentDelta == 0
	}
	if !exactSource(r.planeFace) || !exactSource(r.wallFace) {
		return nil
	}
	p, ok := r.planeFace.Surface.(surfacegeom.Plane)
	c, cyl := r.wallFace.Surface.(surfacegeom.Cylinder)
	if !ok || !cyl || r.wallFace.FilletCarrier == nil || r.wallFace.FilletCarrier.Surface != c || !(r.wallFace.FilletCarrier.Distance >= 0) || (r.planeFace.NormalBound != 0 && (r.planeFace.PlacedPlane == nil || *r.planeFace.PlacedPlane != p)) {
		return nil
	}
	coords := func(v r3.Vec) [3]float64 { return [3]float64{v.X, v.Y, v.Z} }
	axis := -1
	n := coords(tag.Axis)
	for j, x := range n {
		if math.Abs(x) == 1 && axis < 0 {
			axis = j
		} else if x != 0 {
			return nil
		}
	}
	if axis < 0 || !(c.Axis == tag.Axis || c.Axis == tag.Axis.Scale(-1)) || coords(p.Frame.U())[axis] != 0 || coords(p.Frame.V())[axis] != 0 || p.Frame.U().Cross(p.Frame.V()).Len() == 0 {
		return nil
	}
	if !(p.Frame.N() == tag.Axis || p.Frame.N() == tag.Axis.Scale(-1)) {
		return nil
	}
	held := coords(tag.Center)
	base := coords(c.Origin)
	capRecord := r.planeFace.CapRecord
	if capRecord == nil || capRecord.Xform != r3.Identity() || coords(capRecord.Frame.U())[axis] != 0 || coords(capRecord.Frame.V())[axis] != 0 || math.Abs(coords(capRecord.Frame.N())[axis]) != 1 {
		return nil
	}
	if capRecord.ZDenoted != nil && (capRecord.ZDenoted.Num().BitLen() > 4096 || capRecord.ZDenoted.Denom().BitLen() > 4096) {
		return nil
	}
	if !proofbound.FiniteVec(capRecord.Frame.Origin()) || proofbound.IsNonFinite(capRecord.Z) {
		return nil
	}
	level := new(big.Rat).SetFloat64(capRecord.Z)
	if capRecord.ZDenoted != nil {
		level.Set(capRecord.ZDenoted)
	}
	level.Mul(level, new(big.Rat).SetFloat64(coords(capRecord.Frame.N())[axis]))
	level.Add(level, new(big.Rat).SetFloat64(coords(capRecord.Frame.Origin())[axis]))
	base[axis] = coords(p.Frame.Origin())[axis]
	if !proofbound.FiniteVec(p.Frame.Origin()) {
		return nil
	}
	capDeviation := proofbound.RatFloatUp(level.Abs(level.Sub(level, new(big.Rat).SetFloat64(base[axis]))))
	var displacement [3]float64
	for j := range base {
		displacement[j] = proofbound.ExactSumRound(held[j], base[j], n[j]*r.side*radius)
	}
	major, e := tag.Major.In(units.Millimeter)
	if e != nil {
		return nil
	}
	minor, e := tag.Minor.In(units.Millimeter)
	if e != nil || minor != radius {
		return nil
	}
	wall, e := c.Radius.In(units.Millimeter)
	if e != nil || wall != r.radius {
		return nil
	}
	delta := proofbound.AbsSumUpper(r.wallFace.FilletCarrier.Distance, capDeviation, displacement[0], displacement[1], displacement[2], proofbound.ExactSumRound(major, wall, r.kappa*radius))
	if !(delta >= 0) || math.IsInf(delta, 0) {
		return nil
	}
	return &topo.FilletCarrier{Surface: tag, Distance: delta}
}
