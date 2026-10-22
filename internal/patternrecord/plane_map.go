package patternrecord

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// PlaneMap is the exact map from the section plane of one placed frame onto the
// section plane of another: (u, v) ↦ A·(u, v) + T in the target's frame
// coordinates, with every entry an exact rational of the held floats.
//
// A frame is orthonormal only to rounding, so the map reads the target's
// coordinates through the inverse of its Gram matrix, which is exact for a
// point in the target's plane, and keeps two defects it does not hide:
//
//   - the image of a source point can sit a little off the target plane: its
//     distance is |S0 + S1·u + S2·v| / |n|, with n the target's plane normal
//     (NormalCharge);
//   - A is not exactly a similarity. Defect bounds its distance from the nearest
//     one, S* = [[a, −b], [b, a]], whose scale √(a² + b²) lies in
//     [ScaleLo, ScaleHi]. A line and a fit spline map exactly under A. A circle
//     or arc maps to an ellipse, which the charge of MovePlaneRegion bounds
//     against the circle S* gives.
//
// Both defects are zero for a pure rotation about an axis of an orthonormal
// frame whose float matrix is exact, and a few float steps otherwise.
type PlaneMap struct {
	A [2][2]*big.Rat
	T [2]*big.Rat
	// S0, S1 and S2 are the numerators of the source point's signed distance from
	// the target plane, and NormalLo a float at or below |n|.
	S0, S1, S2 *big.Rat
	NormalLo   float64
	// ScaleLo and ScaleHi enclose √(a² + b²), the scale of the nearest similarity.
	ScaleLo, ScaleHi float64
	// Defect bounds ‖A − S*‖ in the operator norm.
	Defect float64
}

// NewPlaneMap reads the map from the section frame `from` placed by fromX onto
// the section frame `to` placed by toX. ok is false only when the target frame
// has no plane (its axes are parallel or zero) or the map has no inverse
// scale; the defects above are charged by the caller, not refused here.
func NewPlaneMap(from r3.Frame, fromX r3.Transform, to r3.Frame, toX r3.Transform) (PlaneMap, bool) {
	u, v, o := placedAxes(from, fromX)
	ru, rv, ro := placedAxes(to, toX)
	n := ratVecCross(ru, rv)
	nn := ratVecDot(n, n)
	if nn.Sign() <= 0 {
		return PlaneMap{}, false
	}
	// Gram inverse, scaled by det = |ru × rv|² (Lagrange's identity).
	g00, g01, g11 := ratVecDot(ru, ru), ratVecDot(ru, rv), ratVecDot(rv, rv)
	inv := func(x *big.Rat) *big.Rat { return new(big.Rat).Quo(x, nn) }
	gi := [2][2]*big.Rat{{inv(g11), inv(new(big.Rat).Neg(g01))}, {inv(new(big.Rat).Neg(g01)), inv(g00)}}
	d := ratVec{new(big.Rat).Sub(o[0], ro[0]), new(big.Rat).Sub(o[1], ro[1]), new(big.Rat).Sub(o[2], ro[2])}
	b := [2][2]*big.Rat{{ratVecDot(u, ru), ratVecDot(v, ru)}, {ratVecDot(u, rv), ratVecDot(v, rv)}}
	t := [2]*big.Rat{ratVecDot(d, ru), ratVecDot(d, rv)}
	mul := proofbound.RatMul
	add := proofbound.RatAdd
	pm := PlaneMap{
		A: [2][2]*big.Rat{
			{add(mul(gi[0][0], b[0][0]), mul(gi[0][1], b[1][0])), add(mul(gi[0][0], b[0][1]), mul(gi[0][1], b[1][1]))},
			{add(mul(gi[1][0], b[0][0]), mul(gi[1][1], b[1][0])), add(mul(gi[1][0], b[0][1]), mul(gi[1][1], b[1][1]))},
		},
		T:        [2]*big.Rat{add(mul(gi[0][0], t[0]), mul(gi[0][1], t[1])), add(mul(gi[1][0], t[0]), mul(gi[1][1], t[1]))},
		S0:       ratVecDot(d, n),
		S1:       ratVecDot(u, n),
		S2:       ratVecDot(v, n),
		NormalLo: proofbound.RatSqrtDown(nn),
	}
	if pm.NormalLo <= 0 {
		return PlaneMap{}, false
	}
	// A reflection reverses every recorded loop and circle sense, which this map
	// does not re-record: it is refused.
	if det := new(big.Rat).Sub(mul(pm.A[0][0], pm.A[1][1]), mul(pm.A[0][1], pm.A[1][0])); det.Sign() <= 0 {
		return PlaneMap{}, false
	}
	half := big.NewRat(1, 2)
	a := mul(add(pm.A[0][0], pm.A[1][1]), half)
	bb := mul(new(big.Rat).Sub(pm.A[1][0], pm.A[0][1]), half)
	scale := add(mul(a, a), mul(bb, bb))
	pm.ScaleLo, pm.ScaleHi = proofbound.RatSqrtDown(scale), proofbound.RatSqrtUp(scale)
	ex := mul(new(big.Rat).Sub(pm.A[0][0], pm.A[1][1]), half)
	ey := mul(add(pm.A[0][1], pm.A[1][0]), half)
	pm.Defect = proofbound.RatFloatUp(add(new(big.Rat).Abs(ex), new(big.Rat).Abs(ey)))
	return pm, pm.ScaleLo > 0
}

// ScaleDeviation is an upper bound on |scale − 1| of the nearest similarity.
func (pm PlaneMap) ScaleDeviation() float64 {
	return max(pm.ScaleHi-1, 1-pm.ScaleLo, 0)
}

// NormalCharge bounds how far the image of a point of the source plane sits off
// the target plane, for every point with |u| ≤ extentU and |v| ≤ extentV.
func (pm PlaneMap) NormalCharge(extentU, extentV float64) float64 {
	sum := new(big.Rat).Abs(pm.S0)
	sum.Add(sum, proofbound.RatMul(new(big.Rat).Abs(pm.S1), proofarith.FloatRat(extentU)))
	sum.Add(sum, proofbound.RatMul(new(big.Rat).Abs(pm.S2), proofarith.FloatRat(extentV)))
	if sum.Sign() == 0 {
		return 0
	}
	return math.Nextafter(proofbound.RatFloatUp(sum)/pm.NormalLo, math.Inf(1))
}

// Exact returns the exact image of a held point.
func (pm PlaneMap) Exact(p Point2) (u, v *big.Rat) {
	pu, pv := proofarith.FloatRat(p.U), proofarith.FloatRat(p.V)
	u = proofbound.RatAdd(proofbound.RatMul(pm.A[0][0], pu), proofbound.RatMul(pm.A[0][1], pv), pm.T[0])
	v = proofbound.RatAdd(proofbound.RatMul(pm.A[1][0], pu), proofbound.RatMul(pm.A[1][1], pv), pm.T[1])
	return u, v
}

// Motion is the map as a PointMotion: the exact image rounded once.
func (pm PlaneMap) Motion() PointMotion {
	return func(p Point2) (Point2, float64, error) {
		u, v := pm.Exact(p)
		return heldPoint(proofbound.PointInterval(u), proofbound.PointInterval(v))
	}
}

func ratVecCross(a, b ratVec) ratVec {
	cross := func(i, j int) *big.Rat {
		return new(big.Rat).Sub(proofbound.RatMul(a[i], b[j]), proofbound.RatMul(a[j], b[i]))
	}
	return ratVec{cross(1, 2), cross(2, 0), cross(0, 1)}
}

// PlaneHeight is the signed distance, along the target frame's plane normal, of
// the plane that lies level·N from the frame's origin (N the frame's own normal,
// level an exact offset along it), once the frame is placed by x and the target
// by toX, as the exact numerator over |n| and a float at or below |n|. Two
// sweeps whose planes coincide have heights that agree up to the placement's
// rounding, and Gap measures it.
func PlaneHeight(frame r3.Frame, x r3.Transform, level *big.Rat, to r3.Frame, toX r3.Transform) (height *big.Rat, normalLo float64, ok bool) {
	_, _, o := placedAxes(frame, x)
	ru, rv, ro := placedAxes(to, toX)
	n := ratVecCross(ru, rv)
	normalLo = proofbound.RatSqrtDown(ratVecDot(n, n))
	if normalLo <= 0 {
		return nil, 0, false
	}
	basis := x.Basis()
	fn := ratVecOf(frame.N())
	ex, ey, ez := ratVecOf(basis.EX), ratVecOf(basis.EY), ratVecOf(basis.EZ)
	var lifted ratVec
	for k := range lifted {
		step := proofbound.RatAdd(proofbound.RatMul(ex[k], fn[0]), proofbound.RatMul(ey[k], fn[1]), proofbound.RatMul(ez[k], fn[2]))
		lifted[k] = new(big.Rat).Sub(new(big.Rat).Add(o[k], proofbound.RatMul(level, step)), ro[k])
	}
	return ratVecDot(lifted, n), normalLo, true
}

// HeightGap is the distance, along the target normal, between two heights that
// PlaneHeight measured against one target, rounded up.
func HeightGap(a, b *big.Rat, normalLo float64) float64 {
	gap := new(big.Rat).Abs(new(big.Rat).Sub(a, b))
	if gap.Sign() == 0 {
		return 0
	}
	return math.Nextafter(proofbound.RatFloatUp(gap)/normalLo, math.Inf(1))
}

// SweepShift reads how far the end frame f1 sits from the start frame f0 along
// the sweep axis, once both are placed by x: the exact multiple of the placed
// normal that carries f0's origin nearest f1's, and Lateral, an upper bound on
// how far f1's origin sits off the axis through f0's origin (zero for an exact
// extrusion; a few float steps when the end frame was made by adding a float
// multiple of the normal to an origin that is not at zero). ok is false for a
// zero normal or a lateral offset past OffPlaneLimit times the origins'
// magnitude plus one: that is a sheared sweep, not an extrusion.
func SweepShift(f0, f1 r3.Frame, x r3.Transform) (shift *big.Rat, lateral float64, ok bool) {
	_, _, o0 := placedAxes(f0, x)
	_, _, o1 := placedAxes(f1, x)
	basis := x.Basis()
	n := ratVecOf(f0.N())
	ex, ey, ez := ratVecOf(basis.EX), ratVecOf(basis.EY), ratVecOf(basis.EZ)
	var nd, d ratVec
	for k := range nd {
		nd[k] = proofbound.RatAdd(proofbound.RatMul(ex[k], n[0]), proofbound.RatMul(ey[k], n[1]), proofbound.RatMul(ez[k], n[2]))
		d[k] = new(big.Rat).Sub(o1[k], o0[k])
	}
	nn := ratVecDot(nd, nd)
	if nn.Sign() == 0 {
		return nil, 0, false
	}
	shift = new(big.Rat).Quo(ratVecDot(d, nd), nn)
	var lat ratVec
	for k := range lat {
		lat[k] = new(big.Rat).Sub(d[k], proofbound.RatMul(shift, nd[k]))
	}
	lateral = 0
	if ll := ratVecDot(lat, lat); ll.Sign() != 0 {
		lateral = proofbound.RatSqrtUp(ll)
	}
	scale := 1.0
	for k := range o0 {
		a, _ := new(big.Rat).Abs(o0[k]).Float64()
		b, _ := new(big.Rat).Abs(d[k]).Float64()
		scale = math.Max(scale, 1+a+b)
	}
	return shift, lateral, lateral <= OffPlaneLimit*scale
}
