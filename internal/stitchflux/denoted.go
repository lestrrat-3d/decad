package stitchflux

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// ExactRevolve is a revolved record whose axis, basis and meridian leaves
// read as exact rational points.
type ExactRevolve struct {
	O, W     [3]*big.Rat
	e0, e1   [3]*big.Rat
	Ends     [][2][2]*big.Rat
	Circular bool
	Centre   [2]*big.Rat
	Radius   *big.Rat
}

// ExactRevolveOf refuses a non-point leaf or a basis that is not exactly
// orthonormal. The returned values are the record's own exact rationals.
func ExactRevolveOf(r surfacenormal.Revolved) (ExactRevolve, bool) {
	var ex ExactRevolve
	vecs := [...]*[3]*big.Rat{&ex.O, &ex.e0, &ex.e1, &ex.W}
	for i, iv := range [...]proofbound.IvVec3{r.Origin, r.Basis[0], r.Basis[1], r.Basis[2]} {
		for k := range 3 {
			x, ok := RatPoint(iv[k])
			if !ok {
				return ExactRevolve{}, false
			}
			vecs[i][k] = x
		}
	}
	basis := [3][3]*big.Rat{ex.e0, ex.e1, ex.W}
	for i := range basis {
		for j := range basis {
			want := int64(0)
			if i == j {
				want = 1
			}
			if ratDot(basis[i], basis[j]).Cmp(big.NewRat(want, 1)) != 0 {
				return ExactRevolve{}, false
			}
		}
	}
	ex.Circular = r.Circular
	if r.Circular {
		zc, okZ := RatPoint(r.Centre[0])
		rc, okR := RatPoint(r.Centre[1])
		if !okZ || !okR {
			return ExactRevolve{}, false
		}
		ex.Centre = [2]*big.Rat{zc, rc}
		// The Sphere arm derives its radius from area; the Torus arm reads
		// this leaf only when it is a point.
		if radius, ok := RatPoint(r.Radius); ok {
			ex.Radius = radius
		}
		return ex, true
	}
	for _, seg := range r.Ends {
		var out [2][2]*big.Rat
		for i := range seg {
			for k := range 2 {
				x, ok := RatPoint(seg[i][k])
				if !ok {
					return ExactRevolve{}, false
				}
				out[i][k] = x
			}
		}
		ex.Ends = append(ex.Ends, out)
	}
	return ex, true
}

func (ex ExactRevolve) axisPoint(z *big.Rat) [3]*big.Rat {
	var out [3]*big.Rat
	for k := range 3 {
		out[k] = new(big.Rat).Add(ex.O[k], new(big.Rat).Mul(ex.W[k], z))
	}
	return out
}

// AtAxis reports whether p is exactly the axis point at z.
func (ex ExactRevolve) AtAxis(p r3.Vec, z *big.Rat) bool {
	q, ok := RatVecOf(p)
	return ok && ratVecEqual(q, ex.axisPoint(z))
}

// OnAxis reports whether p lies exactly on the axis line.
func (ex ExactRevolve) OnAxis(p r3.Vec) bool {
	q, ok := RatVecOf(p)
	if !ok {
		return false
	}
	d := ratSub(q, ex.O)
	c := [3]*big.Rat{
		new(big.Rat).Sub(new(big.Rat).Mul(d[1], ex.W[2]), new(big.Rat).Mul(d[2], ex.W[1])),
		new(big.Rat).Sub(new(big.Rat).Mul(d[2], ex.W[0]), new(big.Rat).Mul(d[0], ex.W[2])),
		new(big.Rat).Sub(new(big.Rat).Mul(d[0], ex.W[1]), new(big.Rat).Mul(d[1], ex.W[0])),
	}
	return c[0].Sign() == 0 && c[1].Sign() == 0 && c[2].Sign() == 0
}

// AlongAxis reports whether a is exactly W or its negation.
func (ex ExactRevolve) AlongAxis(a r3.Vec) bool {
	q, ok := RatVecOf(a)
	if !ok {
		return false
	}
	neg := [3]*big.Rat{new(big.Rat).Neg(ex.W[0]), new(big.Rat).Neg(ex.W[1]), new(big.Rat).Neg(ex.W[2])}
	return ratVecEqual(q, ex.W) || ratVecEqual(q, neg)
}

// RimAtEnd checks a circular rim against the recorded meridian ends.
func (ex ExactRevolve) RimAtEnd(radius units.Value, center r3.Vec) bool {
	r, okR := RatMillimetres(radius)
	c, okC := RatVecOf(center)
	if !okR || !okC {
		return false
	}
	for _, seg := range ex.Ends {
		for _, end := range seg {
			if end[1].Cmp(r) == 0 && ratVecEqual(c, ex.axisPoint(end[0])) {
				return true
			}
		}
	}
	return false
}

// TaggedFace carries only the surface tag, its recorded denotation, and the
// edge curves used by the exact tag gate. Non-circular curves are ignored by
// the rim check, as they were in the face adapter.
type TaggedFace struct {
	Denoted *surfacenormal.Revolved
	Surface surfacegeom.Surface
	Curves  []surfacegeom.Curve
}

// TagIsDenoted decides whether a revolve face's published tag is exactly the
// surface its record denotes. It requires point leaves and an exactly
// orthonormal basis. A cylinder's ends must have its tagged radius; a plane's
// ends must share one level; and a cone's lines must meet at one apex. Sphere
// and torus centres must lie at their recorded positions. Every straight
// wall's circular rims must match recorded meridian ends. The only charged
// departure is a cone's float apex, handled by ConeApexDeparture.
func TagIsDenoted(face TaggedFace) bool {
	r := face.Denoted
	if r == nil || !r.Valid || r.Cap {
		return false
	}
	ex, ok := ExactRevolveOf(*r)
	if !ok {
		return false
	}
	switch s := face.Surface.(type) {
	case surfacegeom.Cylinder:
		radius, ok := RatMillimetres(s.Radius)
		if !ok || ex.Circular || len(ex.Ends) == 0 {
			return false
		}
		for _, seg := range ex.Ends {
			for _, end := range seg {
				if end[1].Cmp(radius) != 0 {
					return false
				}
			}
		}
		return ex.OnAxis(s.Origin) && ex.AlongAxis(s.Axis) && rimsAtEnds(ex, face.Curves)
	case surfacegeom.Plane:
		if ex.Circular || len(ex.Ends) == 0 {
			return false
		}
		z := ex.Ends[0][0][0]
		for _, seg := range ex.Ends {
			for _, end := range seg {
				if end[0].Cmp(z) != 0 {
					return false
				}
			}
		}
		u, okU := RatVecOf(s.Frame.U())
		v, okV := RatVecOf(s.Frame.V())
		origin, okO := RatVecOf(s.Frame.Origin())
		if !okU || !okV || !okO || ratDot(u, ex.W).Sign() != 0 || ratDot(v, ex.W).Sign() != 0 {
			return false
		}
		return ratDot(ratSub(origin, ex.O), ex.W).Cmp(z) == 0 && rimsAtEnds(ex, face.Curves)
	case surfacegeom.Cone:
		radius, ok := RatMillimetres(s.Radius)
		if !ok || radius.Sign() != 0 || ex.Circular || len(ex.Ends) == 0 {
			return false
		}
		var apex *big.Rat
		for _, seg := range ex.Ends {
			z0, rho0, z1, rho1 := seg[0][0], seg[0][1], seg[1][0], seg[1][1]
			drho := new(big.Rat).Sub(rho1, rho0)
			if drho.Sign() == 0 {
				return false
			}
			dz := new(big.Rat).Sub(z1, z0)
			at := new(big.Rat).Sub(z0, new(big.Rat).Quo(new(big.Rat).Mul(rho0, dz), drho))
			if apex != nil && apex.Cmp(at) != 0 {
				return false
			}
			apex = at
		}
		return ex.AlongAxis(s.Axis) && rimsAtEnds(ex, face.Curves)
	case surfacegeom.Sphere:
		return ex.Circular && ex.Centre[1].Sign() == 0 && ex.AtAxis(s.Center, ex.Centre[0])
	case surfacegeom.Torus:
		major, okMajor := RatMillimetres(s.Major)
		minor, okMinor := RatMillimetres(s.Minor)
		return okMajor && okMinor && ex.Circular && ex.Radius != nil &&
			ex.Centre[1].Cmp(major) == 0 && ex.Radius.Cmp(minor) == 0 &&
			ex.AtAxis(s.Center, ex.Centre[0]) && ex.AlongAxis(s.Axis)
	default:
		return false
	}
}

func rimsAtEnds(ex ExactRevolve, curves []surfacegeom.Curve) bool {
	for _, curve := range curves {
		circle, ok := curve.(surfacegeom.Circle3)
		if ok && !ex.RimAtEnd(circle.Radius, circle.Center) {
			return false
		}
	}
	return true
}

// ConeApexDeparture bounds the float cone apex's departure from the exact
// meridian intersection. A face without a separate denotation owes zero.
func ConeApexDeparture(r *surfacenormal.Revolved, cone surfacegeom.Cone) (float64, bool) {
	if r == nil {
		return 0, true
	}
	if !r.Valid || r.Circular || len(r.Ends) == 0 {
		return 0, false
	}
	var end [2][2]*big.Rat
	for i := range end {
		for k := range end[i] {
			x, ok := RatPoint(r.Ends[0][i][k])
			if !ok {
				return 0, false
			}
			end[i][k] = x
		}
	}
	drho := new(big.Rat).Sub(end[1][1], end[0][1])
	if drho.Sign() == 0 {
		return 0, false
	}
	dz := new(big.Rat).Sub(end[1][0], end[0][0])
	za := new(big.Rat).Sub(end[0][0], new(big.Rat).Quo(new(big.Rat).Mul(end[0][1], dz), drho))
	worst := 0.0
	for k, held := range [...]float64{cone.Origin.X, cone.Origin.Y, cone.Origin.Z} {
		o, okO := RatPoint(r.Origin[k])
		w, okW := RatPoint(r.Basis[2][k])
		if !okO || !okW {
			return 0, false
		}
		worst = math.Max(worst, proofarith.RationalFloatError(new(big.Rat).Add(o, new(big.Rat).Mul(w, za)), held))
	}
	return worst, !proofbound.IsNonFinite(worst)
}

// RatPoint returns an interval's exact value when both endpoints agree.
func RatPoint(iv proofbound.RatInterval) (*big.Rat, bool) {
	if iv.Lo == nil || iv.Hi == nil || iv.Lo.Cmp(iv.Hi) != 0 {
		return nil, false
	}
	return iv.Lo, true
}

// RatMillimetres reads a quantity's held millimetres as an exact rational.
func RatMillimetres(v units.Value) (*big.Rat, bool) {
	mm, err := v.In(units.Millimeter)
	if err != nil {
		return nil, false
	}
	r := proofarith.FloatRat(mm)
	return r, r != nil
}

// RatVecOf reads a held vector as exact rational coordinates.
func RatVecOf(v r3.Vec) ([3]*big.Rat, bool) {
	out := [3]*big.Rat{proofarith.FloatRat(v.X), proofarith.FloatRat(v.Y), proofarith.FloatRat(v.Z)}
	return out, out[0] != nil && out[1] != nil && out[2] != nil
}

func ratVecEqual(a, b [3]*big.Rat) bool {
	return a[0].Cmp(b[0]) == 0 && a[1].Cmp(b[1]) == 0 && a[2].Cmp(b[2]) == 0
}

func ratDot(a, b [3]*big.Rat) *big.Rat {
	return new(big.Rat).Add(new(big.Rat).Mul(a[0], b[0]),
		new(big.Rat).Add(new(big.Rat).Mul(a[1], b[1]), new(big.Rat).Mul(a[2], b[2])))
}

func ratSub(a, b [3]*big.Rat) [3]*big.Rat {
	return [3]*big.Rat{new(big.Rat).Sub(a[0], b[0]), new(big.Rat).Sub(a[1], b[1]), new(big.Rat).Sub(a[2], b[2])}
}
