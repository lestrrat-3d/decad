package decad_test

import (
	"fmt"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is Face.NormalAt's own exactness contract (topology.go,
// normal_bound.go). The reading is a DIRECTION, so it is Exact only where the
// float triple handed back really is the exact unit normal of the surface the
// face is tagged with — and two ordinary constructions leave it not being one:
// a rotated placement, whose cap frame normal is the rounded cross product of
// two rounded axes and so is neither of unit length nor exactly perpendicular
// to the plane those axes span, and any cone at all, whose normal is assembled
// from a float cosine and sine that do not satisfy cos² + sin² = 1. The cone's
// own direction defect is MEASURED here, not merely asserted: its rows check
// the held reading against a reference this file proves independently, in
// exact rational interval arithmetic over its own square-root and sine/cosine
// series (ratSqrtIv, ratSinCosIv), never by asking normal_bound.go for its own
// answer.

// normalFaceByRole finds the one face carrying the named feature role.
func normalFaceByRole(t *testing.T, b *decad.Body, role string) *decad.Face {
	t.Helper()
	for _, f := range b.Faces() {
		for _, origin := range f.Origins() {
			if origin.Role == role {
				return f
			}
		}
	}
	require.FailNow(t, `no face carries the role`, role)
	return nil
}

// ratVec is a held coordinate triple as exact rationals. A float64 IS a
// rational, so a vector the body publishes enters here exactly and every
// reading below stays exact — the arms work in float64 and what these
// assertions measure is what that arithmetic committed, so a check that
// rounded once itself could not tell the two apart.
type ratVec [3]*big.Rat

func ratVecOf(v r3.Vec) ratVec {
	return ratVec{
		new(big.Rat).SetFloat64(v.X),
		new(big.Rat).SetFloat64(v.Y),
		new(big.Rat).SetFloat64(v.Z),
	}
}

func ratVecSub(a, b ratVec) ratVec {
	var out ratVec
	for i := range out {
		out[i] = new(big.Rat).Sub(a[i], b[i])
	}
	return out
}

func ratVecDot(a, b ratVec) *big.Rat {
	sum := new(big.Rat)
	for i := range a {
		sum.Add(sum, new(big.Rat).Mul(a[i], b[i]))
	}
	return sum
}

func ratVecCross(a, b ratVec) ratVec {
	term := func(i, j int) *big.Rat {
		return new(big.Rat).Sub(new(big.Rat).Mul(a[i], b[j]), new(big.Rat).Mul(a[j], b[i]))
	}
	return ratVec{term(1, 2), term(2, 0), term(0, 1)}
}

// axialRadialOf is the exact vector from a surface's own axis to p,
// perpendicular to that axis: rel − a·(rel·a)/(a·a). That is the UNIT-axis
// spelling normal_bound.go states each arm is judged against, written so no
// square root enters and the answer stays rational.
func axialRadialOf(p, origin, axis r3.Vec) ratVec {
	rel := ratVecSub(ratVecOf(p), ratVecOf(origin))
	a := ratVecOf(axis)
	share := new(big.Rat).Quo(ratVecDot(rel, a), ratVecDot(a, a))
	var out ratVec
	for i := range out {
		out[i] = new(big.Rat).Sub(rel[i], new(big.Rat).Mul(a[i], share))
	}
	return out
}

// perpDefectSq is the squared part of a held direction n that NO scaling of d
// reaches: |n × d|²/|d|². The surface's exact unit normal is a scaling of d, so
// the whole of this sits inside the distance the reading's bound owes, and
// comparing it squared keeps the check free of any square root.
func perpDefectSq(n r3.Vec, d ratVec) *big.Rat {
	nv := ratVecOf(n)
	cross := ratVecCross(nv, d)
	return new(big.Rat).Quo(ratVecDot(cross, cross), ratVecDot(d, d))
}

// alongDefectSq is the same lower bound read the other way, for a surface
// whose exact normal direction needs a square root to write down but whose
// PLANE does not: w is a direction the exact unit normal is exactly
// perpendicular to, so whatever of the held reading lies along w is distance
// the bound owes. |n·w|²/|w|².
func alongDefectSq(n r3.Vec, w ratVec) *big.Rat {
	nv := ratVecOf(n)
	along := ratVecDot(nv, w)
	return new(big.Rat).Quo(new(big.Rat).Mul(along, along), ratVecDot(w, w))
}

// armDefectSq is a lower bound on how far one NormalAt reading sits from the
// exact unit normal of the surface its face is tagged with, one spelling per
// analytic arm. For Plane, Cylinder, Sphere and Torus that lower bound is
// computed exactly, over rationals with no rounding step anywhere. The Cone
// arm's exact normal needs a transcendental — a sine and cosine of the held
// half angle — so its spelling is instead the LOW end of a proven rational
// interval enclosure, built from this file's own square-root and sine/cosine
// series (ratSqrtIv, ratSinCosIv), never from normal_bound.go's. The reading's
// own outward sign is a float negation and every reading here is squared, so
// the sign never enters.
func armDefectSq(t *testing.T, f *decad.Face, p r3.Vec, n r3.Vec) *big.Rat {
	t.Helper()
	switch s := f.Surface().(type) {
	case decad.Plane:
		// r3.Frame holds its two in-plane axes and derives the normal as their
		// cross product on every call, so the exact direction is that cross
		// taken over rationals and the arm's reading is its rounded image.
		return perpDefectSq(n, ratVecCross(ratVecOf(s.Frame.U()), ratVecOf(s.Frame.V())))
	case decad.Cylinder:
		return perpDefectSq(n, axialRadialOf(p, s.Origin, s.Axis))
	case decad.Cone:
		return coneDefectSq(t, s, p, n)
	case decad.Sphere:
		return perpDefectSq(n, ratVecSub(ratVecOf(p), ratVecOf(s.Center)))
	case decad.Torus:
		// The exact normal runs from the tube centre at p's own azimuth to p,
		// so it lies in the plane the axis and that azimuth span and is
		// exactly perpendicular to their cross product — which the major
		// radius, the one term needing a square root, never enters.
		rel := ratVecSub(ratVecOf(p), ratVecOf(s.Center))
		return alongDefectSq(n, ratVecCross(ratVecOf(s.Axis), rel))
	default:
		require.FailNow(t, `no exact defect is written for this surface`, `%T`, s)
		return nil
	}
}

// requireBoundCoversDefect fails unless a published bound clears an exactly
// computed lower bound on the reading's own distance from its surface's exact
// unit normal. Both sides are compared squared over rationals, so the check
// itself rounds nothing.
func requireBoundCoversDefect(t *testing.T, bound float64, defectSq *big.Rat, label string) {
	t.Helper()
	b := new(big.Rat).SetFloat64(bound)
	require.NotNil(t, b, `%s: the published bound is finite`, label)
	got, _ := new(big.Rat).SetFrac(defectSq.Num(), defectSq.Denom()).Float64()
	require.LessOrEqual(t, defectSq.Cmp(new(big.Rat).Mul(b, b)), 0,
		`%s: the reading is %v from its surface's exact normal, past the bound %v it publishes`,
		label, math.Sqrt(got), bound)
}

// unitDefect is the exact |1 − |n|| of a held direction, computed over
// big.Rat so no float step can hide it. A normal reading whose own bound is
// below this number is understating: no unit vector sits closer to n than
// that.
func unitDefect(n r3.Vec) *big.Rat {
	sum := new(big.Rat)
	for _, c := range []float64{n.X, n.Y, n.Z} {
		r := new(big.Rat).SetFloat64(c)
		sum.Add(sum, new(big.Rat).Mul(r, r))
	}
	// |1 − |n|| = |1 − |n|²| / (1 + |n|). For a near-unit vector the divisor
	// is just under two, so halving the squared defect UNDERSTATES the true
	// distance slightly — which is what this assertion wants: a lower bound
	// the published bound must still clear.
	defect := new(big.Rat).Sub(sum, big.NewRat(1, 1))
	defect.Abs(defect)
	return defect.Quo(defect, big.NewRat(2, 1))
}

// ratIv is an exact rational interval [lo, hi], this file's own — kept apart
// from normal_bound.go's ratInterval so the Cone reference below is an
// independent witness, never the code under test agreeing with itself. Every
// operation is exact: a *big.Rat never rounds, so an enclosure widens only
// where a square root or a series remainder puts slack in.
type ratIv struct {
	lo, hi *big.Rat
}

// ratIvPoint encloses a single exact rational exactly, lo == hi.
func ratIvPoint(x *big.Rat) ratIv {
	return ratIv{lo: x, hi: x}
}

func ratIvAdd(a, b ratIv) ratIv {
	return ratIv{new(big.Rat).Add(a.lo, b.lo), new(big.Rat).Add(a.hi, b.hi)}
}

func ratIvSub(a, b ratIv) ratIv {
	return ratIv{new(big.Rat).Sub(a.lo, b.hi), new(big.Rat).Sub(a.hi, b.lo)}
}

// ratIvMul is the general product: the min and max of the four corner
// products, sound whatever signs a and b straddle.
func ratIvMul(a, b ratIv) ratIv {
	corners := [4]*big.Rat{
		new(big.Rat).Mul(a.lo, b.lo),
		new(big.Rat).Mul(a.lo, b.hi),
		new(big.Rat).Mul(a.hi, b.lo),
		new(big.Rat).Mul(a.hi, b.hi),
	}
	lo, hi := corners[0], corners[0]
	for _, c := range corners[1:] {
		if c.Cmp(lo) < 0 {
			lo = c
		}
		if c.Cmp(hi) > 0 {
			hi = c
		}
	}
	return ratIv{lo, hi}
}

// ratIvSquare is a·a, computed from the endpoints directly rather than via
// ratIvMul(a, a): an interval that straddles zero must report a zero low end,
// never a negative one, since no real square is negative.
func ratIvSquare(a ratIv) ratIv {
	lo2 := new(big.Rat).Mul(a.lo, a.lo)
	hi2 := new(big.Rat).Mul(a.hi, a.hi)
	lo, hi := lo2, hi2
	if hi.Cmp(lo) < 0 {
		lo, hi = hi, lo
	}
	if a.lo.Sign() < 0 && a.hi.Sign() > 0 {
		lo = new(big.Rat)
	}
	return ratIv{lo, hi}
}

// ratIvInv is 1/b for an interval whose low end is strictly positive, so b is
// positive throughout and the reciprocal is monotonic decreasing.
func ratIvInv(b ratIv) ratIv {
	return ratIv{new(big.Rat).Inv(b.hi), new(big.Rat).Inv(b.lo)}
}

// ratIvQuo is a/b for an interval b whose low end is strictly positive.
func ratIvQuo(a, b ratIv) ratIv {
	return ratIvMul(a, ratIvInv(b))
}

// ratSqrtIv is a verified rational enclosure of √x for x >= 0: lo² <= x <=
// hi², proven by exact comparison over big.Rat rather than trusted from the
// float seed. The seed comes from a 200-bit big.Float square root and is
// widened by a RELATIVE 2⁻¹⁵⁰ until both comparisons hold.
//
// The slack width is load-bearing: at a relative 2⁻⁴⁰ the enclosure is about
// 1e-12 wide, which swamps a Cone direction defect of about 5e-17 and would
// make a row's own lower bound exactly zero — a row that asserts nothing
// while passing. 2⁻¹⁵⁰ gives an enclosure about 1e-45 wide, far inside the
// 200-bit seed's own accuracy.
func ratSqrtIv(t *testing.T, x *big.Rat) ratIv {
	t.Helper()
	require.GreaterOrEqual(t, x.Sign(), 0, `ratSqrtIv: the input must be non-negative`)
	if x.Sign() == 0 {
		zero := new(big.Rat)
		return ratIv{zero, zero}
	}
	const prec = 200
	xf := new(big.Float).SetPrec(prec).SetRat(x)
	seed := new(big.Float).SetPrec(prec).Sqrt(xf)
	seedRat, _ := seed.Rat(nil)
	rel := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 150))
	for range 4 {
		widen := new(big.Rat).Mul(seedRat, rel)
		widen.Abs(widen)
		lo := new(big.Rat).Sub(seedRat, widen)
		if lo.Sign() < 0 {
			lo = new(big.Rat)
		}
		hi := new(big.Rat).Add(seedRat, widen)
		loSq := new(big.Rat).Mul(lo, lo)
		hiSq := new(big.Rat).Mul(hi, hi)
		if loSq.Cmp(x) <= 0 && hiSq.Cmp(x) >= 0 {
			return ratIv{lo, hi}
		}
		rel = new(big.Rat).Mul(rel, big.NewRat(2, 1))
	}
	require.FailNow(t, `ratSqrtIv: the enclosure did not converge`)
	return ratIv{}
}

// ratSinCosIv encloses sin(x) and cos(x) for an exact rational RADIAN x in
// [0, π/2), by summing the Maclaurin series over big.Rat through x^37/37! and
// charging the last included term's own magnitude as the remainder on both
// series. Both series are alternating with terms decreasing from the first
// for |x| < 1.57, so the first omitted term already bounds the tail and
// charging the last included one instead is more conservative still.
//
// No π enters anywhere, so nothing here depends on a platform's libm, and
// this must never call normal_bound.go's radSinCosInterval: the point of this
// reference is an independent witness, not the code under test agreeing with
// itself. Every cone half angle this file feeds it lands in range: it is an
// angle between an axis and a wall, read out in radians.
func ratSinCosIv(x *big.Rat) (sin, cos ratIv) {
	const maxPower = 37
	pow := make([]*big.Rat, maxPower+1)
	pow[0] = big.NewRat(1, 1)
	for i := 1; i <= maxPower; i++ {
		pow[i] = new(big.Rat).Mul(pow[i-1], x)
	}
	fact := make([]*big.Rat, maxPower+1)
	fact[0] = big.NewRat(1, 1)
	for i := 1; i <= maxPower; i++ {
		fact[i] = new(big.Rat).Mul(fact[i-1], big.NewRat(int64(i), 1))
	}
	term := func(n int) *big.Rat { return new(big.Rat).Quo(pow[n], fact[n]) }

	sinSum, sinLast := new(big.Rat), new(big.Rat)
	positive := true
	for n := 1; n <= maxPower; n += 2 {
		sinLast = term(n)
		if positive {
			sinSum.Add(sinSum, sinLast)
		} else {
			sinSum.Sub(sinSum, sinLast)
		}
		positive = !positive
	}

	cosSum, cosLast := big.NewRat(1, 1), big.NewRat(1, 1)
	positive = false
	for n := 2; n <= maxPower; n += 2 {
		cosLast = term(n)
		if positive {
			cosSum.Add(cosSum, cosLast)
		} else {
			cosSum.Sub(cosSum, cosLast)
		}
		positive = !positive
	}

	sin = ratIv{new(big.Rat).Sub(sinSum, sinLast), new(big.Rat).Add(sinSum, sinLast)}
	cos = ratIv{new(big.Rat).Sub(cosSum, cosLast), new(big.Rat).Add(cosSum, cosLast)}
	return sin, cos
}

// ratIvVec is a 3D vector of exact rational intervals, the interval sibling
// of ratVec above.
type ratIvVec [3]ratIv

// ratIvUnit encloses the exact unit vector of v: v scaled by a verified
// reciprocal-square-root enclosure of its own squared length. Fails loudly
// through require rather than dividing by a degenerate length silently.
func ratIvUnit(t *testing.T, v ratVec) ratIvVec {
	t.Helper()
	normSq := ratVecDot(v, v)
	require.Positive(t, normSq.Sign(), `ratIvUnit: a unit direction needs a nonzero vector`)
	length := ratSqrtIv(t, normSq)
	var out ratIvVec
	for i := range out {
		out[i] = ratIvQuo(ratIvPoint(v[i]), length)
	}
	return out
}

func ratIvVecScale(v ratIvVec, s ratIv) ratIvVec {
	var out ratIvVec
	for i := range out {
		out[i] = ratIvMul(v[i], s)
	}
	return out
}

func ratIvVecSub(a, b ratIvVec) ratIvVec {
	var out ratIvVec
	for i := range out {
		out[i] = ratIvSub(a[i], b[i])
	}
	return out
}

// coneDefectSq is armDefectSq's Cone arm: a proven LOWER bound on the squared
// distance between a held reading n and the tagged cone's exact unit normal
// r̂·cos(h) − â·sin(h), the closed form normal_bound.go's coneNormalAllow
// states. r̂ and â are this file's own proven unit enclosures of the radial
// and axis directions, h's sine and cosine are this file's own series, and
// nothing here reads normal_bound.go's radSinCosInterval or ivVec3Unit.
func coneDefectSq(t *testing.T, s decad.Cone, p, n r3.Vec) *big.Rat {
	t.Helper()
	radial := axialRadialOf(p, s.Origin, s.Axis)
	rhat := ratIvUnit(t, radial)
	ahat := ratIvUnit(t, ratVecOf(s.Axis))

	half, err := s.HalfAngle.In(units.Radian)
	require.NoError(t, err, `a cone's half angle is an angle`)
	halfRat := new(big.Rat).SetFloat64(half)
	require.NotNil(t, halfRat, `the half angle is a finite float and so an exact rational`)
	sin, cos := ratSinCosIv(halfRat)

	m := ratIvVecSub(ratIvVecScale(rhat, cos), ratIvVecScale(ahat, sin))
	nv := ratVecOf(n)
	sum := ratIvPoint(new(big.Rat))
	for i := range m {
		sum = ratIvAdd(sum, ratIvSquare(ratIvSub(ratIvPoint(nv[i]), m[i])))
	}
	return sum.lo
}

func TestNormalAtExactnessIsProven(t *testing.T) {
	t.Run(`a rotated placement's cap normal is Approximate`, func(t *testing.T) {
		s, p := plateSketch(t)
		doc := decad.New()
		body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
		require.NoError(t, err)
		rotation, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
		require.NoError(t, err)
		placed, err := body.Placed(rotation)
		require.NoError(t, err)

		for _, role := range []string{roleCapStart, roleCapEnd} {
			f := normalFaceByRole(t, placed, role)
			at := f.Surface().(decad.Plane).Frame.Origin()
			n, err := f.NormalAt(at)
			require.NoError(t, err)
			require.Equal(t, decad.Approximate, n.Exactness, `%s: a rounded rotation image is not an exact unit normal`, role)
			bound, err := n.Bound.In(units.One)
			require.NoError(t, err)
			require.Positive(t, bound, `%s: the bound must be positive`, role)

			// The bound is not merely nonzero: it covers the held vector's own
			// distance from the nearest unit vector, which is what disqualifies
			// it from being any surface's exact normal.
			defect, _ := unitDefect(n.Value).Float64()
			require.Positive(t, defect, `%s: the held normal is not of unit length`, role)
			require.GreaterOrEqual(t, bound, defect, `%s: the bound must cover the unit-length defect`, role)
		}
	})

	t.Run(`an unplaced cone normal is Approximate`, func(t *testing.T) {
		wall, cone := coneWall(t)
		require.True(t, cone.HalfAngle.Equal(units.Degrees(45), 1e-12), `the half angle is a whole number of degrees`)

		at := r3.NewVec(5, 5, 0)
		n, err := wall.NormalAt(at)
		require.NoError(t, err)
		// The cone opens toward +u from its apex at the origin, so the outward
		// normal leans away from the axis and back toward the apex.
		require.InDelta(t, -1/math.Sqrt(2), n.Value.X, 1e-9)
		require.InDelta(t, 1/math.Sqrt(2), n.Value.Y, 1e-9)
		require.Equal(t, decad.Approximate, n.Exactness, `a float cosine and sine do not build an exact unit normal`)
		bound, err := n.Bound.In(units.One)
		require.NoError(t, err)
		require.Positive(t, bound)

		defect, _ := unitDefect(n.Value).Float64()
		require.Positive(t, defect, `cos² + sin² off the float library is not one`)
		require.GreaterOrEqual(t, bound, defect, `the bound must cover the unit-length defect`)
		// The reading is still a usable direction: the overclaim was the
		// exactness, never the value.
		require.Less(t, bound, 1e-14, `the bound stays at rounding scale`)
	})

	t.Run(`a normal exact under the lift stays Exact`, func(t *testing.T) {
		s, p := plateSketch(t)
		doc := decad.New()
		body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
		require.NoError(t, err)
		shift, err := r3.Translation(r3.NewVec(1000, -250, 80))
		require.NoError(t, err)
		placed, err := body.Placed(shift)
		require.NoError(t, err)

		// A translation moves no direction at all, and the plate's own cap
		// frame normal is an axis vector, so the lift reproduces it exactly.
		f := normalFaceByRole(t, placed, roleCapEnd)
		n, err := f.NormalAt(f.Surface().(decad.Plane).Frame.Origin())
		require.NoError(t, err)
		require.Equal(t, decad.Exact, n.Exactness)
		bound, err := n.Bound.In(units.One)
		require.NoError(t, err)
		require.Zero(t, bound)
		require.Equal(t, r3.NewVec(0, 0, 1), n.Value)

		// Every side wall of the same plate is axis aligned too, so the whole
		// unplaced body reads Exact rather than only its caps.
		unplacedExact := 0
		for _, face := range placed.Faces() {
			m, err := face.NormalAt(face.Surface().(decad.Plane).Frame.Origin())
			require.NoError(t, err)
			require.Equal(t, decad.Exact, m.Exactness)
			unplacedExact++
		}
		require.Equal(t, 6, unplacedExact)
	})
}

// TestNormalAtPlaneArmCoversItsFrameCross is the Plane arm's own contract, and
// it needs its own test because the plate's six faces are all Plane and the
// one-of-a-kind lookup below can name none of them. r3.Frame holds only its
// origin and its two in-plane axes and derives the normal as their cross
// product on EVERY call, so what the arm hands back is six products and three
// differences, each rounded — not a stored fact. A bound covering only the
// triple's unit-length defect understates that, and this rotation is one an
// ordinary caller reaches: it leaves two of the plate's six faces reading a
// direction measurably off the exact cross of their own frame axes.
func TestNormalAtPlaneArmCoversItsFrameCross(t *testing.T) {
	s, p := plateSketch(t)
	body, err := decad.New().Extrude(s, p, decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	require.NoError(t, err)
	placed, err := body.Placed(planeArmSpin(t))
	require.NoError(t, err)

	faces := placed.Faces()
	require.Len(t, faces, 6, `the plate is bounded by six planes`)
	departed := 0
	for i, f := range faces {
		at := f.Surface().(decad.Plane).Frame.Origin()
		n, err := f.NormalAt(at)
		require.NoError(t, err)
		bound, err := n.Bound.In(units.One)
		require.NoError(t, err)

		defectSq := armDefectSq(t, f, at, n.Value)
		requireBoundCoversDefect(t, bound, defectSq, fmt.Sprintf(`face %d`, i))
		length := unitDefect(n.Value)
		lengthDefect, _ := length.Float64()
		require.GreaterOrEqual(t, bound, lengthDefect, `face %d: the bound must cover the unit-length defect too`, i)
		if defectSq.Sign() > 0 {
			departed++
		}
	}
	// Without this the loop would pass against a bound that charged the
	// direction nothing: what makes it a proof is that the cross product really
	// did leave the exact answer on this placement.
	require.Positive(t, departed, `the placement must genuinely tilt a frame's own cross product`)
}

// planeArmSpin is the rotation the Plane arm's contract is read under: the
// plate is drawn at the sketch origin and carried away from the world origin by
// the placement alone, so no platform's arrangement of the section is left
// deciding a weld band.
func planeArmSpin(t *testing.T) r3.Transform {
	t.Helper()
	spin, err := r3.Rotation(
		r3.NewVec(-1.0356782981096022, 0.5660898094817096, 0.19792151244355746),
		units.Degrees(129.7127487549914),
	)
	require.NoError(t, err)
	return spin
}

// plateCapFace extrudes the plate drawn at the sketch origin and returns the
// face carrying the named cap role, under motion when one is given.
func plateCapFace(t *testing.T, role string, motion *r3.Transform) *decad.Face {
	t.Helper()
	s, p := plateSketch(t)
	body, err := decad.New().Extrude(s, p, decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	require.NoError(t, err)
	if motion != nil {
		body, err = body.Placed(*motion)
		require.NoError(t, err)
	}
	return normalFaceByRole(t, body, role)
}

// placedFace evaluates build, optionally under motion, and returns the one
// face carrying kind. Reading the surface's own published parameters back off
// that face is what lets every fixture below name its sample point in the
// PLACED frame without re-deriving the placement itself.
func placedFace(t *testing.T, build func(t *testing.T) *decad.Body, kind decad.SurfaceKind, motion *r3.Transform) *decad.Face {
	t.Helper()
	body := build(t)
	if motion != nil {
		placed, err := body.Placed(*motion)
		require.NoError(t, err)
		body = placed
	}
	var out *decad.Face
	for _, f := range body.Faces() {
		if f.Surface().Kind() != kind {
			continue
		}
		require.Nil(t, out, `the fixture builds one face of this kind`)
		out = f
	}
	require.NotNil(t, out)
	return out
}

// perpTo is a unit direction perpendicular to a, seeded from whichever world
// axis a leans on least, so a fixture picks the same one on every platform.
func perpTo(t *testing.T, a r3.Vec) r3.Vec {
	t.Helper()
	axis, ok := a.Normalize()
	require.True(t, ok)
	seed := r3.NewVec(1, 0, 0)
	if math.Abs(axis.Dot(seed)) > 0.5 {
		seed = r3.NewVec(0, 1, 0)
	}
	w, ok := axis.Cross(seed).Normalize()
	require.True(t, ok)
	return w
}

// normalBallBody and normalTorusBody are the shipped clearance fixtures under
// this file's own one-argument shape: a ball of radius 5 at the origin, and a
// torus of major 10 and minor 3 about the world X axis.
func normalBallBody(t *testing.T) *decad.Body {
	t.Helper()
	return ballBody(t, decad.New(), 5)
}

func normalTorusBody(t *testing.T) *decad.Body {
	t.Helper()
	return torusBody(t, decad.New(), 10, 3)
}

// holeWallBody extrudes the plate carrying one circular hole, whose wall is
// the shipped REVERSED cylinder: its material lies outside the cylinder, so
// the outward normal is the geometric one negated.
func holeWallBody(t *testing.T) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(70, 30), 10)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			prof = p
		}
	}
	require.NotNil(t, prof)
	body, err := decad.New().Extrude(s, prof, decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// TestNormalAtArmBoundCoversItsOwnDefect is the same contract on the arms
// whose exact normal is a computation off the tag's own held numbers, which is
// every one of them: Plane — whose frame derives its normal as a cross product
// on every call — along with Cylinder, Sphere, Torus and Cone. Each row reads
// one face's own published surface parameters, samples the reading at a point
// named in that surface's own frame, and measures how far the float triple the
// arm handed back sits from the exact unit normal those parameters give. For
// Plane, Cylinder, Sphere and Torus that exact normal is itself rational, so
// the measure is exact; the Cone's is not, since its normal needs a sine and
// cosine of the held half angle, so its rows measure against a proven
// rational-interval enclosure instead (armDefectSq's Cone case). A bound below
// that distance is an understatement, whatever exactness it publishes beside
// it.
func TestNormalAtArmBoundCoversItsOwnDefect(t *testing.T) {
	spin, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	// A cylinder point at a whole-number azimuth off the world axes, and a
	// torus point at a whole-number tube angle off its own equator: both are
	// float cosines and sines, which is exactly where an arm's arithmetic
	// stops being exact.
	tilt := func(a, b r3.Vec, deg float64) r3.Vec {
		sin, cos := math.Sincos(deg * math.Pi / 180)
		return a.Scale(cos).Add(b.Scale(sin))
	}
	cylinderPoint := func(t *testing.T, f *decad.Face, deg float64) (r3.Vec, r3.Vec) {
		t.Helper()
		s := f.Surface().(decad.Cylinder)
		w := perpTo(t, s.Axis)
		axis, _ := s.Axis.Normalize()
		dir := tilt(w, axis.Cross(w), deg)
		return s.Origin.Add(dir.Scale(s.Radius.Mag())), dir
	}
	spherePoint := func(t *testing.T, f *decad.Face, dir r3.Vec) (r3.Vec, r3.Vec) {
		t.Helper()
		s := f.Surface().(decad.Sphere)
		unit, ok := dir.Normalize()
		require.True(t, ok)
		return s.Center.Add(unit.Scale(s.Radius.Mag())), unit
	}
	torusPoint := func(t *testing.T, f *decad.Face, azimuth, tube float64) (r3.Vec, r3.Vec) {
		t.Helper()
		s := f.Surface().(decad.Torus)
		axis, ok := s.Axis.Normalize()
		require.True(t, ok)
		w := perpTo(t, s.Axis)
		radial := tilt(w, axis.Cross(w), azimuth)
		dir := tilt(radial, axis, tube)
		center := s.Center.Add(radial.Scale(s.Major.Mag()))
		return center.Add(dir.Scale(s.Minor.Mag())), dir
	}
	// conePoint samples at a point off the axis by radius 5 and along the axis
	// by 5mm — a point the Cone arm's own formula answers for whether or not
	// it sits on the actual wall, since the projection it computes depends on
	// no radius at all.
	conePoint := func(t *testing.T, f *decad.Face, deg float64) (r3.Vec, r3.Vec) {
		t.Helper()
		s := f.Surface().(decad.Cone)
		w := perpTo(t, s.Axis)
		axis, ok := s.Axis.Normalize()
		require.True(t, ok)
		dir := tilt(w, axis.Cross(w), deg)
		p := s.Origin.Add(dir.Scale(5)).Add(axis.Scale(5))
		half, err := s.HalfAngle.In(units.Radian)
		require.NoError(t, err)
		n := dir.Scale(math.Cos(half)).Sub(axis.Scale(math.Sin(half)))
		return p, n
	}

	for _, tc := range []struct {
		name string
		// at names the sample point and the direction the surface itself says
		// the outward normal takes there, both in the face's OWN frame.
		at    func(t *testing.T) (*decad.Face, r3.Vec, r3.Vec)
		exact bool
		// directionDominates marks the one row that also asserts the direction
		// defect swamps the unit-length one, rather than merely being covered
		// by the same bound as it.
		directionDominates bool
	}{
		{
			name: `a plate cap on a world axis is exact`,
			at: func(t *testing.T) (*decad.Face, r3.Vec, r3.Vec) {
				f := plateCapFace(t, roleCapEnd, nil)
				return f, f.Surface().(decad.Plane).Frame.Origin(), r3.NewVec(0, 0, 1)
			},
			exact: true,
		},
		{
			name: `a placed plate cap carries its frame cross product's own rounding`,
			at: func(t *testing.T) (*decad.Face, r3.Vec, r3.Vec) {
				motion := planeArmSpin(t)
				f := plateCapFace(t, roleCapEnd, &motion)
				return f, f.Surface().(decad.Plane).Frame.Origin(), motion.ApplyDir(r3.NewVec(0, 0, 1))
			},
		},
		{
			name: `a cylinder wall on a world axis is exact`,
			at: func(t *testing.T) (*decad.Face, r3.Vec, r3.Vec) {
				f := placedFace(t, func(t *testing.T) *decad.Body { return circleProfile(t, 20, 10) }, decad.KindCylinder, nil)
				p, dir := cylinderPoint(t, f, 0)
				return f, p, dir
			},
			exact: true,
		},
		{
			name: `a cylinder wall off the world axes is not`,
			at: func(t *testing.T) (*decad.Face, r3.Vec, r3.Vec) {
				f := placedFace(t, func(t *testing.T) *decad.Body { return circleProfile(t, 20, 10) }, decad.KindCylinder, nil)
				p, dir := cylinderPoint(t, f, 37)
				return f, p, dir
			},
		},
		{
			name: `a placed cylinder wall carries the rotation's own rounding`,
			at: func(t *testing.T) (*decad.Face, r3.Vec, r3.Vec) {
				f := placedFace(t, func(t *testing.T) *decad.Body { return circleProfile(t, 20, 10) }, decad.KindCylinder, &spin)
				p, dir := cylinderPoint(t, f, 37)
				return f, p, dir
			},
		},
		{
			name: `a hole wall reads its reversed outward normal under the same bound`,
			at: func(t *testing.T) (*decad.Face, r3.Vec, r3.Vec) {
				f := placedFace(t, holeWallBody, decad.KindCylinder, nil)
				p, dir := cylinderPoint(t, f, 37)
				// The material lies OUTSIDE the hole's cylinder, so the
				// outward normal points back at the axis.
				return f, p, dir.Scale(-1)
			},
		},
		{
			name: `a sphere on a world axis is exact`,
			at: func(t *testing.T) (*decad.Face, r3.Vec, r3.Vec) {
				f := placedFace(t, normalBallBody, decad.KindSphere, nil)
				p, dir := spherePoint(t, f, r3.NewVec(0, 1, 0))
				return f, p, dir
			},
			exact: true,
		},
		{
			name: `a sphere off the world axes is not`,
			at: func(t *testing.T) (*decad.Face, r3.Vec, r3.Vec) {
				f := placedFace(t, normalBallBody, decad.KindSphere, nil)
				p, dir := spherePoint(t, f, r3.NewVec(1, 2, 3))
				return f, p, dir
			},
		},
		{
			name: `a placed sphere carries the rotation's own rounding`,
			at: func(t *testing.T) (*decad.Face, r3.Vec, r3.Vec) {
				f := placedFace(t, normalBallBody, decad.KindSphere, &spin)
				p, dir := spherePoint(t, f, r3.NewVec(1, 2, 3))
				return f, p, dir
			},
		},
		{
			name: `a torus on its own equator is exact`,
			at: func(t *testing.T) (*decad.Face, r3.Vec, r3.Vec) {
				f := placedFace(t, normalTorusBody, decad.KindTorus, nil)
				p, dir := torusPoint(t, f, 0, 0)
				return f, p, dir
			},
			exact: true,
		},
		{
			name: `a torus off its own equator is not`,
			at: func(t *testing.T) (*decad.Face, r3.Vec, r3.Vec) {
				f := placedFace(t, normalTorusBody, decad.KindTorus, nil)
				p, dir := torusPoint(t, f, 40, 30)
				return f, p, dir
			},
		},
		{
			name: `a placed torus carries the rotation's own rounding`,
			at: func(t *testing.T) (*decad.Face, r3.Vec, r3.Vec) {
				f := placedFace(t, normalTorusBody, decad.KindTorus, &spin)
				p, dir := torusPoint(t, f, 40, 30)
				return f, p, dir
			},
		},
		{
			name: `a cone wall on a world axis carries its own cosine and sine`,
			at: func(t *testing.T) (*decad.Face, r3.Vec, r3.Vec) {
				f := placedFace(t, func(t *testing.T) *decad.Body { return coneBody(t, 10) }, decad.KindCone, nil)
				p, dir := conePoint(t, f, 0)
				return f, p, dir
			},
			directionDominates: true,
		},
		{
			name: `a cone wall off the world axes carries more of it`,
			at: func(t *testing.T) (*decad.Face, r3.Vec, r3.Vec) {
				f := placedFace(t, func(t *testing.T) *decad.Body { return coneBody(t, 10) }, decad.KindCone, nil)
				p, dir := conePoint(t, f, 37)
				return f, p, dir
			},
		},
		{
			name: `a placed cone carries the rotation's own rounding`,
			at: func(t *testing.T) (*decad.Face, r3.Vec, r3.Vec) {
				f := placedFace(t, func(t *testing.T) *decad.Body { return coneBody(t, 10) }, decad.KindCone, &spin)
				p, dir := conePoint(t, f, 37)
				return f, p, dir
			},
		},
		{
			name: `a cone whose half angle is not 45 degrees`,
			at: func(t *testing.T) (*decad.Face, r3.Vec, r3.Vec) {
				f := placedFace(t, func(t *testing.T) *decad.Body { return coneBody(t, 5) }, decad.KindCone, nil)
				p, dir := conePoint(t, f, 0)
				return f, p, dir
			},
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			f, at, want := tc.at(t)
			n, err := f.NormalAt(at)
			require.NoError(t, err)
			require.InDelta(t, want.X, n.Value.X, 1e-9, `the reading is the direction the surface names`)
			require.InDelta(t, want.Y, n.Value.Y, 1e-9)
			require.InDelta(t, want.Z, n.Value.Z, 1e-9)

			bound, err := n.Bound.In(units.One)
			require.NoError(t, err)
			defectSq := armDefectSq(t, f, at, n.Value)
			requireBoundCoversDefect(t, bound, defectSq, tc.name)
			length := unitDefect(n.Value)
			lengthDefect, _ := length.Float64()
			require.GreaterOrEqual(t, bound, lengthDefect, `the bound must cover the unit-length defect too`)

			if tc.exact {
				require.Equal(t, decad.Exact, n.Exactness)
				require.Zero(t, bound)
				require.Zero(t, defectSq.Sign(), `an Exact reading owes an exactly zero direction defect`)
				require.Zero(t, length.Sign(), `and an exactly unit length`)
				return
			}
			require.Equal(t, decad.Approximate, n.Exactness)
			require.Positive(t, bound)
			require.Less(t, bound, 1e-14, `the bound stays at rounding scale`)
			// Without this the row would still pass against a bound of zero:
			// what makes it a proof is that the arm's own arithmetic really
			// did leave the exact answer here.
			require.Positive(t, defectSq.Sign()+length.Sign(),
				`the arm's own arithmetic must genuinely depart, or the row proves nothing`)
			if tc.directionDominates {
				// Without this the row would still pass against a bound that
				// charged the direction nothing but the length — exactly the
				// state the Cone arm's coverage was in before this test
				// measured it. Measured ratio on this fixture is 4.824
				// (4.895157e-17 against 1.014654e-17), so the factor 2 here
				// leaves 2.4x of margin.
				fourLengthSq := new(big.Rat).Mul(new(big.Rat).Mul(length, length), big.NewRat(4, 1))
				require.GreaterOrEqual(t, defectSq.Cmp(fourLengthSq), 0,
					`%s: the direction defect must be at least twice the unit-length defect`, tc.name)
			}
		})
	}
}

// TestNormalAtRefusesEveryUntaggedSurface pins the SET the arms answer for.
// The five analytic variants above are the whole of it: a `Faceted` face — the
// tag every boolean-produced face carries — has no closed form to evaluate, so
// its answer waits on the faceted certificate stage and every reading refuses
// today (docs/spline-design.md §7, docs/payload-verification-design.md §5.4,
// §13). A refusal is the honest answer; a direction read off one held facet
// would be exact for the facet and wrong for the part it stands for.
func TestNormalAtRefusesEveryUntaggedSurface(t *testing.T) {
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 20, 20, 8)
	tool := translated(t, diskBody(t, doc, 14, 6, 2), 0, 0, -6)
	got, err := decad.Cut(plate, tool)
	require.NoError(t, err)

	refused := 0
	for _, f := range got.Faces() {
		require.Equal(t, decad.KindFaceted, f.Surface().Kind())
		at := f.Loops()[0].CoEdges()[0].Start().Position().Value
		_, err := f.NormalAt(at)
		require.ErrorIs(t, err, decad.ErrUnsupported,
			`a faceted face has no closed-form normal to publish a bound for`)
		refused++
	}
	require.Equal(t, 7, refused, `4 plate sides + 2 drilled caps + the hole wall`)
}

// TestNormalAtArmDegeneraciesRefuse pins the other end of each arm: a point
// the surface gives no direction at all is ErrDegenerate, never a bounded
// reading of an arbitrary direction.
func TestNormalAtArmDegeneraciesRefuse(t *testing.T) {
	for _, tc := range []struct {
		name string
		at   func(t *testing.T) (*decad.Face, r3.Vec)
	}{
		{
			name: `a point on the cylinder axis`,
			at: func(t *testing.T) (*decad.Face, r3.Vec) {
				f := placedFace(t, func(t *testing.T) *decad.Body { return circleProfile(t, 20, 10) }, decad.KindCylinder, nil)
				return f, f.Surface().(decad.Cylinder).Origin
			},
		},
		{
			name: `the sphere centre`,
			at: func(t *testing.T) (*decad.Face, r3.Vec) {
				f := placedFace(t, normalBallBody, decad.KindSphere, nil)
				return f, f.Surface().(decad.Sphere).Center
			},
		},
		{
			name: `a point on the torus axis`,
			at: func(t *testing.T) (*decad.Face, r3.Vec) {
				f := placedFace(t, normalTorusBody, decad.KindTorus, nil)
				return f, f.Surface().(decad.Torus).Center
			},
		},
		{
			name: `a torus tube centre, which is off the axis and still has no normal`,
			at: func(t *testing.T) (*decad.Face, r3.Vec) {
				f := placedFace(t, normalTorusBody, decad.KindTorus, nil)
				s := f.Surface().(decad.Torus)
				axis, ok := s.Axis.Normalize()
				require.True(t, ok)
				w := perpTo(t, s.Axis)
				_ = axis
				return f, s.Center.Add(w.Scale(s.Major.Mag()))
			},
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			f, at := tc.at(t)
			_, err := f.NormalAt(at)
			require.ErrorIs(t, err, decad.ErrDegenerate)
		})
	}
}

// coneBody revolves the right triangle (0,0)-(10,0)-(10,top) about the u
// axis, giving one Cone wall whose body no placement ever touched. top = 10
// is the 45° cone; a smaller top gives a half angle away from 45° so a row
// built on it cannot lean on cos(h) == sin(h).
func coneBody(t *testing.T, top float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	base := s.CreatePoint(10, 0)
	tip := s.CreatePoint(10, top)
	s.Fix(o)
	s.Fix(base)
	s.Fix(tip)
	s.CreateLine(o, base)
	s.CreateLine(base, tip)
	s.CreateLine(tip, o)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	doc := decad.New()
	body, err := doc.Revolve(s, s.Profiles()[0], uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

// coneWall picks out coneBody(t, 10)'s one Cone wall, whose half angle is a
// whole number of degrees.
func coneWall(t *testing.T) (*decad.Face, decad.Cone) {
	t.Helper()
	body := coneBody(t, 10)
	for _, f := range body.Faces() {
		if c, ok := f.Surface().(decad.Cone); ok {
			return f, c
		}
	}
	require.FailNow(t, `the revolve built no cone wall`)
	return nil, decad.Cone{}
}
