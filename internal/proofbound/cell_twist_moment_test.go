package proofbound_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// refMomentPoly is the map-backed polynomial CellTwistMoment used before it
// moved to a fixed array. It stays here as the reference the array version
// must match exactly.
type refMomentPoly map[[2]int]proofarith.Dyadic

func refMomentPolyAdd(p refMomentPoly, degree [2]int, term proofarith.Dyadic) {
	if term.Sign() == 0 {
		return
	}
	p[degree] = proofarith.DyAdd(p[degree], term)
}

func refMomentPolyMul(a, b refMomentPoly) refMomentPoly {
	out := make(refMomentPoly)
	for da, ca := range a {
		for db, cb := range b {
			refMomentPolyAdd(out, [2]int{da[0] + db[0], da[1] + db[1]}, proofarith.DyMul(ca, cb))
		}
	}
	return out
}

func refBilinearPatchMomentIntegral(q0, a, b, twist proofarith.DyV3, axis int) *big.Rat {
	q := refMomentPoly{
		{0, 0}: q0[axis],
		{1, 0}: a[axis],
		{0, 1}: b[axis],
		{1, 1}: twist[axis],
	}
	n0 := proofarith.DvCross(a, b)
	ns := proofarith.DvCross(a, twist)
	nr := proofarith.DvCross(twist, b)
	n := refMomentPoly{
		{0, 0}: n0[axis],
		{1, 0}: ns[axis],
		{0, 1}: nr[axis],
	}
	integrand := refMomentPolyMul(refMomentPolyMul(q, q), n)
	total := proofarith.DyZero()
	for degree, coefficient := range integrand {
		den := int64((degree[0] + 1) * (degree[1] + 1))
		total = proofarith.DyAdd(total, proofarith.DyMul(coefficient, proofarith.DyInt(144/den)))
	}
	return proofarith.QuoRat(new(big.Rat), total.Rat(), big.NewRat(144, 1))
}

func refTriangleMomentIntegral(q0, q1, q2 proofarith.DyV3, axis int) *big.Rat {
	e1 := proofarith.DvSub(q1, q0)
	e2 := proofarith.DvSub(q2, q0)
	q := refMomentPoly{
		{0, 0}: q0[axis],
		{1, 0}: e1[axis],
		{0, 1}: e2[axis],
	}
	q2Poly := refMomentPolyMul(q, q)
	n := proofarith.DvCross(e1, e2)[axis]
	total := proofarith.DyZero()
	for degree, coefficient := range q2Poly {
		num := int64(1)
		for i := 2; i <= degree[0]; i++ {
			num *= int64(i)
		}
		for i := 2; i <= degree[1]; i++ {
			num *= int64(i)
		}
		den := int64(1)
		for i := 2; i <= degree[0]+degree[1]+2; i++ {
			den *= int64(i)
		}
		total = proofarith.DyAdd(total, proofarith.DyMul(coefficient, proofarith.DyInt(24*num/den)))
	}
	return proofarith.QuoRat(new(big.Rat), proofarith.DyMul(total, n).Rat(), big.NewRat(24, 1))
}

func refCellTwistMoment(vLo, vHi, wLo, wHi, anchor r3.Vec) proofbound.RatV3 {
	qLo := proofbound.HeldDelta(vLo, anchor)
	qHi := proofbound.HeldDelta(vHi, anchor)
	qWLo := proofbound.HeldDelta(wLo, anchor)
	qWHi := proofbound.HeldDelta(wHi, anchor)
	a := proofarith.DvSub(qHi, qLo)
	b := proofarith.DvSub(qWLo, qLo)
	twist := proofarith.DvSub(proofarith.DvSub(qLo, qHi), proofarith.DvSub(qWLo, qWHi))

	var out proofbound.RatV3
	for axis := range out {
		patch := refBilinearPatchMomentIntegral(qLo, a, b, twist, axis)
		held0 := refTriangleMomentIntegral(qLo, qHi, qWHi, axis)
		held1 := refTriangleMomentIntegral(qLo, qWHi, qWLo, axis)
		out[axis] = proofarith.SubRat(new(big.Rat), patch, held0)
		proofarith.SubRat(out[axis], out[axis], held1)
		proofarith.MulRat(out[axis], out[axis], big.NewRat(12, 1))
	}
	return out
}

// momentQuad is one cell's four corners and the anchor they are measured from.
type momentQuad struct {
	vLo, vHi, wLo, wHi, anchor r3.Vec
}

// momentScalar draws a float with a random mantissa and, when mixed is set, a
// random binary exponent, so coefficients of different widths meet in a sum.
func momentScalar(rng *rand.Rand, mixed bool) float64 {
	x := rng.Float64()*2 - 1
	switch rng.IntN(8) {
	case 0:
		return 0
	case 1:
		x = math.Round(x * 16)
	}
	if !mixed {
		return x
	}
	return math.Ldexp(x, rng.IntN(121)-60)
}

func momentVec(rng *rand.Rand, mixed bool) r3.Vec {
	return r3.Vec{X: momentScalar(rng, mixed), Y: momentScalar(rng, mixed), Z: momentScalar(rng, mixed)}
}

func momentQuads(rng *rand.Rand, kind string, mixed bool) momentQuad {
	anchor := momentVec(rng, mixed)
	vLo := momentVec(rng, mixed)
	u := momentVec(rng, mixed)
	v := momentVec(rng, mixed)
	quad := momentQuad{vLo: vLo, vHi: vLo.Add(u), wLo: vLo.Add(v), anchor: anchor}
	switch kind {
	case "planar":
		// A corner in the plane of the other three: any combination of u and v.
		quad.wHi = vLo.Add(u.Scale(math.Round(rng.Float64()*8 - 4))).Add(v.Scale(math.Round(rng.Float64()*8 - 4)))
	default:
		quad.wHi = momentVec(rng, mixed)
	}
	if kind == "reflected" {
		// Swapping the two neighbours reverses the cell's orientation.
		quad.vHi, quad.wLo = quad.wLo, quad.vHi
	}
	return quad
}

func TestCellTwistMomentMatchesMapReference(t *testing.T) {
	t.Parallel()
	for _, kind := range []string{"planar", "nonplanar", "reflected"} {
		for _, mixed := range []bool{false, true} {
			name := kind
			if mixed {
				name += "/mixed"
			}
			t.Run(name, func(t *testing.T) {
				t.Parallel()
				rng := rand.New(rand.NewPCG(7, 11))
				for range 300 {
					q := momentQuads(rng, kind, mixed)
					want := refCellTwistMoment(q.vLo, q.vHi, q.wLo, q.wHi, q.anchor)
					got := proofbound.CellTwistMoment(q.vLo, q.vHi, q.wLo, q.wHi, q.anchor)
					for axis := range got {
						require.Zero(t, got[axis].Cmp(want[axis]), "axis %d quad %+v", axis, q)
					}
				}
			})
		}
	}
}

func BenchmarkCellTwistMoment(b *testing.B) {
	rng := rand.New(rand.NewPCG(3, 5))
	quads := make([]momentQuad, 64)
	for i := range quads {
		quads[i] = momentQuads(rng, "nonplanar", false)
	}
	b.ReportAllocs()
	for i := 0; b.Loop(); i++ {
		q := quads[i%len(quads)]
		_ = proofbound.CellTwistMoment(q.vLo, q.vHi, q.wLo, q.wHi, q.anchor)
	}
}

func BenchmarkCellTwistMomentMapReference(b *testing.B) {
	rng := rand.New(rand.NewPCG(3, 5))
	quads := make([]momentQuad, 64)
	for i := range quads {
		quads[i] = momentQuads(rng, "nonplanar", false)
	}
	b.ReportAllocs()
	for i := 0; b.Loop(); i++ {
		q := quads[i%len(quads)]
		_ = refCellTwistMoment(q.vLo, q.vHi, q.wLo, q.wHi, q.anchor)
	}
}
