package meshbool_test

// CI: isolated-tests

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func requireSameRat(t *testing.T, want, got *big.Rat, msg string) {
	t.Helper()
	require.Zero(t, want.Cmp(got), msg)
	require.Equal(t, want.RatString(), got.RatString(), msg)
	require.Zero(t, want.Num().Cmp(got.Num()), msg)
	require.Zero(t, want.Denom().Cmp(got.Denom()), msg)
	wf, we := want.Float64()
	gf, ge := got.Float64()
	require.Equal(t, math.Float64bits(wf), math.Float64bits(gf), msg)
	require.Equal(t, we, ge, msg)
}

func requireSamePanic(t *testing.T, p proof.Xpt, v r3.Vec) {
	t.Helper()
	var want, got any
	func() {
		defer func() { want = recover() }()
		referenceCoordDistance(p, v)
	}()
	func() {
		defer func() { got = recover() }()
		meshbool.CoordDistance(p, v)
	}()
	require.NotNil(t, want)
	require.Equal(t, want, got)
}

func TestCoordDistanceMatchesReferenceOnRandomInputs(t *testing.T) {
	r := rand.New(rand.NewPCG(0xc0d, 1))
	zero, nonzero := 0, 0
	for range 20000 {
		v := coordDistanceVec(r)
		p := coordDistancePoint(r, v)
		want, got := referenceCoordDistance(p, v), meshbool.CoordDistance(p, v)
		requireSameRat(t, want, got, "random input")
		if want.Sign() == 0 {
			zero++
		} else {
			nonzero++
		}
	}
	require.Greater(t, zero, 2000)
	require.Greater(t, nonzero, 2000)
}

func TestCoordDistanceExactVertexIsZero(t *testing.T) {
	r := rand.New(rand.NewPCG(0xc0d, 2))
	for range 2000 {
		v := coordDistanceVec(r)
		got := meshbool.CoordDistance(proof.XptOf(v), v)
		require.Zero(t, got.Sign())
		requireSameRat(t, referenceCoordDistance(proof.XptOf(v), v), got, "exact vertex")
	}
	// Signed zeros are the same exact zero.
	nz := math.Copysign(0, -1)
	for _, v := range []r3.Vec{{}, {X: nz, Y: nz, Z: nz}, {X: nz, Y: 1, Z: 0}} {
		for _, p := range []proof.Xpt{proof.XptOf(r3.Vec{}), proof.XptOf(v)} {
			requireSameRat(t, referenceCoordDistance(p, v), meshbool.CoordDistance(p, v), "zero")
		}
	}
}

func TestCoordDistanceNonExactKeepsExactBound(t *testing.T) {
	// One coordinate one ulp away, at several magnitudes, and a point at
	// the far end of the exponent range against a zero vertex.
	for _, c := range []float64{1, -1, 0.1, 1e300, -1e-300, math.MaxFloat64, math.SmallestNonzeroFloat64} {
		v := r3.Vec{X: c, Y: -c, Z: c / 3}
		for _, w := range []r3.Vec{
			{X: math.Nextafter(c, 0), Y: -c, Z: v.Z},
			{X: c, Y: -c, Z: math.Nextafter(v.Z, 0)},
		} {
			p := proof.XptOf(w)
			got := meshbool.CoordDistance(p, v)
			requireSameRat(t, referenceCoordDistance(p, v), got, "one ulp")
		}
	}
	far := proof.XptOf(r3.Vec{X: math.MaxFloat64, Y: math.SmallestNonzeroFloat64, Z: -math.MaxFloat64})
	requireSameRat(t, referenceCoordDistance(far, r3.Vec{}), meshbool.CoordDistance(far, r3.Vec{}), "far")
}

func TestCoordDistanceRejectsNonFiniteVertex(t *testing.T) {
	p := proof.XptOf(r3.Vec{X: 1, Y: 2, Z: 3})
	for _, bad := range []float64{math.NaN(), math.Inf(1), math.Inf(-1)} {
		for _, v := range []r3.Vec{{X: bad}, {Y: bad}, {Z: bad}, {X: 1, Y: 2, Z: bad}} {
			requireSamePanic(t, p, v)
		}
	}
}

func TestCoordDistanceNonPositiveWeightTakesFullPath(t *testing.T) {
	// W <= 0 is outside the Xpt contract. The shortcut must not accept it:
	// a zero weight with a zero numerator would otherwise read as a match.
	v := r3.Vec{X: 1, Y: 2, Z: 3}
	zeroW := proof.Xpt{X: big.NewInt(0), Y: big.NewInt(0), Z: big.NewInt(0), W: big.NewInt(0)}
	requireSamePanic(t, zeroW, v)
	neg := proof.Xpt{X: big.NewInt(-1), Y: big.NewInt(-2), Z: big.NewInt(-3), W: big.NewInt(-1)}
	requireSameRat(t, referenceCoordDistance(neg, v), meshbool.CoordDistance(neg, v), "negative weight")
	negZero := proof.Xpt{X: big.NewInt(0), Y: big.NewInt(0), Z: big.NewInt(0), W: big.NewInt(-5)}
	requireSameRat(t, referenceCoordDistance(negZero, r3.Vec{}), meshbool.CoordDistance(negZero, r3.Vec{}), "negative weight origin")
}

func TestCoordDistanceDoesNotMutateInputs(t *testing.T) {
	v := r3.Vec{X: 0.5, Y: -3, Z: 1e-200}
	for _, p := range []proof.Xpt{proof.XptOf(v), proof.XptOf(r3.Vec{X: 1, Y: 1, Z: 1})} {
		fields := [4]*big.Int{p.X, p.Y, p.Z, p.W}
		var before [4]string
		for i, n := range fields {
			before[i] = n.Text(16)
		}
		got := meshbool.CoordDistance(p, v)
		got.SetInt64(-9)
		for i, n := range fields {
			require.Equal(t, before[i], n.Text(16), "field %d", i)
		}
		// A returned zero is a fresh value each call.
		a, b := meshbool.CoordDistance(proof.XptOf(v), v), meshbool.CoordDistance(proof.XptOf(v), v)
		a.SetInt64(7)
		require.Zero(t, b.Sign())
	}
}

// TestCoordDistancePowerOfTwoSpellingsMatchReference spells one exact vertex
// over every power-of-two weight around the word boundaries, then perturbs one
// numerator bit at a time. The shortcut must say zero for the true spellings
// and keep the reference value for every perturbed one.
func TestCoordDistancePowerOfTwoSpellingsMatchReference(t *testing.T) {
	r := rand.New(rand.NewPCG(0xc0d, 3))
	for range 300 {
		v := coordDistanceVec(r)
		base := proof.XptOf(v)
		for _, j := range []uint{0, 1, 31, 32, 33, 62, 63, 64, 65, 127, 128, 129, 200, 1100} {
			p := proof.Xpt{
				X: new(big.Int).Lsh(base.X, j), Y: new(big.Int).Lsh(base.Y, j),
				Z: new(big.Int).Lsh(base.Z, j), W: new(big.Int).Lsh(base.W, j),
			}
			got := meshbool.CoordDistance(p, v)
			require.Zero(t, got.Sign(), "true spelling, shift %d", j)
			requireSameRat(t, referenceCoordDistance(p, v), got, "true spelling")
			for _, field := range []func(proof.Xpt) *big.Int{
				func(q proof.Xpt) *big.Int { return q.X },
				func(q proof.Xpt) *big.Int { return q.Y },
				func(q proof.Xpt) *big.Int { return q.Z },
			} {
				for _, perturb := range []func(*big.Int){
					func(n *big.Int) { n.Add(n, big.NewInt(1)) },
					func(n *big.Int) { n.Sub(n, big.NewInt(1)) },
					func(n *big.Int) { n.Neg(n) },
					func(n *big.Int) { n.Xor(n, new(big.Int).Lsh(big.NewInt(1), uint(n.BitLen()+int(j)%70))) },
					func(n *big.Int) { n.Rsh(n, 1) },
					func(n *big.Int) { n.Lsh(n, 1) },
				} {
					q := proof.Xpt{
						X: new(big.Int).Set(p.X), Y: new(big.Int).Set(p.Y),
						Z: new(big.Int).Set(p.Z), W: new(big.Int).Set(p.W),
					}
					perturb(field(q))
					requireSameRat(t, referenceCoordDistance(q, v), meshbool.CoordDistance(q, v), "perturbed")
				}
			}
		}
	}
}
