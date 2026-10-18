package proof_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func xhpBenchTriangle() (a, b, c proof.Xhp) {
	return proof.XhpOf(r3.NewVec(0.1, 0.2, 0.3)),
		proof.XhpOf(r3.NewVec(1.7, 0.35, -0.9)),
		proof.XhpOf(r3.NewVec(-0.55, 2.25, 0.125))
}

// xhpGrid is the 8×8×8 probe grid every xhp differential test sweeps.
var xhpGrid = [8]float64{-2.5, -1, -0.125, 0, 0.1, 0.3, 1, 2.75}

// refPoint is an exact rational 3D point built with math/big.Rat directly —
// the same shape this package's exact kernel carried before this change.
// It is kept entirely apart from production's xpt/xhp kernel, private to this
// test file, so the differential tests below stay a genuine independent
// check rather than becoming circular once the production sign path is
// itself built on xhp.
type refPoint struct{ x, y, z *big.Rat }

func refPointOf(v r3.Vec) refPoint {
	return refPoint{polynomial.MustRatOf(v.X), polynomial.MustRatOf(v.Y), polynomial.MustRatOf(v.Z)}
}

func refSub(a, b refPoint) refPoint {
	return refPoint{new(big.Rat).Sub(a.x, b.x), new(big.Rat).Sub(a.y, b.y), new(big.Rat).Sub(a.z, b.z)}
}

func refCross(a, b refPoint) refPoint {
	return refPoint{
		new(big.Rat).Sub(new(big.Rat).Mul(a.y, b.z), new(big.Rat).Mul(a.z, b.y)),
		new(big.Rat).Sub(new(big.Rat).Mul(a.z, b.x), new(big.Rat).Mul(a.x, b.z)),
		new(big.Rat).Sub(new(big.Rat).Mul(a.x, b.y), new(big.Rat).Mul(a.y, b.x)),
	}
}

func refDot(a, b refPoint) *big.Rat {
	s := new(big.Rat).Mul(a.x, b.x)
	s.Add(s, new(big.Rat).Mul(a.y, b.y))
	return s.Add(s, new(big.Rat).Mul(a.z, b.z))
}

func refOrientSign(a, b, c, d refPoint) int {
	return refDot(refCross(refSub(b, a), refSub(c, a)), refSub(d, a)).Sign()
}

func refLerp(a, b refPoint, t *big.Rat) refPoint {
	d := refSub(b, a)
	return refPoint{
		new(big.Rat).Add(a.x, new(big.Rat).Mul(t, d.x)),
		new(big.Rat).Add(a.y, new(big.Rat).Mul(t, d.y)),
		new(big.Rat).Add(a.z, new(big.Rat).Mul(t, d.z)),
	}
}

// TestXHPAgreesWithTheRationalOrientSign is the direct differential proof
// behind this change (fu163 §1): the homogeneous-integer sign must never
// disagree with an independent math/big.Rat computation of the same
// determinant, because disagreement would mean a topology decision could
// flip. It sweeps 512 float-derived probes and, separately, 512
// refLerp-derived probes at t = 37/91 — the investigation's own check, which
// passed on all 1024.
func TestXHPAgreesWithTheRationalOrientSign(t *testing.T) {
	t.Parallel()
	a, b, c := r3.NewVec(0.1, 0.2, 0.3), r3.NewVec(1.7, 0.35, -0.9), r3.NewVec(-0.55, 2.25, 0.125)
	ra, rb, rc := refPointOf(a), refPointOf(b), refPointOf(c)
	ha, hb, hc := proof.XhpOf(a), proof.XhpOf(b), proof.XhpOf(c)
	tRat := big.NewRat(37, 91)
	tn, td := big.NewInt(37), big.NewInt(91)

	for _, px := range xhpGrid {
		for _, py := range xhpGrid {
			for _, pz := range xhpGrid {
				p := r3.NewVec(px, py, pz)
				rp := refPointOf(p)

				want := refOrientSign(ra, rb, rc, rp)
				got := proof.XhpOrientSign(ha, hb, hc, proof.XhpOf(p))
				require.Equalf(t, want, got, `direct probe (%v,%v,%v): the homogeneous and independent rational signs must agree`, px, py, pz)

				wantLerp := refOrientSign(ra, rb, rc, refLerp(ra, rp, tRat))
				gotLerp := proof.XhpOrientSign(ha, hb, hc, proof.XhpLerp(ha, proof.XhpOf(p), tn, td))
				require.Equalf(t, wantLerp, gotLerp, `lerped probe (%v,%v,%v): the homogeneous and independent rational signs must agree`, px, py, pz)
			}
		}
	}
}

// TestXHPLerpDenotesTheSameCoordinate pins the positivity invariant the sign
// argument rests on: for every lerped probe proof.XhpLerp produces, the exact
// rational it denotes (x/w, y/w, z/w) equals the coordinate an independent
// math/big.Rat lerp computes for the identical t, and w is strictly positive.
func TestXHPLerpDenotesTheSameCoordinate(t *testing.T) {
	t.Parallel()
	a := r3.NewVec(0.1, 0.2, 0.3)
	ra, ha := refPointOf(a), proof.XhpOf(a)
	tRat := big.NewRat(37, 91)
	tn, td := big.NewInt(37), big.NewInt(91)

	for _, px := range xhpGrid {
		for _, py := range xhpGrid {
			for _, pz := range xhpGrid {
				p := r3.NewVec(px, py, pz)
				want := refLerp(ra, refPointOf(p), tRat)
				got := proof.XhpLerp(ha, proof.XhpOf(p), tn, td)
				require.Positive(t, got.W.Sign(), `the homogeneous denominator must stay positive`)
				gx, gy, gz := proof.XhpRat(got)
				require.Zerof(t, gx.Cmp(want.x), `x at probe (%v,%v,%v)`, px, py, pz)
				require.Zerof(t, gy.Cmp(want.y), `y at probe (%v,%v,%v)`, px, py, pz)
				require.Zerof(t, gz.Cmp(want.z), `z at probe (%v,%v,%v)`, px, py, pz)
				wx, _ := want.x.Float64()
				wy, _ := want.y.Float64()
				wz, _ := want.z.Float64()
				require.Equalf(t, r3.Vec{X: wx, Y: wy, Z: wz}, proof.XhpVec(got), `float rounding at probe (%v,%v,%v)`, px, py, pz)
			}
		}
	}
}

// TestXHPArithmeticMatchesRationalAndPreservesInputs checks arithmetic results
// against an independent rational computation and verifies unchanged inputs.
func TestXHPArithmeticMatchesRationalAndPreservesInputs(t *testing.T) {
	t.Parallel()
	for _, v := range []r3.Vec{
		r3.NewVec(math.SmallestNonzeroFloat64, -math.MaxFloat64, 0),
		r3.NewVec(-math.SmallestNonzeroFloat64, math.Ldexp(1, -1022), math.MaxFloat64),
	} {
		assertXHPRatEqual(t, proof.XhpOf(v), refPointOf(v))
	}
	a := proof.XhpOf(r3.NewVec(0.1, -2.25, 3.5))
	b := proof.XhpOf(r3.NewVec(-1.75, 0.375, 4.125))
	aBefore, bBefore := xhpKeyOf(a), xhpKeyOf(b)
	ra, rb := refPointOf(r3.NewVec(0.1, -2.25, 3.5)), refPointOf(r3.NewVec(-1.75, 0.375, 4.125))

	gotSub := proof.XhpSub(a, b)
	wantSub := refSub(ra, rb)
	assertXHPRatEqual(t, gotSub, wantSub)
	sharedA, sharedB := r3.NewVec(1.5, 2.25, 0), r3.NewVec(-0.5, 1.25, 0)
	assertXHPRatEqual(t, proof.XhpSub(proof.XhpOf(sharedA), proof.XhpOf(sharedB)),
		refSub(refPointOf(sharedA), refPointOf(sharedB)))
	gotCross := proof.XhpCross(a, b)
	wantCross := refCross(ra, rb)
	assertXHPRatEqual(t, gotCross, wantCross)
	gotDot := new(big.Rat).SetFrac(proof.XhpDotNum(a, b), new(big.Int).Mul(a.W, b.W))
	require.Zero(t, gotDot.Cmp(refDot(ra, rb)))

	require.Equal(t, aBefore, xhpKeyOf(a))
	require.Equal(t, bBefore, xhpKeyOf(b))
}

func assertXHPRatEqual(t *testing.T, got proof.Xhp, want refPoint) {
	t.Helper()
	x, y, z := proof.XhpRat(got)
	require.Zero(t, x.Cmp(want.x))
	require.Zero(t, y.Cmp(want.y))
	require.Zero(t, z.Cmp(want.z))
}

// TestOrientRatAgreesWithOrientSignExact checks the split orientVal was cut
// into: proof.OrientRat's materialised value and proof.OrientSignExact's plain integer
// sign must agree over the same probes, sign consumer and value consumer
// alike.
func TestOrientRatAgreesWithOrientSignExact(t *testing.T) {
	t.Parallel()
	a, b, c := r3.NewVec(0.1, 0.2, 0.3), r3.NewVec(1.7, 0.35, -0.9), r3.NewVec(-0.55, 2.25, 0.125)
	xa, xb, xc := proof.XptOf(a), proof.XptOf(b), proof.XptOf(c)
	for _, px := range xhpGrid {
		for _, py := range xhpGrid {
			for _, pz := range xhpGrid {
				p := proof.XptOf(r3.NewVec(px, py, pz))
				require.Equalf(t, proof.OrientSignExact(xa, xb, xc, p), proof.OrientRat(xa, xb, xc, p).Sign(),
					`probe (%v,%v,%v): the sign and the materialised value must agree`, px, py, pz)
			}
		}
	}
}

// xhpKeyOf is a decimal rendering of the four canonical integers. It stays
// local so the differential tests compare canonical tuples independently of
// production's packed key encoding.
func xhpKeyOf(p proof.Xhp) string {
	return p.X.String() + `|` + p.Y.String() + `|` + p.Z.String() + `|` + p.W.String()
}

// TestXHPCanonIsAUniqueIdentity is what stands between this change and a
// silently cracked mesh: a homogeneous point has infinitely many spellings,
// and welding (boolean_mesh.go:998) is by exact identity, so two spellings of
// one point MUST canonicalise to one key. It takes a depth-2 lerped point,
// scales all four integers by 6 — a second, non-reduced spelling of the same
// coordinate — and asserts both canonicalise to the same key, the same
// four-tuple, and the same rational coordinate.
func TestXHPCanonIsAUniqueIdentity(t *testing.T) {
	t.Parallel()
	a, b, c := xhpBenchTriangle()
	tn, td := big.NewInt(37), big.NewInt(91)
	depth1 := proof.XhpLerp(a, b, tn, td)
	depth2 := proof.XhpLerp(depth1, c, tn, td)

	six := big.NewInt(6)
	scaled := proof.Xhp{
		X: new(big.Int).Mul(depth2.X, six),
		Y: new(big.Int).Mul(depth2.Y, six),
		Z: new(big.Int).Mul(depth2.Z, six),
		W: new(big.Int).Mul(depth2.W, six),
	}

	canonA, canonB := proof.XhpCanon(depth2), proof.XhpCanon(scaled)
	require.Equal(t, xhpKeyOf(canonA), xhpKeyOf(canonB),
		`two spellings of one point must canonicalise to the same key`)
	require.Equal(t, proof.Xpt(depth2).Key(), proof.Xpt(scaled).Key(),
		`the packed production key must canonicalise both spellings identically`)
	require.Equal(t, canonA.X.String(), canonB.X.String())
	require.Equal(t, canonA.Y.String(), canonB.Y.String())
	require.Equal(t, canonA.Z.String(), canonB.Z.String())
	require.Equal(t, canonA.W.String(), canonB.W.String())

	rx, ry, rz := proof.XhpRat(depth2)
	crx, cry, crz := proof.XhpRat(canonA)
	require.Zero(t, rx.Cmp(crx), `canonicalising must not change the denoted x`)
	require.Zero(t, ry.Cmp(cry), `canonicalising must not change the denoted y`)
	require.Zero(t, rz.Cmp(crz), `canonicalising must not change the denoted z`)
}

func TestXPTKeyDistinguishesCoordinates(t *testing.T) {
	t.Parallel()
	points := []proof.Xpt{
		proof.XptOf(r3.NewVec(0, 0, 0)),
		proof.XptOf(r3.NewVec(1, 23, 4)),
		proof.XptOf(r3.NewVec(12, 3, 4)),
		proof.XptOf(r3.NewVec(-1, 23, 4)),
		proof.XptOf(r3.NewVec(1, -23, 4)),
		proof.XptOf(r3.NewVec(1, 23, -4)),
		proof.XptOf(r3.NewVec(0.5, 23, 4)),
	}
	seen := make(map[string]struct{}, len(points))
	for _, p := range points {
		key := p.Key()
		require.NotContains(t, seen, key)
		seen[key] = struct{}{}
	}
}

// BenchmarkOrientSignExact measures one exact orient sign over the
// homogeneous-integer representation on float-derived points (fu163 §9's
// cost guard: the math/big.Rat baseline this replaces was 14.6us/206
// allocs/op).
func BenchmarkOrientSignExact(b *testing.B) {
	ta, tb, tc := xhpBenchTriangle()
	p := proof.XhpOf(r3.NewVec(0.3, -1, 2.75))
	b.ResetTimer()
	for b.Loop() {
		proof.XhpOrientSign(ta, tb, tc, p)
	}
}

// BenchmarkOrientSignExactLerped is BenchmarkOrientSignExact with an
// proof.XhpLerp-derived probe in place of a float-derived one (baseline:
// 15.0us/199 allocs/op — indistinguishable from the dyadic case, which is
// what disproves the "non-dyadic denominators" half of the original claim).
func BenchmarkOrientSignExactLerped(b *testing.B) {
	ta, tb, tc := xhpBenchTriangle()
	tn, td := big.NewInt(37), big.NewInt(91)
	p := proof.XhpLerp(tb, tc, tn, td)
	b.ResetTimer()
	for b.Loop() {
		proof.XhpOrientSign(ta, tb, tc, p)
	}
}

// BenchmarkPlaneCrossingChain measures the meshbool.PlaneCrossings → predicate path:
// two lerps feeding one orient sign (baseline: 26.1us/348 allocs/op).
func BenchmarkPlaneCrossingChain(b *testing.B) {
	ta, tb, tc := xhpBenchTriangle()
	tn, td := big.NewInt(37), big.NewInt(91)
	b.ResetTimer()
	for b.Loop() {
		p1 := proof.XhpStripTwosOwned(proof.XhpLerp(ta, tb, tn, td))
		p2 := proof.XhpStripTwosOwned(proof.XhpLerp(tb, tc, tn, td))
		proof.XhpOrientSign(ta, p1, p2, tc)
	}
}

// BenchmarkLerpDepth3Orient is the growth-control benchmark: three nested
// lerps, each stripped of its common power of two before feeding the next,
// then one orient sign (baseline: 31.4us/432 allocs/op; the arrangement to
// avoid — a full proof.XhpCanon on every construction instead of xhpStripTwos —
// measured 23.4us, only 1.3x, per this file's proof.XhpCanon doc comment).
func BenchmarkLerpDepth3Orient(b *testing.B) {
	ta, tb, tc := xhpBenchTriangle()
	tn, td := big.NewInt(37), big.NewInt(91)
	b.ResetTimer()
	for b.Loop() {
		p := proof.XhpStripTwosOwned(proof.XhpLerp(ta, tb, tn, td))
		p = proof.XhpStripTwosOwned(proof.XhpLerp(p, tc, tn, td))
		p = proof.XhpStripTwosOwned(proof.XhpLerp(p, ta, tn, td))
		proof.XhpOrientSign(ta, tb, tc, p)
	}
}

// BenchmarkExactVertexKey measures the canonicalise-and-key cost paid once
// per welded vertex, on a depth-2 lerped point (baseline: math/big.Rat's
// RatString with no canonicalisation step at all, 1.5us/20 allocs/op — the
// canonical key legitimately costs more per vertex than the old spelling
// did, because welding correctness, not speed, is what the canonical form
// buys).
func BenchmarkExactVertexKey(b *testing.B) {
	ta, tb, tc := xhpBenchTriangle()
	tn, td := big.NewInt(37), big.NewInt(91)
	p1 := proof.XhpStripTwosOwned(proof.XhpLerp(ta, tb, tn, td))
	p2 := proof.XhpStripTwosOwned(proof.XhpLerp(p1, tc, tn, td))
	b.ResetTimer()
	for b.Loop() {
		_ = proof.Xpt(p2).Key()
	}
}
