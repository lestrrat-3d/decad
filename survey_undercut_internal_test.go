package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is survey_undercut.go's own internal coverage: decidePull's
// equivalence with opposesPull at zero allowance, and wallNormalDecision's
// soundness against an exact-rational ground truth computed independently
// from the walk's own held tangent and the placed frame's own held
// directions.

// TestDecidePullMatchesOpposesPullAtZeroAllowance proves decidePull(mn, mx, 0)
// reduces to exactly opposesPull(mn, mx)'s own answer — pullOpposes where
// opposesPull is true, pullClear everywhere else, never pullUndecided — over
// a table that includes both exact carve-outs (0 and -1) and the float
// readings fu155's own repro produced.
func TestDecidePullMatchesOpposesPullAtZeroAllowance(t *testing.T) {
	pairs := [][2]float64{
		{0, 0},
		{-1, -1},
		{-1, 0},
		{-1e-17, -1e-17},
		{-0.99999999999999989, -0.99999999999999989},
		{-4.6811112914356013e-17, -4.6811112914356013e-17},
		{-0.5, 0.5},
		{-0.5, -0.5},
		{0.5, 0.5},
		{-1, 1},
		{-1, -0.5},
		{-0.9999999999999998, -0.9999999999999998},
		{-1.0000000000000002, -1.0000000000000002},
		{-1e-300, -1e-300},
		{-1e-300, 1e-300},
		{0, 1},
		{-2, -2},
		{-2, -1},
		{-2, 0.5},
		{-0.1, -0.1},
		{-0.1, 0},
		{-0.1, 0.1},
		{-0.3, -0.2},
		{-0.9, -0.1},
		{-0.99, -0.98},
		{-1, -0.9999999999999999},
		{-0.7071067811865476, -0.7071067811865476},
		{-0.7071067811865476, 0.7071067811865476},
		{0, 0.0001},
		{-0.0001, 0},
		{-3, -2},
	}
	for _, p := range pairs {
		mn, mx := p[0], p[1]
		want := pullClear
		if opposesPull(mn, mx) {
			want = pullOpposes
		}
		got := decidePull(mn, mx, 0)
		require.Equalf(t, want, got, "mn=%v mx=%v", mn, mx)
		require.NotEqualf(t, pullUndecided, got, "mn=%v mx=%v", mn, mx)
	}
}

// wallNormalDecisionFixture is one placed prism body ready for
// wallNormalDecision soundness checks: its own placedFrameMap and the
// coalesced side walks of its recorded profile.
type wallNormalDecisionFixture struct {
	name  string
	pp    prismPayload
	m     placedFrameMap
	walks []sideWalk
}

// wallNormalDecisionFixtures builds identity, translated and rotated-by-one-
// radian placements of the same round-cornered rectangle prism (straight AND
// circular walls both), each read back into its placedFrameMap and side
// walks — everything wallNormalDecision needs, and everything this test
// needs to build its own exact-rational ground truth independently.
func wallNormalDecisionFixtures(t *testing.T) []wallNormalDecisionFixture {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 40, 25)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)

	doc := New()
	rectBody, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(10), Dir: Along})
	require.NoError(t, err)
	body, err := rectBody.Fillet(Edges(ParallelTo(r3.NewVec(0, 0, 1))), units.Millimeters(6))
	require.NoError(t, err)

	translate, err := r3.Translation(r3.NewVec(7, -3, 19))
	require.NoError(t, err)
	rotate, err := r3.Rotation(r3.NewVec(1, 1, 1), units.Radians(1))
	require.NoError(t, err)

	placements := []struct {
		name  string
		xform *r3.Transform
	}{
		{"identity", nil},
		{"translation", &translate},
		{"rotation", &rotate},
	}
	var out []wallNormalDecisionFixture
	for _, pl := range placements {
		b := body
		if pl.xform != nil {
			var err error
			b, err = b.PlacedCopy(*pl.xform)
			require.NoError(t, err)
		}
		pp, ok := b.payload.(prismPayload)
		require.True(t, ok)
		m, ok := newPlacedFrameMap(pp)
		require.True(t, ok)
		loops, err := recordLoops(nil, pp.profile)
		require.NoError(t, err)
		var walks []sideWalk
		for _, loop := range loops {
			walks = append(walks, loop...)
		}
		require.NotEmpty(t, walks)
		out = append(out, wallNormalDecisionFixture{name: pl.name, pp: pp, m: m, walks: walks})
	}
	return out
}

// exactWallComponentSquared computes, from the SAME held numbers
// wallNormalDecision itself reads (the walk's own tangent or its held
// circular sweep angles, and the placed frame's held directions), a
// ground-truth answer over big.Rat: whether the wall's exact
// normal-component (against the pull, at the walk's own start for a straight
// walk) is >= 0, <= -1, or strictly between — built independently of
// circularNormalRange/decideRationalComponent so it does not share their own
// bugs. ok is false for a circular walk, where this test instead samples
// float64 endpoints only (see the caller).
func exactWallComponentSquared(w sideWalk, m placedFrameMap, pull r3.Vec) (num, scale2, pull2 *big.Rat, ok bool) {
	if w.isCircular() {
		return nil, nil, nil, false
	}
	pv, okP := ivVec3Of(pull)
	if !okP {
		return nil, nil, nil, false
	}
	tu, tv := floatRat(w.tanInU), floatRat(w.tanInV)
	if tu == nil || tv == nil {
		return nil, nil, nil, false
	}
	du := ivVec3Dot(m.du, pv).lo
	dv := ivVec3Dot(m.dv, pv).lo
	num = new(big.Rat).Sub(new(big.Rat).Mul(tv, du), new(big.Rat).Mul(tu, dv))
	scale2 = ratAdd(ratMul(tu, tu), ratMul(tv, tv))
	pull2 = ivVec3NormSq(pv).lo
	return num, scale2, pull2, true
}

// TestWallNormalDecisionEnclosesExactComponent is the soundness property:
// over a spread of placed prism payloads (identity, a translation, a
// rotation by 1 radian none of whose held direction components are
// "nice"), every side walk and a spread of pull directions, the reader
// never answers pullOpposes when the exact component is provably >= 0 or
// <= -1, and never answers pullClear when the exact component is provably
// strictly inside (-1, 0) — asserted on the computed exact-rational ground
// truth, not on which code path ran.
func TestWallNormalDecisionEnclosesExactComponent(t *testing.T) {
	fixtures := wallNormalDecisionFixtures(t)
	pulls := []r3.Vec{
		r3.NewVec(1, 0, 0),
		r3.NewVec(0, 1, 0),
		r3.NewVec(0, 0, 1),
		r3.NewVec(1, 1, 0),
		r3.NewVec(1, -1, 0.3),
		r3.NewVec(3, 9, 0),
		r3.NewVec(-9, 3, 0),
		r3.NewVec(0.001, 1, 0),
		r3.NewVec(2.5, -1.5, 4.25),
		r3.NewVec(-1, -1, -1),
	}
	checked := 0
	for _, fix := range fixtures {
		for _, w := range fix.walks {
			for _, pull := range pulls {
				verdict, ok := wallNormalDecision(w, fix.m, pull)
				if !ok {
					continue
				}
				num, scale2, pull2, okExact := exactWallComponentSquared(w, fix.m, pull)
				if !okExact {
					requireSoundCircularVerdict(t, fix.name, verdict, fix.pp, w, pull)
					checked++
					continue
				}
				requireSoundVerdict(t, fix.name, verdict, num, scale2, pull2)
				checked++
			}
		}
	}
	require.Positive(t, checked, "the fixtures must exercise at least one walk/pull pair")
}

// requireSoundVerdict asserts wallNormalDecision's verdict against the exact
// rational component num/sqrt(scale2*pull2): pullOpposes only where that
// component is provably in the open interval (-1, 0), and pullClear only
// where it is provably at or above 0 or at or below -1.
func requireSoundVerdict(t *testing.T, fixture string, verdict pullVerdict, num, scale2, pull2 *big.Rat) {
	t.Helper()
	nonNegative := num.Sign() >= 0
	lhs := new(big.Rat).Mul(num, num)
	rhs := new(big.Rat).Mul(scale2, pull2)
	atOrBeyondAntiparallel := num.Sign() <= 0 && lhs.Cmp(rhs) >= 0
	strictlyBetween := !nonNegative && !atOrBeyondAntiparallel
	switch verdict {
	case pullOpposes:
		require.Truef(t, strictlyBetween, "%s: pullOpposes but exact component is not strictly in (-1, 0)", fixture)
	case pullClear:
		require.Falsef(t, strictlyBetween, "%s: pullClear but exact component is strictly in (-1, 0)", fixture)
	}
}

// requireSoundCircularVerdict is the circular-walk counterpart of
// requireSoundVerdict. A circular walk's component varies continuously over
// its window, so there is no single exact rational to compare against;
// instead this densely samples sigma*(du*cosθ + dv*sinθ)/|pull| in float64
// across [th0, th1] — an independent evaluation of the same closed form
// wallNormalDecision encloses, never calling circularNormalRange or any of
// its helpers — and checks the verdict against what that dense sample
// (with a healthy float64 margin around the 0 and -1 boundaries, so an
// ordinary rounding difference between the two evaluations never trips it)
// can support: pullOpposes only where a sampled point reads clearly inside
// (-1, 0), and pullClear only where no sampled point does.
func requireSoundCircularVerdict(t *testing.T, fixture string, verdict pullVerdict, pp prismPayload, w sideWalk, pull r3.Vec) {
	t.Helper()
	unit, ok := pull.Normalize()
	if !ok {
		return
	}
	du := pp.dir(1, 0, 0).Dot(unit)
	dv := pp.dir(0, 1, 0).Dot(unit)
	sigma := 1.0
	if w.th1 < w.th0 {
		sigma = -1
	}
	lo, hi := math.Min(w.th0, w.th1), math.Max(w.th0, w.th1)

	const margin = 1e-9
	sawStrictlyBetween := false
	allClear := true
	const samples = 2000
	for i := 0; i <= samples; i++ {
		th := lo + (hi-lo)*float64(i)/samples
		c := sigma * (du*math.Cos(th) + dv*math.Sin(th))
		if c < -margin && c > -1+margin {
			sawStrictlyBetween = true
		}
		if !(c >= -margin || c <= -1+margin) {
			allClear = false
		}
	}
	switch verdict {
	case pullOpposes:
		require.Truef(t, sawStrictlyBetween, "%s: pullOpposes but no sampled point reads inside (-1, 0)", fixture)
	case pullClear:
		require.Truef(t, allClear, "%s: pullClear but a sampled point reads inside (-1, 0)", fixture)
	}
}
