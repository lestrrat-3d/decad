package decad

import (
	"fmt"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/coil"
	"github.com/lestrrat-3d/decad/internal/coilshell"
	"github.com/lestrrat-3d/decad/internal/diameter"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures assert docs/helix-design.md §13's PR 1 rows against the
// production path. Bound legs shown to fail by deleting them and watching
// the named assertion go red, then restoring them:
//
//   - the π enclosure (coil.Theta and SegmentArea's k read a point 2π):
//     TestCoilSquareSpring's "enclosures hold the exact values" went red;
//   - the asinh term (SegmentArea's second term dropped): the annular wall
//     assertions in TestCoilSquareSpring went red;
//   - the δ widening (coilBounds without ±δ): TestCoilBoundsEncloseTheSolid
//     went red at 1.3 turns, where the quarter-turn extreme falls between
//     stations.

// coilAxisV is the sketch's own V axis.
var coilAxisV = SketchLine{Start: Point2{U: 0, V: 0}, End: Point2{U: 0, V: 1}}

// coilLoopsSketch fixes every loop's points on the XY plane, closes each
// loop with lines, and returns the profile with the most holes.
func coilLoopsSketch(t *testing.T, loops ...[][2]float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	for _, loop := range loops {
		points := make([]*sketch.Point, len(loop))
		for i, p := range loop {
			points[i] = s.CreatePoint(p[0], p[1])
			s.Fix(points[i])
		}
		for i := range points {
			s.CreateLine(points[i], points[(i+1)%len(points)])
		}
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var best *sketch.Profile
	for _, p := range s.Profiles() {
		if best == nil || len(p.Holes) > len(best.Holes) {
			best = p
		}
	}
	require.NotNil(t, best)
	return s, best
}

// coilSquare is the §13 square section: ρ ∈ [2, 3], ζ ∈ [0, 1] about V.
func coilSquare(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	return coilLoopsSketch(t, [][2]float64{{2, 0}, {3, 0}, {3, 1}, {2, 1}})
}

// bigPi is π to 75 digits, the in-tree enclosure's lower end.
func bigPi() *big.Float {
	return new(big.Float).SetPrec(512).SetRat(proofbound.PiLower)
}

func bigOf(x float64) *big.Float { return new(big.Float).SetPrec(512).SetFloat64(x) }

// requireEnclosesBig asserts a published reading's own interval holds the
// exact value, computed in 512-bit floating point.
func requireEnclosesBig(t *testing.T, value, bound float64, exact *big.Float, msg string) {
	t.Helper()
	diff := new(big.Float).SetPrec(512).Sub(bigOf(value), exact)
	diff.Abs(diff)
	slack := new(big.Float).SetPrec(512).SetFloat64(bound)
	// 1e-60 relative charges the 75-digit π the reference itself reads.
	tiny := new(big.Float).SetPrec(512).Mul(new(big.Float).Abs(exact), big.NewFloat(1e-60))
	slack.Add(slack, tiny)
	require.LessOrEqual(t, diff.Cmp(slack), 0, "%s: |%v − exact| exceeds bound %v", msg, value, bound)
}

// requireIntervalHolds asserts a rational enclosure contains an exact value.
func requireIntervalHolds(t *testing.T, iv coil.Iv, exact *big.Float, msg string) {
	t.Helper()
	tiny := new(big.Float).SetPrec(512).Mul(new(big.Float).Abs(exact), big.NewFloat(1e-60))
	lo := new(big.Float).SetPrec(512).SetRat(iv.Lo)
	hi := new(big.Float).SetPrec(512).SetRat(iv.Hi)
	require.LessOrEqual(t, lo.Cmp(new(big.Float).SetPrec(512).Add(exact, tiny)), 0, "%s: enclosure lower end above the exact value", msg)
	require.GreaterOrEqual(t, hi.Cmp(new(big.Float).SetPrec(512).Sub(exact, tiny)), 0, "%s: enclosure upper end below the exact value", msg)
}

// annulusIntegral is 2π·turns·∫_a^b sqrt(ρ² + k²) dρ with k = pitch/2π, by
// composite Simpson in 512-bit floating point, an independent check on the
// closed form. Its second result bounds Simpson's own error,
// 2π·turns·(b − a)·h⁴·max|f|/180, with f = 3k²(4ρ² − k²)/(ρ² + k²)^(7/2)
// at most 12k²/ρ⁵, doubled for the float evaluation of the bound itself.
func annulusIntegral(a, b, pitch, turns float64) (*big.Float, float64) {
	const n = 4000
	prec := uint(512)
	pi := bigPi()
	twoPi := new(big.Float).SetPrec(prec).Mul(pi, big.NewFloat(2))
	k := new(big.Float).SetPrec(prec).Quo(bigOf(pitch), twoPi)
	k2 := new(big.Float).SetPrec(prec).Mul(k, k)
	h := new(big.Float).SetPrec(prec).Quo(new(big.Float).SetPrec(prec).Sub(bigOf(b), bigOf(a)), big.NewFloat(n))
	f := func(i int) *big.Float {
		x := new(big.Float).SetPrec(prec).Mul(h, big.NewFloat(float64(i)))
		x.Add(x, bigOf(a))
		x.Mul(x, x)
		x.Add(x, k2)
		return x.Sqrt(x)
	}
	sum := new(big.Float).SetPrec(prec).Add(f(0), f(n))
	for i := 1; i < n; i++ {
		w := big.NewFloat(4)
		if i%2 == 0 {
			w = big.NewFloat(2)
		}
		sum.Add(sum, new(big.Float).SetPrec(prec).Mul(w, f(i)))
	}
	sum.Mul(sum, h)
	sum.Quo(sum, big.NewFloat(3))
	sum.Mul(sum, twoPi)
	sum.Mul(sum, bigOf(turns))
	kf := pitch / (2 * math.Pi)
	hf := (b - a) / n
	quadErr := 2 * 2 * math.Pi * turns * (b - a) * math.Pow(hf, 4) * 12 * kf * kf / (180 * math.Pow(a, 5))
	return sum, quadErr
}

func coilFace(t *testing.T, b *Body, role string) *Face {
	t.Helper()
	for _, f := range b.Faces() {
		for _, o := range f.Origins() {
			if o.Role == role {
				return f
			}
		}
	}
	require.Failf(t, "no face", "role %s", role)
	return nil
}

func TestCoilSquareSpring(t *testing.T) {
	s, p := coilSquare(t)
	doc := New()
	b, err := doc.Coil(t.Context(), s, p, coilAxisV, units.Millimeters(1.5), units.Scalar(2))
	require.NoError(t, err)
	pi := bigPi()
	mulPi := func(x float64) *big.Float { return new(big.Float).SetPrec(512).Mul(pi, bigOf(x)) }

	t.Run("volume encloses 10π", func(t *testing.T) {
		vol, err := b.Volume()
		require.NoError(t, err)
		require.Equal(t, Approximate, vol.Exactness)
		require.Less(t, vol.Bound.Base(), 1e-12)
		requireEnclosesBig(t, vol.Value.Base(), vol.Bound.Base(), mulPi(10), "volume")
	})
	t.Run("centroid is exact on the axis", func(t *testing.T) {
		c, err := b.Centroid()
		require.NoError(t, err)
		require.Equal(t, Exact, c.Exactness)
		require.Equal(t, r3.NewVec(0, 2, 0), c.Value)
	})
	t.Run("walls read their closed forms", func(t *testing.T) {
		annulus, quadErr := annulusIntegral(2, 3, 1.5, 2)
		// Segment j of the outer loop is side(0,j): two annuli at ζ = 0 and
		// ζ = 1, and the bands at ρ = 3 and ρ = 2.
		for j, want := range []struct {
			exact *big.Float
			err   float64
		}{
			{annulus, quadErr},
			{mulPi(12), 0},
			{annulus, quadErr},
			{mulPi(8), 0},
		} {
			role := fmt.Sprintf("side(0,%d)", j)
			a, err := coilFace(t, b, role).Area()
			require.NoError(t, err)
			requireEnclosesBig(t, a.Value.Base(), a.Bound.Base()+want.err, want.exact, role)
			require.Equal(t, KindFaceted, coilFace(t, b, role).Surface().Kind())
		}
		area, err := b.Area()
		require.NoError(t, err)
		total := new(big.Float).SetPrec(512).Add(mulPi(20), new(big.Float).SetPrec(512).Mul(annulus, big.NewFloat(2)))
		total.Add(total, big.NewFloat(2))
		requireEnclosesBig(t, area.Value.Base(), area.Bound.Base()+2*quadErr, total, "body area")
	})
	t.Run("enclosures hold the exact values", func(t *testing.T) {
		cp := b.payload.(coilPayload)
		rec, err := coilRecordOfPayload(cp)
		require.NoError(t, err)
		m := coil.RegionMoments(rec.Rho, rec.Zeta, rec.LoopIdx, big.NewRat(1, 1), rec.Axis.Side)
		requireIntervalHolds(t, coil.Volume(m, rec.Turns), mulPi(10), "volume enclosure")
		band, ok := coil.SegmentArea(rec.Rho[1], rec.Zeta[1], rec.Rho[2], rec.Zeta[2], rec.Pitch, rec.Turns)
		require.True(t, ok)
		requireIntervalHolds(t, band, mulPi(12), "band enclosure")
		ring, ok := coil.SegmentArea(rec.Rho[0], rec.Zeta[0], rec.Rho[1], rec.Zeta[1], rec.Pitch, rec.Turns)
		require.True(t, ok)
		annulus, quadErr := annulusIntegral(2, 3, 1.5, 2)
		lo := new(big.Float).SetPrec(512).SetRat(ring.Lo)
		hi := new(big.Float).SetPrec(512).SetRat(ring.Hi)
		require.LessOrEqual(t, lo.Cmp(new(big.Float).SetPrec(512).Add(annulus, bigOf(quadErr))), 0, "annulus enclosure lower end")
		require.GreaterOrEqual(t, hi.Cmp(new(big.Float).SetPrec(512).Sub(annulus, bigOf(quadErr))), 0, "annulus enclosure upper end")
		// The enclosure itself is far narrower than the quadrature's error.
		width := new(big.Rat).Sub(ring.Hi, ring.Lo)
		require.Negative(t, width.Cmp(big.NewRat(1, 1<<62)))
	})
	t.Run("topology", func(t *testing.T) {
		require.Len(t, b.Faces(), 6)
		require.Len(t, b.Edges(), 12)
		require.Len(t, b.Vertices(), 8)
		require.Len(t, b.Lumps(), 1)
		require.Len(t, b.Shells(), 1)
		for _, e := range b.Edges() {
			require.Len(t, e.Faces(), 2)
		}
		helices := 0
		for _, e := range b.Edges() {
			if _, ok := e.Curve().(FacetedCurve); ok {
				helices++
				// The outer square turns left at every corner.
				require.True(t, e.IsConvex())
			}
		}
		require.Equal(t, 4, helices)
	})
	t.Run("held table bounds", func(t *testing.T) {
		cp := b.payload.(coilPayload)
		require.Len(t, cp.verts, 4*(512+1))
		maxBeta := 0.0
		for _, beta := range cp.vertexBound {
			maxBeta = math.Max(maxBeta, beta)
		}
		require.Equal(t, cp.delta, maxBeta)
		require.Positive(t, cp.delta)
		box, err := b.Bounds()
		require.NoError(t, err)
		require.Equal(t, Approximate, box.Exactness)
	})
	t.Run("verify is sound", func(t *testing.T) {
		rep, err := doc.Verify(t.Context())
		require.NoError(t, err)
		require.Equal(t, Sound, rep.Status)
		require.Equal(t, ValidityValid, rep.Bodies[0].Validity.Outcome)
		cp := b.payload.(coilPayload)
		held, ok := diameter.Points(cp.verts)
		require.True(t, ok)
		want, ok := diameter.LowerForDisplacement(held, cp.delta)
		require.True(t, ok)
		got, ok, err := bodyGateDiameter(t.Context(), b)
		require.NoError(t, err)
		require.True(t, ok)
		require.Equal(t, want, got)
		require.True(t, payloadProvesSimple(t.Context(), b.payload))
	})
}

// coilSamples evaluates Φ at a dense grid of boundary and interior points of
// the square section about V: e_r = +X, e_t = −σZ, n = +Y.
func coilSamples(pitch, turns float64, sigma float64) []r3.Vec {
	k := pitch / (2 * math.Pi)
	theta := 2 * math.Pi * turns
	var out []r3.Vec
	const steps = 4096
	for i := 0; i <= steps; i++ {
		th := theta * float64(i) / steps
		for _, rho := range []float64{2, 2.5, 3} {
			for _, zeta := range []float64{0, 0.5, 1} {
				out = append(out, r3.NewVec(rho*math.Cos(th), zeta+k*th, -sigma*rho*math.Sin(th)))
			}
		}
	}
	// Every exact quarter turn inside the sweep, where the box's extremes sit.
	for q := 0.0; q <= 4*turns; q++ {
		th := q * math.Pi / 2
		out = append(out, r3.NewVec(3*math.Cos(th), 1+k*th, -sigma*3*math.Sin(th)))
	}
	return out
}

func TestCoilBoundsEncloseTheSolid(t *testing.T) {
	for _, turns := range []float64{2, 1.3} {
		s, p := coilSquare(t)
		b, err := New().Coil(t.Context(), s, p, coilAxisV, units.Millimeters(1.5), units.Scalar(turns))
		require.NoError(t, err)
		box, err := b.Bounds()
		require.NoError(t, err)
		lo, hi := r3.NewVec(math.Inf(1), math.Inf(1), math.Inf(1)), r3.NewVec(math.Inf(-1), math.Inf(-1), math.Inf(-1))
		// The samples' own float evaluation is off by a few ulps.
		const slop = 1e-12
		for _, q := range coilSamples(1.5, turns, 1) {
			require.GreaterOrEqual(t, q.X, box.Min.X-slop)
			require.GreaterOrEqual(t, q.Y, box.Min.Y-slop)
			require.GreaterOrEqual(t, q.Z, box.Min.Z-slop)
			require.LessOrEqual(t, q.X, box.Max.X+slop)
			require.LessOrEqual(t, q.Y, box.Max.Y+slop)
			require.LessOrEqual(t, q.Z, box.Max.Z+slop)
			lo = r3.NewVec(math.Min(lo.X, q.X), math.Min(lo.Y, q.Y), math.Min(lo.Z, q.Z))
			hi = r3.NewVec(math.Max(hi.X, q.X), math.Max(hi.Y, q.Y), math.Max(hi.Z, q.Z))
		}
		bound := box.Bound.Base() + slop
		require.LessOrEqual(t, lo.X-box.Min.X, bound)
		require.LessOrEqual(t, lo.Y-box.Min.Y, bound)
		require.LessOrEqual(t, lo.Z-box.Min.Z, bound)
		require.LessOrEqual(t, box.Max.X-hi.X, bound)
		require.LessOrEqual(t, box.Max.Y-hi.Y, bound)
		require.LessOrEqual(t, box.Max.Z-hi.Z, bound)
	}
}

func TestCoilFractionalTurnCentroid(t *testing.T) {
	s, p := coilSquare(t)
	b, err := New().Coil(t.Context(), s, p, coilAxisV, units.Millimeters(1.5), units.Scalar(2.5))
	require.NoError(t, err)
	pi := bigPi()
	vol, err := b.Volume()
	require.NoError(t, err)
	requireEnclosesBig(t, vol.Value.Base(), vol.Bound.Base(), new(big.Float).SetPrec(512).Mul(pi, big.NewFloat(12.5)), "volume")

	// I/Q = (19/3)/(5/2) = 38/15; sin Θ = 0 and cos Θ = −1 exactly at 2.5
	// turns, so the transverse part is (I/Q)·2/Θ along e_t = −Z, and the
	// axial part is M/Q + pitch·turns/2 = 1/2 + 15/8.
	c, err := b.Centroid()
	require.NoError(t, err)
	require.Equal(t, Approximate, c.Exactness)
	require.Zero(t, c.Value.X)
	require.Equal(t, 2.375, c.Value.Y)
	wantZ := new(big.Float).SetPrec(512).Quo(big.NewFloat(-76), new(big.Float).SetPrec(512).Mul(pi, big.NewFloat(75)))
	requireEnclosesBig(t, c.Value.Z, c.Bound.Base(), wantZ, "centroid z")

	cp := b.payload.(coilPayload)
	rec, err := coilRecordOfPayload(cp)
	require.NoError(t, err)
	m := coil.RegionMoments(rec.Rho, rec.Zeta, rec.LoopIdx, big.NewRat(1, 1), rec.Axis.Side)
	r, tr, n, ok := coil.CentroidCoefficients(m, rec.Pitch, rec.Turns, rec.Sigma)
	require.True(t, ok)
	require.Zero(t, r.Lo.Sign())
	require.Zero(t, r.Hi.Sign())
	require.Zero(t, n.Lo.Cmp(big.NewRat(19, 8)))
	require.Zero(t, n.Hi.Cmp(big.NewRat(19, 8)))
	requireIntervalHolds(t, tr, new(big.Float).SetPrec(512).Quo(big.NewFloat(76), new(big.Float).SetPrec(512).Mul(pi, big.NewFloat(75))), "transverse")
}

func TestCoilLeftHandMirrors(t *testing.T) {
	s, p := coilSquare(t)
	doc := New()
	right, err := doc.Coil(t.Context(), s, p, coilAxisV, units.Millimeters(1.5), units.Scalar(2.5))
	require.NoError(t, err)
	left, err := doc.Coil(t.Context(), s, p, coilAxisV, units.Millimeters(1.5), units.Scalar(2.5), WithLeftHand())
	require.NoError(t, err)

	rv, lv := right.payload.(coilPayload).verts, left.payload.(coilPayload).verts
	require.Len(t, lv, len(rv))
	for i := range rv {
		require.Equal(t, r3.NewVec(rv[i].X, rv[i].Y, -rv[i].Z), lv[i], "vertex %d", i)
	}
	rc, err := right.Centroid()
	require.NoError(t, err)
	lc, err := left.Centroid()
	require.NoError(t, err)
	require.Equal(t, r3.NewVec(rc.Value.X, rc.Value.Y, -rc.Value.Z), lc.Value)
	require.Equal(t, rc.Bound, lc.Bound)
	rvol, err := right.Volume()
	require.NoError(t, err)
	lvol, err := left.Volume()
	require.NoError(t, err)
	require.Equal(t, rvol, lvol)
}

func TestCoilTiltedAxisAgrees(t *testing.T) {
	// The axis runs along (3, 4)/5, a direction whose float components carry
	// a rounding bound. The profile is the square ρ ∈ [10, 15], ζ ∈ [0, 5]
	// about it, whose corners ζ·d + ρ·e_r land on integers.
	ts, tp := coilLoopsSketch(t, [][2]float64{{-8, 6}, {-12, 9}, {-9, 13}, {-5, 10}})
	tilted, err := New().Coil(t.Context(), ts, tp, SketchLine{Start: Point2{}, End: Point2{U: 3, V: 4}},
		units.Millimeters(6), units.Scalar(2))
	require.NoError(t, err)
	as, ap := coilLoopsSketch(t, [][2]float64{{10, 0}, {15, 0}, {15, 5}, {10, 5}})
	aligned, err := New().Coil(t.Context(), as, ap, coilAxisV, units.Millimeters(6), units.Scalar(2))
	require.NoError(t, err)

	tv, err := tilted.Volume()
	require.NoError(t, err)
	av, err := aligned.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(tv.Value.Base()-av.Value.Base()), tv.Bound.Base()+av.Bound.Base())
	ta, err := tilted.Area()
	require.NoError(t, err)
	aa, err := aligned.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(ta.Value.Base()-aa.Value.Base()), ta.Bound.Base()+aa.Bound.Base())
	tc, err := tilted.Centroid()
	require.NoError(t, err)
	require.Equal(t, Approximate, tc.Exactness)
	// Both centroids sit on their axis at M/Q + pitch·turns/2 along it.
	ac, err := aligned.Centroid()
	require.NoError(t, err)
	along := ac.Value.Y
	want := r3.NewVec(0.6*along, 0.8*along, 0)
	require.LessOrEqual(t, tc.Value.Sub(want).Len(), tc.Bound.Base()+ac.Bound.Base()+1e-12)
}

func TestCoilHolePassage(t *testing.T) {
	outer := [][2]float64{{2, 0}, {4, 0}, {4, 2}, {2, 2}}
	hole := [][2]float64{{2.5, 0.5}, {3.5, 0.5}, {3.5, 1.5}, {2.5, 1.5}}
	s, p := coilLoopsSketch(t, outer, hole)
	require.Len(t, p.Holes, 1)
	doc := New()
	b, err := doc.Coil(t.Context(), s, p, coilAxisV, units.Millimeters(3), units.Scalar(2))
	require.NoError(t, err)
	require.Len(t, b.Lumps(), 1)
	require.Len(t, b.Shells(), 1)
	require.Len(t, b.Faces(), 10)

	// Θ·(Q_outer − Q_hole) = 4π·(12 − 3) = 36π.
	vol, err := b.Volume()
	require.NoError(t, err)
	exact := new(big.Float).SetPrec(512).Mul(bigPi(), big.NewFloat(36))
	requireEnclosesBig(t, vol.Value.Base(), vol.Bound.Base(), exact, "holed volume")

	so, po := coilLoopsSketch(t, outer)
	whole, err := doc.Coil(t.Context(), so, po, coilAxisV, units.Millimeters(3), units.Scalar(2))
	require.NoError(t, err)
	sh, ph := coilLoopsSketch(t, hole)
	core, err := doc.Coil(t.Context(), sh, ph, coilAxisV, units.Millimeters(3), units.Scalar(2))
	require.NoError(t, err)
	wv, err := whole.Volume()
	require.NoError(t, err)
	cv, err := core.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(vol.Value.Base()-(wv.Value.Base()-cv.Value.Base())),
		vol.Bound.Base()+wv.Bound.Base()+cv.Bound.Base()+2*proofbound.UlpOf(wv.Value.Base()))

	rep, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, ValidityValid, rep.Bodies[0].Validity.Outcome)
}

func TestCoilRefusals(t *testing.T) {
	type call struct {
		name     string
		run      func(doc *Document, s *sketch.Sketch, p *sketch.Profile) error
		sentinel error
	}
	pitch, turns := units.Millimeters(1.5), units.Scalar(2)
	foreign := func(doc *Document, s *sketch.Sketch, _ *sketch.Profile) error {
		_, other := coilSquare(t)
		_, err := doc.Coil(t.Context(), s, other, coilAxisV, pitch, turns)
		return err
	}
	calls := []call{
		{"nil context", func(doc *Document, s *sketch.Sketch, p *sketch.Profile) error {
			_, err := doc.Coil(nil, s, p, coilAxisV, pitch, turns) //nolint:staticcheck // CS1 refuses a nil context.
			return err
		}, ErrDegenerate},
		{"nil sketch", func(doc *Document, _ *sketch.Sketch, p *sketch.Profile) error {
			_, err := doc.Coil(t.Context(), nil, p, coilAxisV, pitch, turns)
			return err
		}, ErrDegenerate},
		{"nil profile", func(doc *Document, s *sketch.Sketch, _ *sketch.Profile) error {
			_, err := doc.Coil(t.Context(), s, nil, coilAxisV, pitch, turns)
			return err
		}, ErrDegenerate},
		{"nil axis", func(doc *Document, s *sketch.Sketch, p *sketch.Profile) error {
			_, err := doc.Coil(t.Context(), s, p, nil, pitch, turns)
			return err
		}, ErrDegenerate},
		{"nil option", func(doc *Document, s *sketch.Sketch, p *sketch.Profile) error {
			_, err := doc.Coil(t.Context(), s, p, coilAxisV, pitch, turns, nil)
			return err
		}, ErrDegenerate},
		{"repeated hand", func(doc *Document, s *sketch.Sketch, p *sketch.Profile) error {
			_, err := doc.Coil(t.Context(), s, p, coilAxisV, pitch, turns, WithLeftHand(), WithLeftHand())
			return err
		}, ErrDegenerate},
		{"foreign profile", foreign, ErrForeignProfile},
		{"axis out of plane", func(doc *Document, s *sketch.Sketch, p *sketch.Profile) error {
			_, err := doc.Coil(t.Context(), s, p, ConstructionAxis{Dir: r3.NewVec(0, 0, 1)}, pitch, turns)
			return err
		}, ErrDegenerate},
		{"pitch kind", func(doc *Document, s *sketch.Sketch, p *sketch.Profile) error {
			_, err := doc.Coil(t.Context(), s, p, coilAxisV, units.Scalar(1.5), turns)
			return err
		}, ErrUnitKind},
		{"turns kind", func(doc *Document, s *sketch.Sketch, p *sketch.Profile) error {
			_, err := doc.Coil(t.Context(), s, p, coilAxisV, pitch, units.Millimeters(2))
			return err
		}, ErrUnitKind},
		{"pitch not finite", func(doc *Document, s *sketch.Sketch, p *sketch.Profile) error {
			_, err := doc.Coil(t.Context(), s, p, coilAxisV, units.Millimeters(math.Inf(1)), turns)
			return err
		}, ErrNotFinite},
		{"zero pitch", func(doc *Document, s *sketch.Sketch, p *sketch.Profile) error {
			_, err := doc.Coil(t.Context(), s, p, coilAxisV, units.Millimeters(0), turns)
			return err
		}, ErrDegenerate},
		{"negative turns", func(doc *Document, s *sketch.Sketch, p *sketch.Profile) error {
			_, err := doc.Coil(t.Context(), s, p, coilAxisV, pitch, units.Scalar(-1))
			return err
		}, ErrDegenerate},
		{"past the station ceiling", func(doc *Document, s *sketch.Sketch, p *sketch.Profile) error {
			_, err := doc.Coil(t.Context(), s, p, coilAxisV, pitch, units.Scalar(129))
			return err
		}, ErrUnsupported},
	}
	for _, c := range calls {
		t.Run(c.name, func(t *testing.T) {
			s, p := coilSquare(t)
			doc := New()
			before := doc.nextProducer
			err := c.run(doc, s, p)
			require.ErrorIs(t, err, c.sentinel)
			require.Empty(t, doc.Bodies())
			require.Equal(t, before, doc.nextProducer)
		})
	}

	refuse := func(t *testing.T, sentinel error, says string, s *sketch.Sketch, p *sketch.Profile, axis Axis, pitch, turns units.Value) {
		t.Helper()
		doc := New()
		before := doc.nextProducer
		_, err := doc.Coil(t.Context(), s, p, axis, pitch, turns)
		require.ErrorIs(t, err, sentinel)
		require.ErrorContains(t, err, says)
		require.Empty(t, doc.Bodies())
		require.Equal(t, before, doc.nextProducer)
	}
	t.Run("CS5 touching the axis", func(t *testing.T) {
		s, p := coilLoopsSketch(t, [][2]float64{{0, 0}, {1, 0}, {1, 1}, {0, 1}})
		refuse(t, ErrDegenerate, "touches its axis", s, p, coilAxisV, units.Millimeters(1.5), units.Scalar(2))
	})
	t.Run("CS5 undecided on a tilted axis", func(t *testing.T) {
		// (3, 4) lies on the axis through (0, 0) along (3, 4); the axis
		// direction's own rounding leaves that vertex's side undecided.
		s, p := coilLoopsSketch(t, [][2]float64{{3, 4}, {-1, 7}, {-4, 3}})
		refuse(t, ErrUnsupported, "does not prove it off the axis", s, p, SketchLine{Start: Point2{}, End: Point2{U: 3, V: 4}}, units.Millimeters(9), units.Scalar(0.5))
	})
	t.Run("CS6 at one pitch", func(t *testing.T) {
		s, p := coilLoopsSketch(t, [][2]float64{{2, 0}, {3, 0}, {3, 1.5}, {2, 1.5}})
		refuse(t, ErrUnsupported, "at or past the 1.5 mm pitch", s, p, coilAxisV, units.Millimeters(1.5), units.Scalar(1))
		doc := New()
		b, err := doc.Coil(t.Context(), s, p, coilAxisV, units.Millimeters(1.5), units.Scalar(0.75))
		require.NoError(t, err)
		require.Len(t, doc.Bodies(), 1)
		vol, err := b.Volume()
		require.NoError(t, err)
		// Θ·Q = 1.5π·(2.5·1.5).
		exact := new(big.Float).SetPrec(512).Mul(bigPi(), big.NewFloat(1.5*2.5*1.5))
		requireEnclosesBig(t, vol.Value.Base(), vol.Bound.Base(), exact, "three-quarter turn")
	})
	t.Run("CS7 a free-form segment", func(t *testing.T) {
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		var control []*sketch.Point
		for _, c := range [][2]float64{{4, -1}, {6, -1}, {6, 1}, {4, 1}} {
			pt := s.CreatePoint(c[0], c[1])
			s.Fix(pt)
			control = append(control, pt)
		}
		_, err = s.CreateClosedSpline(control...)
		require.NoError(t, err)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		refuse(t, ErrUnsupported, "line, arc and circle profile segments only", s, s.Profiles()[0], coilAxisV, units.Millimeters(3), units.Scalar(2))
	})
	t.Run("CS8 past the facet ceiling", func(t *testing.T) {
		s, p := vertexBoundPolygonSketchAt(t, 24, 1, 10)
		refuse(t, ErrUnsupported, "triangle ceiling", s, p, coilAxisV, units.Millimeters(3), units.Scalar(100))
	})
}

// vertexBoundPolygonSketchAt fixes an n-gon of circumradius r about (cx, 0).
func vertexBoundPolygonSketchAt(t *testing.T, n int, r, cx float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	poly, err := s.CreatePolygon(cx, 0, n, r)
	require.NoError(t, err)
	s.Fix(poly.Center)
	for _, v := range poly.Vertices {
		s.Fix(v)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	return s, s.Profiles()[0]
}

func TestCoilRepeatAndPlacement(t *testing.T) {
	s, p := coilSquare(t)
	doc := New()
	first, err := doc.Coil(t.Context(), s, p, coilAxisV, units.Millimeters(1.5), units.Scalar(2.5))
	require.NoError(t, err)
	second, err := doc.Coil(t.Context(), s, p, coilAxisV, units.Millimeters(1.5), units.Scalar(2.5))
	require.NoError(t, err)
	fp, sp := first.payload.(coilPayload), second.payload.(coilPayload)
	require.Equal(t, fp.verts, sp.verts)
	require.Equal(t, fp.vertexBound, sp.vertexBound)
	require.Equal(t, fp.tris, sp.tris)
	for i, f := range first.Faces() {
		require.Equal(t, f.Origins()[0].Role, second.Faces()[i].Origins()[0].Role)
	}

	rot, err := r3.Rotation(r3.NewVec(1, 2, 2), units.Degrees(37))
	require.NoError(t, err)
	fv, err := first.Volume()
	require.NoError(t, err)
	fa, err := first.Area()
	require.NoError(t, err)
	fc, err := first.Centroid()
	require.NoError(t, err)
	placedA, err := first.Placed(t.Context(), rot)
	require.NoError(t, err)
	placedB, err := second.Placed(t.Context(), rot)
	require.NoError(t, err)
	require.Equal(t, placedA.payload.(coilPayload).verts, placedB.payload.(coilPayload).verts)

	// L = rot·[U V N] is orthonormal only to rounding, so the placed coil
	// denotes the affine image of the unplaced one: its volume is det(L)
	// times the unplaced closed form and a cap's area is |L·U × L·V| times
	// the recorded one. Both are read here off the placement's own floats as
	// exact rationals, independently of the build.
	basis := rot.Basis()
	col := func(v r3.Vec) [3]*big.Rat {
		return [3]*big.Rat{new(big.Rat).SetFloat64(v.X), new(big.Rat).SetFloat64(v.Y), new(big.Rat).SetFloat64(v.Z)}
	}
	ex, ey, ez := col(basis.EX), col(basis.EY), col(basis.EZ)
	cross := func(a, b [3]*big.Rat) [3]*big.Rat {
		c := func(i, j int) *big.Rat {
			return new(big.Rat).Sub(new(big.Rat).Mul(a[i], b[j]), new(big.Rat).Mul(a[j], b[i]))
		}
		return [3]*big.Rat{c(1, 2), c(2, 0), c(0, 1)}
	}
	dot := func(a, b [3]*big.Rat) *big.Rat {
		out := new(big.Rat)
		for i := range 3 {
			out.Add(out, new(big.Rat).Mul(a[i], b[i]))
		}
		return out
	}
	det := dot(ex, cross(ey, ez))
	require.NotZero(t, det.Cmp(big.NewRat(1, 1)), "the fixture's rotation must not be exactly orthonormal")
	pcp := placedA.payload.(coilPayload)
	prec, err := coilRecordOfPayload(pcp)
	require.NoError(t, err)
	pm := coil.RegionMoments(prec.Rho, prec.Zeta, prec.LoopIdx, big.NewRat(1, 1), prec.Axis.Side)
	wantVol := new(big.Float).SetPrec(512).Mul(bigPi(), big.NewFloat(12.5))
	wantVol.Mul(wantVol, new(big.Float).SetPrec(512).SetRat(det))
	requireIntervalHolds(t, coilshell.Volume(prec, pm), wantVol, "placed volume enclosure")
	ux := cross(ex, ey)
	capScale := new(big.Float).SetPrec(512).SetRat(dot(ux, ux))
	capScale.Sqrt(capScale)
	capArea, err := coilFace(t, placedA, roleCapStart).Area()
	require.NoError(t, err)
	requireEnclosesBig(t, capArea.Value.Base(), capArea.Bound.Base(), capScale, "placed cap area")

	// Volume and area stay within both bodies' bounds of the unplaced ones.
	pv, err := placedA.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(pv.Value.Base()-fv.Value.Base()), pv.Bound.Base()+fv.Bound.Base())
	pa, err := placedA.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(pa.Value.Base()-fa.Value.Base()), pa.Bound.Base()+fa.Bound.Base())
	pc, err := placedA.Centroid()
	require.NoError(t, err)
	moved := rot.Apply(fc.Value)
	require.LessOrEqual(t, pc.Value.Sub(moved).Len(), pc.Bound.Base()+fc.Bound.Base()+1e-12)

	box, err := placedA.Bounds()
	require.NoError(t, err)
	for _, q := range coilSamples(1.5, 2.5, 1) {
		w := rot.Apply(q)
		require.True(t, w.X >= box.Min.X-1e-12 && w.X <= box.Max.X+1e-12)
		require.True(t, w.Y >= box.Min.Y-1e-12 && w.Y <= box.Max.Y+1e-12)
		require.True(t, w.Z >= box.Min.Z-1e-12 && w.Z <= box.Max.Z+1e-12)
	}
}

// closestOnTriangle is the distance from p to the closed triangle abc,
// Ericson's Voronoi-region walk in float64.
func closestOnTriangle(p, a, b, c r3.Vec) float64 {
	ab, ac, ap := b.Sub(a), c.Sub(a), p.Sub(a)
	d1, d2 := ab.Dot(ap), ac.Dot(ap)
	if d1 <= 0 && d2 <= 0 {
		return ap.Len()
	}
	bp := p.Sub(b)
	d3, d4 := ab.Dot(bp), ac.Dot(bp)
	if d3 >= 0 && d4 <= d3 {
		return bp.Len()
	}
	vc := d1*d4 - d3*d2
	if vc <= 0 && d1 >= 0 && d3 <= 0 {
		return p.Sub(a.Add(ab.Scale(d1 / (d1 - d3)))).Len()
	}
	cp := p.Sub(c)
	d5, d6 := ab.Dot(cp), ac.Dot(cp)
	if d6 >= 0 && d5 <= d6 {
		return cp.Len()
	}
	vb := d5*d2 - d1*d6
	if vb <= 0 && d2 >= 0 && d6 <= 0 {
		return p.Sub(a.Add(ac.Scale(d2 / (d2 - d6)))).Len()
	}
	va := d3*d6 - d5*d4
	if va <= 0 && d4-d3 >= 0 && d5-d6 >= 0 {
		w := (d4 - d3) / ((d4 - d3) + (d5 - d6))
		return p.Sub(b.Add(c.Sub(b).Scale(w))).Len()
	}
	denom := 1 / (va + vb + vc)
	return p.Sub(a.Add(ab.Scale(vb * denom)).Add(ac.Scale(vc * denom))).Len()
}

// TestCoilFacetBoundHoldsTheSurface samples the true helicoidal wall of
// every cell of each fixture — Φ evaluated in float64 straight from §3's
// formula — and asserts each sample lies within the cell's facet bound (the
// largest β over its four corners) of the cell's two held triangles. It is a
// falsifier of docs/helix-design.md §5.4's β, never its proof. Legs shown to
// fail by deleting them and watching this test go red, then restoring them:
//
//   - the twist leg of coil.CellDepartureUpper: the annular walls of every
//     fixture went red;
//   - the sag leg: the split band's middle band went red.
//
// The wide profile ρ ∈ [2, 4] read Suspect on Bounds under the matched-corner
// bound |Δρ|·sin(π/256)/2 ≈ 1.2e-2 mm against a tolerance near 9e-3 mm; it
// verifies Sound under the shifted correspondence.
func TestCoilFacetBoundHoldsTheSurface(t *testing.T) {
	for _, c := range []struct {
		name  string
		loop  [][2]float64
		turns float64
	}{
		{"square spring", [][2]float64{{2, 0}, {3, 0}, {3, 1}, {2, 1}}, 2},
		{"wide profile", [][2]float64{{2, 0}, {4, 0}, {4, 1}, {2, 1}}, 1.5},
		{"near the axis", [][2]float64{{0.5, 0}, {3, 0}, {3, 1}, {0.5, 1}}, 1},
		// The band between ζ = 0.3 and ζ = 0.6 touches no annular wall, so
		// its facet bound is its own sag and rounding alone.
		{"split band", [][2]float64{{2, 0}, {3, 0}, {3, 0.3}, {3, 0.6}, {3, 1}, {2, 1}}, 1},
	} {
		t.Run(c.name, func(t *testing.T) {
			s, p := coilLoopsSketch(t, c.loop)
			doc := New()
			const pitch = 1.5
			b, err := doc.Coil(t.Context(), s, p, coilAxisV, units.Millimeters(pitch), units.Scalar(c.turns))
			require.NoError(t, err)
			cp := b.payload.(coilPayload)
			rec, err := coilRecordOfPayload(cp)
			require.NoError(t, err)
			stride, n := len(rec.Pts), rec.N
			k := pitch / (2 * math.Pi)
			theta := 2 * math.Pi * c.turns
			worst := 0.0
			for j := range n {
				idx := rec.LoopIdx[0]
				for kk := range stride {
					v, w := idx[kk], idx[(kk+1)%stride]
					pv, pw := rec.Pts[v], rec.Pts[w]
					corners := []int{int(j)*stride + v, int(j)*stride + w, int(j+1)*stride + v, int(j+1)*stride + w}
					facet := 0.0
					for _, q := range corners {
						facet = math.Max(facet, cp.vertexBound[q])
					}
					cell := 2 * (int(j)*stride + kk)
					tris := [][3]int{cp.tris[cell], cp.tris[cell+1]}
					for _, l := range []float64{0, 0.25, 0.5, 0.75, 1} {
						for _, sf := range []float64{0, 0.2, 0.4, 0.5, 0.6, 0.8, 1} {
							rho := pv.U + l*(pw.U-pv.U)
							zeta := pv.V + l*(pw.V-pv.V)
							th := theta * (float64(j) + sf) / float64(n)
							q := r3.NewVec(rho*math.Cos(th), zeta+k*th, -rho*math.Sin(th))
							d := math.Inf(1)
							for _, tri := range tris {
								d = math.Min(d, closestOnTriangle(q, cp.verts[tri[0]], cp.verts[tri[1]], cp.verts[tri[2]]))
							}
							// The sample's own float evaluation is off by a few ulps.
							require.LessOrEqual(t, d, facet+1e-12, "cell %d of segment %d at (%v, %v)", j, v, l, sf)
							worst = math.Max(worst, d/facet)
						}
					}
				}
			}
			require.Positive(t, worst)
			rep, err := doc.Verify(t.Context())
			require.NoError(t, err)
			if c.name != "near the axis" {
				require.Equal(t, Sound, rep.Status)
			}
		})
	}
}
