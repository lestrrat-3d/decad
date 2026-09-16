package decad_test

import (
	"fmt"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file proves the revolve angular-denotation rule PR 1 adopts
// (docs/evaluator-design.md §6): every reading built on a held sweep angle
// must contain the angle the recorded AngularExtent denotes, not merely the
// float the resolver rounded to. Every assertion below is a RELATION — a
// containment, a ratio, or a zero/nonzero split — never a bound literal,
// since the bound differs between amd64 and arm64 through FMA.

// TestRevolveBoundsEnclosesDenotedExtreme is design §11 test 9: the box a
// degree-stated quarter turn publishes must contain the exact quarter-turn
// minimum (cos(pi/2) = 0) once it is grown by its own proven bound, and that
// bound must be nonzero — the charge is not free. A wide (120deg/180deg)
// sweep's box stays tight through the interior-critical-angle arm this PR
// also adds to sweepExtremeBounds. A partial-sweep cap vertex carries the
// same charge and nothing else, so it is exactly zero for a radian-stated
// sweep and nonzero for a degree-stated one; its cap face's own normal
// carries the charge on top of the frame's own baseline rounding, so it
// stays nonzero-but-tiny for a radian-stated sweep and grows (still tiny)
// for a degree-stated one.
func TestRevolveBoundsEnclosesDenotedExtreme(t *testing.T) {
	t.Parallel()

	t.Run("quarter turn box encloses the exact minimum", func(t *testing.T) {
		s, p := annularSketch(t)
		doc := decad.New()
		body, err := doc.Revolve(s, p, uAxis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
		require.NoError(t, err)

		box, err := body.Bounds()
		require.NoError(t, err)
		require.Positive(t, box.Bound.Base(), `the angular displacement charge is not free`)

		// The quarter turn's true Y minimum is cos(pi/2) = 0 exactly; the
		// held float misses it by ~3e-16, and the box must contain 0 once
		// grown by its own bound.
		decadtest.Encloses(t, "quarter-turn box", box, r3.NewVec(box.Min.X, 0, box.Min.Z))
	})

	t.Run("wide sweeps keep a tight box", func(t *testing.T) {
		for _, deg := range []float64{120, 180} {
			t.Run("", func(t *testing.T) {
				s, p := annularSketch(t)
				doc := decad.New()
				body, err := doc.Revolve(s, p, uAxis, decad.AngleExtent{A: units.Degrees(deg), Dir: decad.Along})
				require.NoError(t, err)

				box, err := body.Bounds()
				require.NoError(t, err)
				diameter := box.Max.Sub(box.Min).Len()
				decadtest.HasBoundAtMost(t, "wide-sweep box", box.Bound, units.Millimeters(1e-9*diameter))
			})
		}
	})

	t.Run("partial cap vertex and normal carry the angular charge", func(t *testing.T) {
		s, p := annularSketch(t)

		degDoc := decad.New()
		degBody, err := degDoc.Revolve(s, p, uAxis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
		require.NoError(t, err)

		radDoc := decad.New()
		radBody, err := radDoc.Revolve(s, p, uAxis, decad.AngleExtent{A: units.Radians(math.Pi / 2), Dir: decad.Along})
		require.NoError(t, err)

		degCap := faceByRole(t, degBody, roleCapEnd)
		radCap := faceByRole(t, radBody, roleCapEnd)

		degEdges := degCap.Loops()[0].Edges()
		radEdges := radCap.Loops()[0].Edges()
		require.NotEmpty(t, degEdges)
		require.NotEmpty(t, radEdges)

		degVertex := degEdges[0].Start()
		radVertex := radEdges[0].Start()
		require.Positive(t, degVertex.Position().Bound.Base(),
			`a degree-stated sweep's cap vertex carries the angular displacement`)
		require.Zero(t, radVertex.Position().Bound.Base(),
			`a radian-stated sweep denotes its own held angle exactly, so its cap vertex is exact`)

		degPlane, ok := degCap.Surface().(decad.Plane)
		require.True(t, ok)
		radPlane, ok := radCap.Surface().(decad.Plane)
		require.True(t, ok)

		degNormal, err := degCap.NormalAt(degPlane.Frame.Origin())
		require.NoError(t, err)
		radNormal, err := radCap.NormalAt(radPlane.Frame.Origin())
		require.NoError(t, err)
		require.Positive(t, degNormal.Bound.Base(),
			`a degree-stated sweep's cap normal carries the angular displacement`)
		// The frame's own cross-product rounding (planeNormalAllow,
		// normal_bound.go) contributes a baseline ulp-level bound even at a
		// radian-stated angle, so the radian case is not claimed exactly
		// zero here — only that adopting the denotation rule keeps it at
		// that same tiny scale rather than widening it to the old envelope.
		decadtest.HasBoundAtMost(t, "radian-stated cap normal", radNormal.Bound, units.Scalar(1e-9))
	})
}

// The tests below prove rp.sweep()'s own tightening (revolve_denotation.go):
// it replaces the old |h|+2π magnitude-envelope fallback with the denoted
// sweep width's own certified bracket wherever the denotation admits one,
// and the partial-sweep centroid's endpoint sin/cos (endSinCos) takes the
// matching tightening. Every assertion is a RELATION — a ratio ceiling or an
// independently computed containment — never a bound literal, since the
// bound differs between amd64 and arm64 through FMA.

// piRefLo and piRefHi bracket pi to 60 decimal digits: a REFERENCE these
// tests compute their own expected answer from, never a bound production
// code trusts (that proof lives in rat_interval.go's own piLower/piUpper,
// unreachable from this external package). Checking both ends against the
// published interval is what proves the tightened bound still sound, not
// merely narrow.
var (
	piRefLo = mustTestDecimal("3.14159265358979323846264338327950288419716939937510582097494")
	piRefHi = mustTestDecimal("3.14159265358979323846264338327950288419716939937510582097495")
)

func mustTestDecimal(s string) *big.Rat {
	r, ok := new(big.Rat).SetString(s)
	if !ok {
		panic("decad_test: invalid decimal constant " + s)
	}
	return r
}

// sweepWidthCase names one sweep this file checks: the extent to revolve
// with, and the denoted width's own bracket in radians (lo, hi), built from
// the record's own exact rational content independently of decad — turn is a
// fraction of a full turn (0 for a radian-stated sweep) and rad is an exact
// radian addend (the whole width for a radian-stated sweep).
type sweepWidthCase struct {
	name string
	ext  decad.AngularExtent
	turn *big.Rat
	rad  *big.Rat
}

func (c sweepWidthCase) widthBracket() (lo, hi *big.Rat) {
	lo = new(big.Rat).Add(c.rad, new(big.Rat).Mul(new(big.Rat).Mul(big.NewRat(2, 1), piRefLo), c.turn))
	hi = new(big.Rat).Add(c.rad, new(big.Rat).Mul(new(big.Rat).Mul(big.NewRat(2, 1), piRefHi), c.turn))
	return lo, hi
}

// annularSweepCases is the sweep list TestRevolveSweepBoundTightens checks: a
// full turn, a half turn, a quarter turn, an awkward 37 degrees, a
// radian-stated sweep, a symmetric 50-degrees-each-way extent and an uneven
// two-sided 30/70 extent — every AngularExtent variant the denotation covers.
func annularSweepCases() []sweepWidthCase {
	zero := new(big.Rat)
	return []sweepWidthCase{
		{"full", decad.FullRevolution{}, big.NewRat(1, 1), zero},
		{"180deg", decad.AngleExtent{A: units.Degrees(180), Dir: decad.Along}, big.NewRat(1, 2), zero},
		{"90deg", decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along}, big.NewRat(1, 4), zero},
		{"37deg", decad.AngleExtent{A: units.Degrees(37), Dir: decad.Along}, big.NewRat(37, 360), zero},
		{"1rad", decad.AngleExtent{A: units.Radians(1), Dir: decad.Along}, zero, big.NewRat(1, 1)},
		{
			"symmetric50deg", decad.SymmetricAngle{A: units.Degrees(50)},
			big.NewRat(100, 360), zero, // 50 deg each way: 100 deg total
		},
		{
			"twoSided30_70", decad.TwoSidedAngle{
				One: decad.AngleSide{A: units.Degrees(30)},
				Two: decad.AngleSide{A: units.Degrees(70)},
			},
			big.NewRat(100, 360), zero, // 30 + 70 deg total, split unevenly
		},
	}
}

// TestRevolveSweepBoundTightens proves the volume bound for every sweep in
// annularSweepCases shrinks from the old |h|+2π envelope (measured ratio 2 to
// 10.7 before this change) to a fraction of the value, and that the
// published interval [Value−Bound, Value+Bound] contains q·D for BOTH ends
// of the independently computed pi bracket.
func TestRevolveSweepBoundTightens(t *testing.T) {
	t.Parallel()

	// q = ∫ρ dA = 1000 mm³ exactly for annularSketch (A = 100 mm², mean
	// radius 10 mm; revolve_test.go's own comment on annularSketch).
	q := big.NewRat(1000, 1)

	for _, c := range annularSweepCases() {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			s, p := annularSketch(t)
			doc := decad.New()
			body, err := doc.Revolve(s, p, uAxis, c.ext)
			require.NoError(t, err)

			vol, err := body.Volume()
			require.NoError(t, err)
			value, bound := vol.Value.Base(), vol.Bound.Base()

			require.LessOrEqual(t, bound, 1e-9*value,
				`the sweep's own certified bracket must replace the old |h|+2pi envelope`)

			lo, hi := c.widthBracket()
			trueLo, _ := new(big.Rat).Mul(q, lo).Float64()
			trueHi, _ := new(big.Rat).Mul(q, hi).Float64()
			require.LessOrEqual(t, value-bound, trueLo,
				`the published interval must not exclude the pi bracket's lower end`)
			require.GreaterOrEqual(t, value+bound, trueHi,
				`the published interval must not exclude the pi bracket's upper end`)
		})
	}
}

// TestRevolveRadianSweepStaysExact proves a radian-stated sweep denotes
// itself exactly (rad has no rounding to charge), so its volume publishes
// Exact with a zero bound and the value Pappus by hand gives: q·1 = 1000.
func TestRevolveRadianSweepStaysExact(t *testing.T) {
	t.Parallel()
	s, p := annularSketch(t)
	doc := decad.New()
	body, err := doc.Revolve(s, p, uAxis, decad.AngleExtent{A: units.Radians(1), Dir: decad.Along})
	require.NoError(t, err)

	vol, err := body.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, vol.Exactness)
	require.Zero(t, vol.Bound.Base())
	require.Equal(t, 1000.0, vol.Value.Base())
}

// TestRevolveFullTurnStaysApproximate is the negative guard beside
// TestRevolveRadianSweepStaysExact: a full turn's own denoted width is
// 2*pi, which no float64 holds exactly, so its volume must stay Approximate
// with a nonzero bound even after the tightening — never Exact, which would
// mean the turn component was read as though it denoted a radian value
// instead of a fraction of pi.
func TestRevolveFullTurnStaysApproximate(t *testing.T) {
	t.Parallel()
	s, p := annularSketch(t)
	doc := decad.New()
	body, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	vol, err := body.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, vol.Exactness)
	require.Positive(t, vol.Bound.Base())
}

// TestRevolvePartialCentroidBoundTightens proves the partial-sweep
// centroid's bound shrinks from the old per-endpoint sin/cos's ≥1 envelope
// (measured 460-469 mm before this change) to a fraction of the centroid's
// own distance from the origin, and that the closed-form centroid
// revolve_test.go's TestRevolvePartialSweeps derives from the moments by
// hand lies inside the published interval.
func TestRevolvePartialCentroidBoundTightens(t *testing.T) {
	t.Parallel()

	// mzr = ∫zp dA = 5000, mrr = ∫p^2 dA = 32500/3 for annularSketch (u in
	// [0,10], v in [5,15]); q = 1000. axial = mzr/q = 5 is sweep-independent.
	const (
		mzr = 5000.0
		mrr = 32500.0 / 3
		q   = 1000.0
	)

	for _, degrees := range []float64{90, 37} {
		t.Run(fmt.Sprintf("%gdeg", degrees), func(t *testing.T) {
			t.Parallel()
			s, p := annularSketch(t)
			doc := decad.New()
			body, err := doc.Revolve(s, p, uAxis, decad.AngleExtent{A: units.Degrees(degrees), Dir: decad.Along})
			require.NoError(t, err)

			cen, err := body.Centroid()
			require.NoError(t, err)
			value, bound := cen.Value, cen.Bound.Base()

			require.LessOrEqual(t, bound, 1e-9*value.Len(),
				`the endpoint trig's own certified enclosure must replace the old ge-1 envelope`)

			sweep := degrees * math.Pi / 180
			sin, cos := math.Sincos(sweep)
			radialScale := mrr / (sweep * q)
			wantX := mzr / q
			wantY := radialScale * sin
			wantZ := radialScale * (1 - cos)

			require.InDelta(t, wantX, value.X, bound+1e-9)
			require.InDelta(t, wantY, value.Y, bound+1e-9)
			require.InDelta(t, wantZ, value.Z, bound+1e-9)
		})
	}
}

// widthBracketFor returns the widthBracket of the named case in
// annularSweepCases, for a test that needs the reference pi bracket without
// duplicating the case table.
func widthBracketFor(t *testing.T, name string) (lo, hi *big.Rat) {
	t.Helper()
	for _, c := range annularSweepCases() {
		if c.name == name {
			return c.widthBracket()
		}
	}
	t.Fatalf("decad_test: no annularSweepCases entry named %q", name)
	return nil, nil
}

// TestRevolveStraightWallAreaTightens is design §11 test 5: walkAxisMoment's
// straight arm, the latitude-circle length and the junction-arc length all
// replace their old |Δφ|+2π-scaled magnitude envelope with composed bounded
// arithmetic over the walk's own already-proven inputs (segmentWalk.length
// with lengthBound, startV/endV with startVBound/endVBound). It proves this
// on annularSketch's full turn, whose corners sweep latitude circles
// (checked against 2*pi*rho), and its 37-degree sweep, whose corners sweep
// junction arcs (checked against rho*D) — D the sweep's own reference pi
// bracket, independent of decad.
func TestRevolveStraightWallAreaTightens(t *testing.T) {
	t.Parallel()

	t.Run("full turn / latitude circles", func(t *testing.T) {
		t.Parallel()
		s, p := annularSketch(t)
		doc := decad.New()
		body, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{})
		require.NoError(t, err)

		area, err := body.Area()
		require.NoError(t, err)
		require.LessOrEqual(t, area.Bound.Base(), 1e-9*area.Value.Base(),
			`walkAxisMoment's straight arm must replace the old magnitude envelope`)
		for _, f := range body.Faces() {
			fa, err := f.Area()
			require.NoError(t, err)
			require.LessOrEqual(t, fa.Bound.Base(), 1e-9*fa.Value.Base())
		}

		twoPiLo := new(big.Rat).Mul(big.NewRat(2, 1), piRefLo)
		twoPiHi := new(big.Rat).Mul(big.NewRat(2, 1), piRefHi)
		sawLatitude := false
		for _, e := range body.Edges() {
			l, err := e.Length()
			require.NoError(t, err)
			value, bound := l.Value.Base(), l.Bound.Base()
			require.LessOrEqual(t, bound, 1e-9*math.Max(value, 1))

			circle, ok := e.Curve().(decad.Circle3)
			if !ok {
				continue
			}
			sawLatitude = true
			rho := new(big.Rat).SetFloat64(circle.Radius.Base())
			require.NotNil(t, rho)
			trueLo, _ := new(big.Rat).Mul(twoPiLo, rho).Float64()
			trueHi, _ := new(big.Rat).Mul(twoPiHi, rho).Float64()
			require.LessOrEqual(t, value-bound, trueLo,
				`the published interval must not exclude 2*pi*rho's lower end`)
			require.GreaterOrEqual(t, value+bound, trueHi,
				`the published interval must not exclude 2*pi*rho's upper end`)
		}
		require.True(t, sawLatitude, `a full annular revolve's junctions are latitude circles`)
	})

	t.Run("37deg / junction arcs", func(t *testing.T) {
		t.Parallel()
		s, p := annularSketch(t)
		doc := decad.New()
		body, err := doc.Revolve(s, p, uAxis, decad.AngleExtent{A: units.Degrees(37), Dir: decad.Along})
		require.NoError(t, err)

		area, err := body.Area()
		require.NoError(t, err)
		require.LessOrEqual(t, area.Bound.Base(), 1e-9*area.Value.Base(),
			`walkAxisMoment's straight arm must replace the old magnitude envelope`)
		for _, f := range body.Faces() {
			fa, err := f.Area()
			require.NoError(t, err)
			require.LessOrEqual(t, fa.Bound.Base(), 1e-9*fa.Value.Base())
		}

		widthLo, widthHi := widthBracketFor(t, "37deg")
		sawArc := false
		for _, e := range body.Edges() {
			l, err := e.Length()
			require.NoError(t, err)
			value, bound := l.Value.Base(), l.Bound.Base()
			require.LessOrEqual(t, bound, 1e-9*math.Max(value, 1))

			arc, ok := e.Curve().(decad.Arc3)
			if !ok {
				continue
			}
			sawArc = true
			rho := new(big.Rat).SetFloat64(arc.Radius.Base())
			require.NotNil(t, rho)
			trueLo, _ := new(big.Rat).Mul(rho, widthLo).Float64()
			trueHi, _ := new(big.Rat).Mul(rho, widthHi).Float64()
			require.LessOrEqual(t, value-bound, trueLo,
				`the published interval must not exclude rho*D's lower end`)
			require.GreaterOrEqual(t, value+bound, trueHi,
				`the published interval must not exclude rho*D's upper end`)
		}
		require.True(t, sawArc, `a partial annular revolve's junctions are swept arcs`)
	})
}

// TestRevolveCircularWallAreaTightens is design §11 test 7:
// circularAxisMomentInterval (moments_circular.go) replaces walkAxisMoment's
// circular arm's old |Δφ|+2π-scaled magnitude envelope with a
// rational-interval closed form over the wall's own recorded segment. It
// proves this on a full-turn torus (a whole CircleSeg wall, checked against
// 4*pi^2*R*r) and the grooved partial sweep (an ArcSeg wall,
// revolve_test.go's grooveSketch) — the same envelope walkAxisMoment's
// straight arm carried before TestRevolveStraightWallAreaTightens.
func TestRevolveCircularWallAreaTightens(t *testing.T) {
	t.Parallel()

	t.Run("torus / whole circle", func(t *testing.T) {
		t.Parallel()
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		center := s.CreatePoint(0, 10)
		s.Fix(center)
		s.CreateCircle(center, 3)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)

		doc := decad.New()
		body, err := doc.Revolve(s, s.Profiles()[0], uAxis, decad.FullRevolution{})
		require.NoError(t, err)

		area, err := body.Area()
		require.NoError(t, err)
		value, bound := area.Value.Base(), area.Bound.Base()
		require.LessOrEqual(t, bound, 1e-9*value,
			`circularAxisMomentInterval must replace walkAxisMoment's old circular-arm envelope`)

		// Pappus: a torus's area is its generating circle's circumference
		// (2*pi*r) times the distance its centroid travels (2*pi*R) — 4*pi^2*R*r
		// with R = 10 (the circle's centre distance from the axis) and r = 3.
		fourPiSqLo := new(big.Rat).Mul(new(big.Rat).Mul(big.NewRat(4, 1), piRefLo), piRefLo)
		fourPiSqHi := new(big.Rat).Mul(new(big.Rat).Mul(big.NewRat(4, 1), piRefHi), piRefHi)
		rr := big.NewRat(30, 1) // R*r = 10*3
		trueLo, _ := new(big.Rat).Mul(fourPiSqLo, rr).Float64()
		trueHi, _ := new(big.Rat).Mul(fourPiSqHi, rr).Float64()
		require.LessOrEqual(t, value-bound, trueLo,
			`the published interval must not exclude 4*pi^2*R*r's lower end`)
		require.GreaterOrEqual(t, value+bound, trueHi,
			`the published interval must not exclude 4*pi^2*R*r's upper end`)
	})

	t.Run("groove / arc wall", func(t *testing.T) {
		t.Parallel()
		s, p := grooveSketch(t)
		doc := decad.New()
		body, err := doc.Revolve(s, p, uAxis, decad.AngleExtent{A: units.Radians(math.Pi / 2), Dir: decad.Along})
		require.NoError(t, err)

		area, err := body.Area()
		require.NoError(t, err)
		require.LessOrEqual(t, area.Bound.Base(), 1e-9*area.Value.Base(),
			`circularAxisMomentInterval must replace walkAxisMoment's old circular-arm envelope`)

		sawTorus := false
		for _, f := range body.Faces() {
			if _, ok := f.Surface().(decad.Torus); !ok {
				continue
			}
			sawTorus = true
			fa, err := f.Area()
			require.NoError(t, err)
			require.LessOrEqual(t, fa.Bound.Base(), 1e-9*fa.Value.Base(),
				`the groove wall's own face area must carry the tightened bound`)
		}
		require.True(t, sawTorus, `the groove's wall is the body's one torus`)
	})
}

// TestRevolveVerifySoundAnnular is design §11 test 6: with PRs 1-3 all
// landed, annularSketch verifies Sound with no diagnostics for both a full
// turn and an awkward 37-degree sweep — every reading the revolve publishes
// (box, volume, area, centroid, edge lengths) now contains the value its own
// record denotes within the default tolerance.
func TestRevolveVerifySoundAnnular(t *testing.T) {
	t.Parallel()

	for _, ext := range []decad.AngularExtent{
		decad.FullRevolution{},
		decad.AngleExtent{A: units.Degrees(37), Dir: decad.Along},
	} {
		t.Run(fmt.Sprintf("%T", ext), func(t *testing.T) {
			t.Parallel()
			s, p := annularSketch(t)
			doc := decad.New()
			_, err := doc.Revolve(s, p, uAxis, ext)
			require.NoError(t, err)

			decadtest.IsSound(t, doc)
		})
	}
}

// tiltedAxis is a SketchLine axis on direction (3,4)/5 through (0,-20): the
// annularSketch rectangle (u in [0,10], v in [5,15]) lies wholly on its +
// side. 3-4-5 is a Pythagorean triple, so the axis's own held length is
// exactly representable and squares back to the exact rational 25 — the
// condition axisDirectionSqrtBracket's sqrt bracket is checked against below
// — while the unit direction 3/5, 4/5 it divides out is NOT itself an
// exact float64, so the direction bound is not the zero-charge case
// TestRevolveRadianSweepStaysExact's sibling would be.
var tiltedAxis = decad.SketchLine{Start: decad.Point2{U: 0, V: -20}, End: decad.Point2{U: 3, V: -16}}

// TestRevolveTiltedAxisBoundsTighten is design §11 test 8: the axis
// direction's own sqrt bracket (axisDirectionSqrtBracket, the transfer of
// the straight-prism campaign's lineWalkBounds/sqrtIntervalError) replaces
// sketchAxisDirectionBounds's old conservativeValueError(dU, 1) envelope —
// measured ratio ~4.07 before this change on tiltedAxis, for both a full
// turn and a 1-radian sweep — so the tilted-axis volume and area bounds
// shrink to the same tiny fraction of their values every axis-aligned
// bracket already reaches. The volume is checked against an independent
// hand computation: rho, the perpendicular distance from annularSketch's
// area centroid (5, 10) to tiltedAxis, is linear in (u, v), so
// integral(rho dA) over the region equals rho at the centroid times the
// region's own area (100 mm^2) — q' = 14 * 100 = 1400 mm^3 — and Pappus
// gives Volume = q' * sweep width.
func TestRevolveTiltedAxisBoundsTighten(t *testing.T) {
	t.Parallel()

	t.Run("full turn", func(t *testing.T) {
		t.Parallel()
		s, p := annularSketch(t)
		doc := decad.New()
		body, err := doc.Revolve(s, p, tiltedAxis, decad.FullRevolution{})
		require.NoError(t, err)

		vol, err := body.Volume()
		require.NoError(t, err)
		value, bound := vol.Value.Base(), vol.Bound.Base()
		require.LessOrEqual(t, bound, 1e-9*value,
			`axisDirectionSqrtBracket must replace sketchAxisDirectionBounds's old envelope`)

		qPrime := big.NewRat(1400, 1)
		trueLo, _ := new(big.Rat).Mul(qPrime, new(big.Rat).Mul(big.NewRat(2, 1), piRefLo)).Float64()
		trueHi, _ := new(big.Rat).Mul(qPrime, new(big.Rat).Mul(big.NewRat(2, 1), piRefHi)).Float64()
		require.LessOrEqual(t, value-bound, trueLo,
			`the published interval must not exclude q'*2*pi's lower end`)
		require.GreaterOrEqual(t, value+bound, trueHi,
			`the published interval must not exclude q'*2*pi's upper end`)

		area, err := body.Area()
		require.NoError(t, err)
		require.LessOrEqual(t, area.Bound.Base(), 1e-9*area.Value.Base(),
			`axisDirectionSqrtBracket must replace the old envelope in the area's own axis-moment reads`)
	})

	t.Run("1 rad", func(t *testing.T) {
		t.Parallel()
		s, p := annularSketch(t)
		doc := decad.New()
		body, err := doc.Revolve(s, p, tiltedAxis, decad.AngleExtent{A: units.Radians(1), Dir: decad.Along})
		require.NoError(t, err)

		vol, err := body.Volume()
		require.NoError(t, err)
		value, bound := vol.Value.Base(), vol.Bound.Base()
		require.LessOrEqual(t, bound, 1e-9*value,
			`axisDirectionSqrtBracket must replace sketchAxisDirectionBounds's old envelope`)
		require.Equal(t, 1400.0, value, `q' * a 1-radian sweep is exactly q'`)

		area, err := body.Area()
		require.NoError(t, err)
		require.LessOrEqual(t, area.Bound.Base(), 1e-9*area.Value.Base(),
			`axisDirectionSqrtBracket must replace the old envelope in the area's own axis-moment reads`)
	})
}
