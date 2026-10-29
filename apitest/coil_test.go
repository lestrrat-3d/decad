package apitest_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func coilSketch(t *testing.T, loops ...[][2]float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	for _, loop := range loops {
		vertices := make([]*sketch.Point, len(loop))
		for i, point := range loop {
			vertices[i] = s.CreatePoint(point[0], point[1])
			s.Fix(vertices[i])
		}
		for i := range vertices {
			s.CreateLine(vertices[i], vertices[(i+1)%len(vertices)])
		}
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.NotEmpty(t, s.Profiles())
	return s, s.Profiles()[0]
}

func coilAxis() decad.SketchLine {
	return decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{V: 1}}
}

func TestCoilPublishesSpringGeometry(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	points := [][2]float64{{2, 0}, {3, 0}, {3, 1}, {2, 1}}
	vertices := make([]*sketch.Point, len(points))
	for i, point := range points {
		vertices[i] = s.CreatePoint(point[0], point[1])
		s.Fix(vertices[i])
	}
	for i := range vertices {
		s.CreateLine(vertices[i], vertices[(i+1)%len(vertices)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)

	doc := decad.New()
	spring, err := decad.Coil(t.Context(), doc, s, s.Profiles()[0], decad.SketchLine{
		Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 0, V: 1},
	}, units.Millimeters(1.5), units.Scalar(2))
	require.NoError(t, err)

	volume, err := spring.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-10*math.Pi), volume.Bound.Base()+1e-12)
	centroid, err := spring.Centroid()
	require.NoError(t, err)
	require.LessOrEqual(t, centroid.Value.Sub(r3.NewVec(0, 2, 0)).Len(), centroid.Bound.Base())
	box, err := spring.Bounds()
	require.NoError(t, err)
	require.LessOrEqual(t, box.Min.Y, 0.0)
	require.GreaterOrEqual(t, box.Max.Y, 4.0)
}

func TestCoilThreadCutAndCoreUnionPreserveVolume(t *testing.T) {
	t.Run("external thread cut", func(t *testing.T) {
		axis := coilAxis()
		doc := decad.New()
		baseSketch, baseProfile := coilSketch(t, [][2]float64{{0, 0}, {5, 0}, {5, 20}, {0, 20}})
		cylinder, err := decad.Revolve(t.Context(), doc, baseSketch, baseProfile, axis, decad.FullRevolution{})
		require.NoError(t, err)

		const halfWidth = 0.6928203230275509
		grooveSketch, grooveProfile := coilSketch(t, [][2]float64{
			{4.1, 3}, {5.3, 3 - halfWidth}, {5.3, 3 + halfWidth},
		})
		groove, err := decad.Coil(t.Context(), doc, grooveSketch, grooveProfile, axis,
			units.Millimeters(1.5), units.Scalar(1))
		require.NoError(t, err)
		cut, err := decad.Cut(t.Context(), cylinder, groove)
		require.NoError(t, err)
		require.Len(t, cut.Lumps(), 1)

		volume, err := cut.Volume()
		require.NoError(t, err)
		clippedQ := 0.75 * halfWidth * 0.9 * 4.7
		want := math.Pi * (500 - 2*clippedQ)
		require.LessOrEqual(t, math.Abs(volume.Value.Base()-want), volume.Bound.Base()+1e-10)
		report, err := doc.Verify(t.Context())
		require.NoError(t, err)
		require.Equal(t, decad.ValidityValid, report.Bodies[len(report.Bodies)-1].Validity.Outcome)
	})

	t.Run("internal thread cut", func(t *testing.T) {
		axis := coilAxis()
		doc := decad.New()
		baseSketch, baseProfile := coilSketch(t, [][2]float64{{5, -20}, {10, -20}, {10, 0}, {5, 0}})
		block, err := decad.Revolve(t.Context(), doc, baseSketch, baseProfile, axis, decad.FullRevolution{})
		require.NoError(t, err)

		const halfWidth = 0.6928203230275509
		grooveSketch, grooveProfile := coilSketch(t, [][2]float64{
			{5.9, -17}, {4.7, -17 - halfWidth}, {4.7, -17 + halfWidth},
		})
		groove, err := decad.Coil(t.Context(), doc, grooveSketch, grooveProfile, axis,
			units.Millimeters(1.5), units.Scalar(1))
		require.NoError(t, err)
		cut, err := decad.Cut(t.Context(), block, groove)
		require.NoError(t, err)
		require.Len(t, cut.Lumps(), 1)

		volume, err := cut.Volume()
		require.NoError(t, err)
		clippedQ := 0.75 * halfWidth * 0.9 * (5.9 + 2*5) / 3
		want := math.Pi * (1500 - 2*clippedQ)
		require.LessOrEqual(t, math.Abs(volume.Value.Base()-want), volume.Bound.Base()+1e-10)
		report, err := doc.Verify(t.Context())
		require.NoError(t, err)
		require.Equal(t, decad.ValidityValid, report.Bodies[len(report.Bodies)-1].Validity.Outcome)
	})
}

func TestCoilPublicRimsBoundTheirRenderedGeometry(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(3, 0)
	s.Fix(center)
	s.CreateCircle(center, 0.5)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	body, err := decad.Coil(t.Context(), decad.New(), s, s.Profiles()[0], coilAxis(),
		units.Millimeters(1.5), units.Scalar(2.3))
	require.NoError(t, err)
	for _, edge := range body.Edges() {
		circle, ok := edge.Curve().(decad.Circle3)
		require.True(t, ok, "each round-wire rim is a public Circle3")
		require.InDelta(t, 0.5, circle.Radius.Base(), 1e-12)
		require.InDelta(t, 3, math.Hypot(circle.Center.X, circle.Center.Z), 1e-10)
		require.InDelta(t, 0, math.Abs(circle.Axis.Y), 1e-12)
	}
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.05))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestCoilPublicArcRimFollowsWalkSense(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	point := func(u, v float64) *sketch.Point {
		p := s.CreatePoint(u, v)
		s.Fix(p)
		return p
	}
	a, b, c, d := point(2, 0), point(3.2, 0), point(3.2, 1), point(2, 1)
	s.CreateLine(a, b)
	s.CreateArc(point(3.6, 0.5), c, b)
	s.CreateLine(c, d)
	s.CreateLine(d, a)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)

	body, err := decad.Coil(t.Context(), decad.New(), s, s.Profiles()[0], coilAxis(),
		units.Millimeters(3), units.Scalar(1))
	require.NoError(t, err)
	arcs := 0
	for _, edge := range body.Edges() {
		arc, ok := edge.Curve().(decad.Arc3)
		if !ok {
			continue
		}
		arcs++
		require.Less(t, arc.Axis.Z, 0.0, "the published rim follows the profile's arc orientation")
		require.False(t, edge.IsConvex())
		require.InDelta(t, math.Sqrt(0.41), arc.Radius.Base(), 1e-12)
	}
	require.Positive(t, arcs)
}

func TestCoilFullTurnArcPublishesClosedRims(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(3, 0)
	s.Fix(center)
	start := s.CreatePoint(3.5, 0)
	s.Fix(start)
	s.CreateArc(center, start, start)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)

	body, err := decad.Coil(t.Context(), decad.New(), s, s.Profiles()[0], coilAxis(),
		units.Millimeters(1.5), units.Scalar(2.3))
	require.NoError(t, err)
	require.Len(t, body.Faces(), 3)
	require.Len(t, body.Edges(), 2)
	for _, edge := range body.Edges() {
		_, ok := edge.Curve().(decad.Circle3)
		require.True(t, ok, "a whole-turn arc publishes a closed circle rim")
		require.Equal(t, edge.Start().Position().Value, edge.End().Position().Value)
		require.Len(t, edge.Faces(), 2)
	}
}

// TestCoilThreadEndClippedVolume starts the external V groove of
// agent-docs/helix-design.md §9 at ζ = -0.375, below the cylinder's end plane,
// so the cut opens through that end. Over T turns of pitch 1.5 the cut removes
// ∫ρ·(2πT − max(0, −ζ/k)) dA over the groove's part inside ρ = 5, with
// k = 1.5/(2π), so the result's volume is π·(25L − 2T·Q − (4/3)·M⁻): Q = ∫ρ dA
// over that part and M⁻ = ∫ρζ dA over its part below ζ = 0. At the gallery's
// T = 8 and L = 14 this is π·(350 − 16Q − (4/3)·M⁻); the test runs T = 2 and
// L = 5. Both endpoints of the published volume must enclose the closed form
// times both endpoints of a π enclosure, computed over exact rationals; the
// body is valid and its mesh is watertight.
func TestCoilThreadEndClippedVolume(t *testing.T) {
	t.Parallel()
	const halfWidth, start, turns, length = 0.6928203230275509, -0.375, 2, 5
	doc := decad.New()
	cylinderSketch, cylinderProfile := coilSketch(t, [][2]float64{{0, 0}, {5, 0}, {5, length}, {0, length}})
	cylinder, err := decad.Revolve(t.Context(), doc, cylinderSketch, cylinderProfile, coilAxis(), decad.FullRevolution{})
	require.NoError(t, err)
	points := [][2]float64{{4.1, start}, {5.3, start - halfWidth}, {5.3, start + halfWidth}}
	grooveSketch, grooveProfile := coilSketch(t, points)
	groove, err := decad.Coil(t.Context(), doc, grooveSketch, grooveProfile, coilAxis(),
		units.Millimeters(1.5), units.Scalar(turns))
	require.NoError(t, err)
	cut, err := decad.Cut(t.Context(), cylinder, groove)
	require.NoError(t, err)
	require.Len(t, cut.Lumps(), 1)

	rat := func(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }
	profile := make([][2]*big.Rat, len(points))
	for i, point := range points {
		profile[i] = [2]*big.Rat{rat(point[0]), rat(point[1])}
	}
	inside := clipRatPolygon(profile, func(p [2]*big.Rat) *big.Rat { return new(big.Rat).Sub(rat(5), p[0]) })
	q, _ := ratPolygonMoments(inside)
	_, below := ratPolygonMoments(clipRatPolygon(inside, func(p [2]*big.Rat) *big.Rat { return new(big.Rat).Neg(p[1]) }))
	require.Negative(t, below.Sign(), "the groove crosses the end plane")
	exactOverPi := new(big.Rat).Mul(big.NewRat(2*turns, 1), q)
	exactOverPi.Add(exactOverPi, new(big.Rat).Mul(big.NewRat(4, 3), below))
	exactOverPi.Sub(big.NewRat(25*length, 1), exactOverPi)

	volume, err := cut.Volume()
	require.NoError(t, err)
	publishedLo := new(big.Rat).Sub(rat(volume.Value.Base()), rat(volume.Bound.Base()))
	publishedHi := new(big.Rat).Add(rat(volume.Value.Base()), rat(volume.Bound.Base()))
	require.LessOrEqual(t, publishedLo.Cmp(new(big.Rat).Mul(exactOverPi, proofbound.PiLower)), 0,
		"the volume's lower end lies below the closed form")
	require.GreaterOrEqual(t, publishedHi.Cmp(new(big.Rat).Mul(exactOverPi, proofbound.PiUpper)), 0,
		"the volume's upper end lies above the closed form")

	requireBodyWatertight(t, cut)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.ValidityValid, report.Bodies[len(report.Bodies)-1].Validity.Outcome)
}

// ratPolygonMoments returns ∫ρ dA and ∫ρζ dA over a simple polygon of (ρ, ζ)
// corners, by the shoelace forms of the first and mixed moments.
func ratPolygonMoments(pts [][2]*big.Rat) (*big.Rat, *big.Rat) {
	q, m := new(big.Rat), new(big.Rat)
	mul := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Mul(a, b) }
	for i := range pts {
		x0, y0 := pts[i][0], pts[i][1]
		x1, y1 := pts[(i+1)%len(pts)][0], pts[(i+1)%len(pts)][1]
		c := new(big.Rat).Sub(mul(x0, y1), mul(x1, y0))
		q.Add(q, mul(new(big.Rat).Add(x0, x1), c))
		mixed := new(big.Rat).Add(mul(x0, y1), mul(x1, y0))
		mixed.Add(mixed, mul(big.NewRat(2, 1), new(big.Rat).Add(mul(x0, y0), mul(x1, y1))))
		m.Add(m, mul(mixed, c))
	}
	q.Quo(q, big.NewRat(6, 1))
	m.Quo(m, big.NewRat(24, 1))
	if q.Sign() < 0 {
		q.Neg(q)
		m.Neg(m)
	}
	return q, m
}

// clipRatPolygon keeps the part of a polygon where keep(p) ≥ 0 for an affine
// keep, by Sutherland–Hodgman over exact rationals.
func clipRatPolygon(pts [][2]*big.Rat, keep func(p [2]*big.Rat) *big.Rat) [][2]*big.Rat {
	var out [][2]*big.Rat
	for i := range pts {
		a, b := pts[i], pts[(i+1)%len(pts)]
		fa, fb := keep(a), keep(b)
		if fa.Sign() >= 0 {
			out = append(out, a)
		}
		if fa.Sign()*fb.Sign() < 0 {
			t := new(big.Rat).Quo(fa, new(big.Rat).Sub(fa, fb))
			var p [2]*big.Rat
			for k := range 2 {
				p[k] = new(big.Rat).Add(a[k], new(big.Rat).Mul(t, new(big.Rat).Sub(b[k], a[k])))
			}
			out = append(out, p)
		}
	}
	return out
}
