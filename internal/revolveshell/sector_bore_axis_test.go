package revolveshell_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/coneaxis"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// The general bore ring places its inner vertices on the two caps of the
// six-span hub about (1, 2, 0). Every vertex, and every chord's midpoint,
// lies within the returned bound of the exact circle of radius 1 about the
// exact axis on its cap, read in 512-bit arithmetic, and the plane point
// SweptPointGap reads holds the exact cap point at that radius within its
// stated box (agent-docs/revolve-axis-design.md §9.3).
func TestBoreRingAboutAxisCoversTheBoreCircles(t *testing.T) {
	t.Parallel()
	design := [][2]float64{{6, 0}, {9, 0}, {9, 3}, {8, 4}, {5, 2.5}, {6, 1.5}}
	outline := make([]sectionrecord.Point2, len(design))
	for i, p := range design {
		outline[i] = sectionrecord.Point2{U: p[0]/2 - p[1], V: p[0] + p[1]/2}
	}
	hub := revolveFixtureAbout(t, outline, sectionrecord.Point2{U: 1, V: 2})
	candidates := revolveshell.RootCandidates(hub)
	require.NotEmpty(t, candidates)
	section := candidates[0]
	require.NotNil(t, section.General)
	axis := section.Axis.Vec()
	count := len(section.Spans)
	low := section.Spans[(section.AxisSpan+count-1)%count].Start
	high := section.Spans[(section.AxisSpan+1)%count].End
	const stations, radius = 64, 1.0
	angles := make([]float64, stations)
	for j := range angles {
		angles[j] = 2 * math.Pi * float64(j) / stations
	}
	maxStep, ok := revolvemesh.SectorRingMaxStep(angles)
	require.True(t, ok)
	lowRing, highRing, bound, plane, within, err := revolveshell.BoreRingAbout(t.Context(), section, low, high,
		radius, angles, maxStep, 0)
	require.NoError(t, err)
	limit := ringBig(bound)
	moved := 0
	for k, ring := range [2][]r3.Vec{lowRing, highRing} {
		capPoint := [2]sectionrecord.Point2{low, high}[k]
		h, _ := circleReading(section.Frame.ToWorldUV(capPoint.U, capPoint.V), axis)
		off := func(p r3.Vec) *big.Float {
			hp, rp := circleReading(p, axis)
			dh := new(big.Float).SetPrec(ringPrec).Sub(hp, h)
			dr := new(big.Float).SetPrec(ringPrec).Sub(rp, ringBig(radius))
			sum := new(big.Float).SetPrec(ringPrec).Add(new(big.Float).SetPrec(ringPrec).Mul(dh, dh),
				new(big.Float).SetPrec(ringPrec).Mul(dr, dr))
			return sum.Sqrt(sum)
		}
		require.Len(t, ring, stations)
		for j, v := range ring {
			distance := off(v)
			if distance.Sign() > 0 {
				moved++
			}
			require.LessOrEqual(t, distance.Cmp(limit), 0, "cap %d station %d lies %s off the circle, bound %g",
				k, j, distance.Text('g', 6), bound)
			next := ring[(j+1)%stations]
			mid := v.Add(next).Scale(0.5)
			require.LessOrEqual(t, off(mid).Cmp(limit), 0, "cap %d chord %d leaves the circle by %s, bound %g",
				k, j, off(mid).Text('g', 6), bound)
		}
	}
	require.Positive(t, moved, "some vertex must round off its circle")
	// The exact plane point is (Z/A·A)·(1, 2) + (radius/|A|)·(−2, 1), with Z
	// the low cap's exact axial coordinate and |A| = √5.
	z, _ := section.Axis.PlaneScaled(section.Frame, low.U, low.V)
	norm := new(big.Float).SetPrec(ringPrec).Sqrt(ringBig(5))
	scale := new(big.Float).SetPrec(ringPrec).Quo(new(big.Float).SetPrec(ringPrec).SetRat(z.Rat()), ringBig(5))
	offset := new(big.Float).SetPrec(ringPrec).Quo(ringBig(radius), norm)
	for i, pair := range [2][2]float64{{1, -2}, {2, 1}} {
		exact := new(big.Float).SetPrec(ringPrec).Add(new(big.Float).SetPrec(ringPrec).Mul(scale, ringBig(pair[0])),
			new(big.Float).SetPrec(ringPrec).Mul(offset, ringBig(pair[1])))
		held, box := [2]float64{plane.U, plane.V}[i], [2]float64{within.U, within.V}[i]
		gap := new(big.Float).SetPrec(ringPrec).Sub(exact, ringBig(held))
		require.LessOrEqual(t, gap.Abs(gap).Cmp(ringBig(box)), 0, "plane coordinate %d is %s off, box %g", i,
			gap.Text('g', 6), box)
	}
}

// boreFrame builds a tool frame from its origin and in-plane axes.
func boreFrame(t *testing.T, origin, u, v r3.Vec) r3.Frame {
	t.Helper()
	frame, err := r3.NewFrame(origin, u, v)
	require.NoError(t, err)
	return frame
}

// wallDeviation is the largest | |p×Â| − r | over sampled tool wall points
// p = O + z·N + r·(cos θ·U + sin θ·V), read in 512-bit arithmetic.
func wallDeviation(frame r3.Frame, axis r3.Vec, radius float64, levels []float64) *big.Float {
	worst := new(big.Float).SetPrec(ringPrec)
	for _, z := range levels {
		for k := range 720 {
			theta := 2 * math.Pi * float64(k) / 720
			var p [3]*big.Float
			o, n, u, v := frame.Origin(), frame.N(), frame.U(), frame.V()
			// The float cosine and sine scaled to an exact unit pair, so the
			// sample lies on the wall the frame denotes.
			c, s := ringBig(math.Cos(theta)), ringBig(math.Sin(theta))
			unit := new(big.Float).SetPrec(ringPrec).Add(new(big.Float).SetPrec(ringPrec).Mul(c, c),
				new(big.Float).SetPrec(ringPrec).Mul(s, s))
			unit.Sqrt(unit)
			c.Quo(c, unit)
			s.Quo(s, unit)
			for i, parts := range [3][4]float64{{o.X, n.X, u.X, v.X}, {o.Y, n.Y, u.Y, v.Y}, {o.Z, n.Z, u.Z, v.Z}} {
				x := ringBig(parts[0])
				x.Add(x, new(big.Float).SetPrec(ringPrec).Mul(ringBig(z), ringBig(parts[1])))
				w := new(big.Float).SetPrec(ringPrec).Add(new(big.Float).SetPrec(ringPrec).Mul(c, ringBig(parts[2])),
					new(big.Float).SetPrec(ringPrec).Mul(s, ringBig(parts[3])))
				x.Add(x, w.Mul(w, ringBig(radius)))
				p[i] = x
			}
			_, rho := circleReadingBig(p, axis)
			gap := new(big.Float).SetPrec(ringPrec).Sub(rho, ringBig(radius))
			if gap.Abs(gap).Cmp(worst) > 0 {
				worst = gap
			}
		}
	}
	return worst
}

// circleReadingBig is circleReading for a point held in 512-bit arithmetic.
func circleReadingBig(p [3]*big.Float, a r3.Vec) (*big.Float, *big.Float) {
	av := [3]*big.Float{ringBig(a.X), ringBig(a.Y), ringBig(a.Z)}
	dot := func(x, y [3]*big.Float) *big.Float {
		out := new(big.Float).SetPrec(ringPrec)
		for i := range x {
			out.Add(out, new(big.Float).SetPrec(ringPrec).Mul(x[i], y[i]))
		}
		return out
	}
	norm := new(big.Float).SetPrec(ringPrec).Sqrt(dot(av, av))
	h := new(big.Float).SetPrec(ringPrec).Quo(dot(p, av), norm)
	rhoSq := new(big.Float).SetPrec(ringPrec).Sub(dot(p, p), new(big.Float).SetPrec(ringPrec).Mul(h, h))
	if rhoSq.Sign() < 0 {
		rhoSq.SetInt64(0)
	}
	return h, new(big.Float).SetPrec(ringPrec).Sqrt(rhoSq)
}

// The tool deviation covers the sampled distance of the tool's side wall
// from the coaxial cylinder: for an exactly orthonormal frame whose origin
// sits 2⁻⁴⁷ off the axis (0, 1, 0), and for a frame whose normal tilts
// 10⁻¹³ off it along a 1000 mm tool. Each fixture realises its own term:
// the first is zero without the origin's offset, the second far below its
// sampled deviation without the normal's tilt
// (agent-docs/revolve-axis-design.md §9.4).
func TestBoreToolDeviationCoversTheWall(t *testing.T) {
	t.Parallel()
	y, ok := coneaxis.New(r3.Vec{Y: 1})
	require.True(t, ok)
	offset := boreFrame(t, r3.Vec{X: 0x1p-47, Y: 4}, r3.Vec{Z: 1}, r3.Vec{X: 1})
	tilted := boreFrame(t, r3.Vec{Y: 4}, r3.Vec{Z: 1}, r3.Vec{X: 1, Y: 1e-13})
	for name, tc := range map[string]struct {
		frame  r3.Frame
		z1     float64
		radius float64
	}{
		"offset origin": {offset, 6, 1},
		"tilted normal": {tilted, 1000, 1},
	} {
		prism := prismrecord.Payload{Frame: tc.frame, Z0: 0, Z1: tc.z1}
		deviation, ok := revolveshell.BoreToolDeviation(prism, y, tc.radius)
		require.True(t, ok, name)
		sampled := wallDeviation(tc.frame, y.Vec(), tc.radius, []float64{0, tc.z1 / 2, tc.z1})
		require.Positive(t, sampled.Sign(), name)
		require.LessOrEqual(t, sampled.Cmp(ringBig(deviation)), 0, "%s: sampled %s, bound %g", name,
			sampled.Text('g', 6), deviation)
	}
	exact := boreFrame(t, r3.Vec{Y: 4}, r3.Vec{Z: 1}, r3.Vec{X: 1})
	deviation, ok := revolveshell.BoreToolDeviation(prismrecord.Payload{Frame: exact, Z1: 6}, y, 1)
	require.True(t, ok)
	require.Zero(t, deviation, "an exact frame on the axis has no deviation")
}

// A tool frame is coaxial when its normal and origin lie on the axis to
// rounding: the frame of a 70° shaft rounded to 26 bits, built as a
// consumer builds it, passes, while a normal tilted 10⁻¹³, an origin 10⁻¹³
// off the axis and a normal against the axis decline
// (agent-docs/revolve-axis-design.md §9.1).
func TestCoaxialBoreFrameAdmitsRoundingAndDeclinesTilt(t *testing.T) {
	t.Parallel()
	round26 := func(v float64) float64 {
		frac, exp := math.Frexp(v)
		return math.Ldexp(math.Round(math.Ldexp(frac, 26)), exp-26)
	}
	shaft := 70 * math.Pi / 180
	a := r3.Vec{X: round26(math.Cos(shaft)), Y: round26(math.Sin(shaft))}
	axis, ok := coneaxis.New(a)
	require.True(t, ok)
	unit := a.Scale(1 / a.Len())
	normal := r3.Vec{X: -unit.Y, Y: unit.X}
	require.True(t, revolveshell.CoaxialBoreFrame(boreFrame(t, unit.Scale(3.25), normal, r3.Vec{Z: 1}), axis))
	y, ok := coneaxis.New(r3.Vec{Y: 1})
	require.True(t, ok)
	require.True(t, revolveshell.CoaxialBoreFrame(boreFrame(t, r3.Vec{Y: 4}, r3.Vec{Z: 1}, r3.Vec{X: 1}), y))
	for name, frame := range map[string]r3.Frame{
		"tilted normal":  boreFrame(t, r3.Vec{Y: 4}, r3.Vec{Z: 1}, r3.Vec{X: 1, Y: 1e-13}),
		"offset origin":  boreFrame(t, r3.Vec{X: 1e-13, Y: 4}, r3.Vec{Z: 1}, r3.Vec{X: 1}),
		"reverse normal": boreFrame(t, r3.Vec{Y: 4}, r3.Vec{X: 1}, r3.Vec{Z: 1}),
	} {
		require.False(t, revolveshell.CoaxialBoreFrame(frame, y), name)
	}
}

// Both end discs must clear the caps. With a normal tilted 10⁻¹³ off the
// axis (0, 1, 0), a disc of radius 1 spans 2·10⁻¹³ along the axis about its
// centre, so a cap 5·10⁻¹⁴ past the centre's level still cuts the disc,
// while one 2·10⁻¹³ past it clears it (agent-docs/revolve-axis-design.md
// §9.1).
func TestBoreToolSpansCapsReadsTheTiltedDisc(t *testing.T) {
	t.Parallel()
	y, ok := coneaxis.New(r3.Vec{Y: 1})
	require.True(t, ok)
	frame := boreFrame(t, r3.Vec{Y: 4}, r3.Vec{Z: 1}, r3.Vec{X: 1, Y: 1e-13})
	prism := prismrecord.Payload{Frame: frame, Z0: 0, Z1: 6}
	level := func(z float64) *big.Rat {
		o, n := frame.Origin(), frame.N()
		out := proofarith.FloatRat(o.Y)
		return out.Add(out, new(big.Rat).Mul(proofarith.FloatRat(z), proofarith.FloatRat(n.Y)))
	}
	shift := func(z float64, by float64) *big.Rat {
		return new(big.Rat).Add(level(z), proofarith.FloatRat(by))
	}
	require.True(t, revolveshell.BoreToolSpansCaps(prism, y, 1, shift(0, 2e-13), shift(6, -2e-13)))
	require.False(t, revolveshell.BoreToolSpansCaps(prism, y, 1, shift(0, 5e-14), shift(6, -2e-13)))
	require.False(t, revolveshell.BoreToolSpansCaps(prism, y, 1, shift(0, 2e-13), shift(6, -5e-14)))
	require.False(t, revolveshell.BoreToolSpansCaps(prism, y, 1, shift(0, -1), shift(6, -2e-13)))
}

// The general band reading of the six-span hub about (1, 2, 0), whose band
// runs from design point (5, 2.5) to the low cap's (6, 1.5) at slope 1, for a
// radius-2 bore whose tool deviates by 10⁻³ mm. A tool wall that far outside
// the radius crosses the band at radius 2 + 10⁻³ and 10⁻³ mm farther along
// the axis, √2·10⁻³ from the ideal crossing. Every clipped vertex lies within
// the bound, less the chord sag, of the exact crossing at either radius at
// its own angle, read in 512-bit arithmetic; every chord midpoint lies within
// the bound of the ideal crossing circle (agent-docs/revolve-axis-design.md
// §9.7). The high inner ring is ring's (§9.3).
func TestBoreBandAboutAxisCoversTheCrossing(t *testing.T) {
	t.Parallel()
	design := [][2]float64{{6, 0}, {9, 0}, {9, 3}, {8, 4}, {5, 2.5}, {6, 1.5}}
	outline := make([]sectionrecord.Point2, len(design))
	for i, p := range design {
		outline[i] = sectionrecord.Point2{U: p[0]/2 - p[1], V: p[0] + p[1]/2}
	}
	hub := revolveFixtureAbout(t, outline, sectionrecord.Point2{U: 1, V: 2})
	candidates := revolveshell.RootCandidates(hub)
	require.NotEmpty(t, candidates)
	section := candidates[0]
	require.NotNil(t, section.General)
	const stations, radius, deviation = 64, 2.0, 1e-3
	angles := make([]float64, stations)
	for j := range angles {
		angles[j] = 2 * math.Pi * float64(j) / stations
	}
	maxStep, ok := revolvemesh.SectorRingMaxStep(angles)
	require.True(t, ok)
	mesh := &facetedrecord.JoinedMesh{}
	ring, _, err := revolveshell.BuildRevolvedSectionRing(mesh, section, angles, nil, 0.1)
	require.NoError(t, err)
	cut, high, bound, err := revolveshell.BoreBandAbout(t.Context(), section, mesh.Verts, ring, radius, maxStep,
		deviation)
	require.NoError(t, err)
	require.Len(t, cut, stations)
	require.Len(t, high, stations)
	sag := ringBig(radius * maxStep * maxStep / 8)
	vertexLimit := new(big.Float).SetPrec(ringPrec).Sub(ringBig(bound), sag)
	// The plane point of design point (x, r) is (x/2)·(1, 2) + (r/2)·(−2, 1),
	// at axial h = x·√5/2 and radius ρ = r·√5/2 mm. The design band is
	// r = 7.5 − x, so it crosses radius ρ at h = 7.5·√5/2 − ρ.
	axis := section.Axis.Vec()
	apex := new(big.Float).SetPrec(ringPrec).Sqrt(ringBig(5))
	apex.Mul(apex, ringBig(7.5/2))
	off := func(p r3.Vec, rho float64) *big.Float {
		h, rp := circleReading(p, axis)
		dh := new(big.Float).SetPrec(ringPrec).Sub(h, new(big.Float).SetPrec(ringPrec).Sub(apex, ringBig(rho)))
		dr := new(big.Float).SetPrec(ringPrec).Sub(rp, ringBig(rho))
		sum := new(big.Float).SetPrec(ringPrec).Add(new(big.Float).SetPrec(ringPrec).Mul(dh, dh),
			new(big.Float).SetPrec(ringPrec).Mul(dr, dr))
		return sum.Sqrt(sum)
	}
	for j, v := range cut {
		for _, rho := range []float64{radius, radius + deviation} {
			distance := off(v, rho)
			require.LessOrEqual(t, distance.Cmp(vertexLimit), 0,
				"station %d lies %s off the crossing at radius %g, bound less sag %s", j, distance.Text('g', 6),
				rho, vertexLimit.Text('g', 6))
		}
		mid := v.Add(cut[(j+1)%stations]).Scale(0.5)
		require.LessOrEqual(t, off(mid, radius).Cmp(ringBig(bound)), 0, "chord %d leaves the circle", j)
	}
}
