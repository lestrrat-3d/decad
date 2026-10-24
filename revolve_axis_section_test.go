package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/revolveproof"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The section-certificate fixtures about a general axis
// (agent-docs/revolve-axis-design.md §7.3). Each builds the hub and radial lug
// of the public sector Union tests about one axis:
//
//   - x: the +X control, the hub on the world XY plane about U and the lug on
//     the plane X = 10;
//   - permuted: every input mapped by the exact rotation (x, y, z) → (z, x, y),
//     so the axis is A = (0, 1, 0) and the fixture is congruent to x;
//   - tilted: the hub on the world XY plane about the line to (1, 2), so
//     A = (1, 2, 0) of irrational length √5, and the lug on the plane 10 mm
//     along A. The hub's design point (z, r) is the plane point
//     z/2·(1, 2) + r/2·(−2, 1), so its A-scaled readings are Z = 2.5z and
//     R = 2.5r and its root cone keeps the apex at the origin and slope 1/2.

// taperedHubOutline is the hub of the public sector Union tests in design
// coordinates (z, r). Its root cone runs from (8, 4) to (5, 2.5).
var taperedHubOutline = []planePoint{{U: 6}, {U: 9}, {U: 9, V: 1.5}, {U: 9, V: 3}, {U: 8, V: 4},
	{U: 5, V: 2.5}, {U: 6, V: 1.5}}

// axisRevolve fully revolves a closed polygon drawn on a plane through the
// origin with axes u and v about the sketch line from (0, 0) to end.
func axisRevolve(t *testing.T, doc *Document, u, v r3.Vec, end planePoint, outline []planePoint) *Body {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(axisPlane(t, world, r3.Vec{}, u, v))
	require.NoError(t, err)
	points := make([]*sketch.Point, len(outline))
	for i, p := range outline {
		points[i] = s.CreatePoint(p.U, p.V)
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	body, err := Revolve(t.Context(), doc, s, profiles[0], SketchLine{Start: planePoint{}, End: end},
		FullRevolution{})
	require.NoError(t, err)
	return body
}

// tiltedPoint maps a design point (z, r) to the XY plane point
// z/2·(1, 2) + r/2·(−2, 1).
func tiltedPoint(p planePoint) planePoint {
	return planePoint{U: p.U/2 - p.V, V: p.U + p.V/2}
}

func tiltedOutline(outline []planePoint) []planePoint {
	out := make([]planePoint, len(outline))
	for i, p := range outline {
		out[i] = tiltedPoint(p)
	}
	return out
}

// axisLug lofts the radial lug of the public sector Union tests, drawn on the
// plane with the given frame, from the origin and trims it with the toe and
// heel tools.
func axisLug(t *testing.T, doc *Document, origin, u, v r3.Vec, fitted bool, toe, heel *Body) *Body {
	t.Helper()
	const rootRadius, half = 4.3, 0.4
	world := sketch.NewWorld()
	s, err := world.CreateSketch(axisPlane(t, world, origin, u, v))
	require.NoError(t, err)
	rootLow, flankLow := s.CreatePoint(-rootRadius, -half), s.CreatePoint(-5.5, -half)
	tipLow, tipHigh := s.CreatePoint(-6, -half), s.CreatePoint(-6, half)
	flankHigh, rootHigh := s.CreatePoint(-5.5, half), s.CreatePoint(-rootRadius, half)
	center := s.CreatePoint(0, 0)
	for _, p := range []*sketch.Point{rootLow, flankLow, tipLow, tipHigh, flankHigh, rootHigh, center} {
		s.Fix(p)
	}
	flank := func(a, b *sketch.Point, midV float64) {
		if !fitted {
			s.CreateLine(a, b)
			return
		}
		mid := s.CreatePoint(-(rootRadius+5.5)/2, midV)
		s.Fix(mid)
		_, err := s.CreateFitSpline(a, mid, b)
		require.NoError(t, err)
	}
	flank(rootLow, flankLow, -1.2*half)
	s.CreateLine(flankLow, tipLow)
	s.CreateArc(center, tipHigh, tipLow)
	s.CreateLine(tipHigh, flankHigh)
	flank(flankHigh, rootHigh, 1.2*half)
	s.CreateArc(center, rootHigh, rootLow)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	lug, err := LoftFromPoint(t.Context(), doc, r3.Vec{}, s, profiles[0])
	require.NoError(t, err)
	cut, err := Cut(t.Context(), lug, toe)
	require.NoError(t, err)
	trimmed, err := Intersect(t.Context(), cut, heel)
	require.NoError(t, err)
	return trimmed
}

// sectionAxisFixture is one hub and trimmed lug about one axis.
type sectionAxisFixture struct {
	doc      *Document
	hub, lug *Body
	axis     r3.Vec
}

func xSectionFixture(t *testing.T, fitted bool) sectionAxisFixture {
	t.Helper()
	doc := New()
	return sectionAxisFixture{doc: doc, axis: worldX,
		hub: axisRevolve(t, doc, worldX, worldY, planePoint{U: 1}, taperedHubOutline),
		lug: axisLug(t, doc, r3.Vec{X: 10}, r3.Vec{Y: -1}, r3.Vec{Z: -1}, fitted,
			pointLoftConeTool(t, doc, 7.5), pointLoftConeTool(t, doc, 12))}
}

func permutedSectionFixture(t *testing.T, fitted bool) sectionAxisFixture {
	t.Helper()
	doc := New()
	return sectionAxisFixture{doc: doc, axis: worldY,
		hub: axisRevolve(t, doc, worldY, worldZ, planePoint{U: 1}, taperedHubOutline),
		lug: axisLug(t, doc, r3.Vec{Y: 10}, r3.Vec{Z: -1}, r3.Vec{X: -1}, fitted,
			permutedConeTool(t, doc, 7.5), permutedConeTool(t, doc, 12))}
}

// tiltedSectionFixture trims the lug with cones about (1, 2, 0) of slope 1
// whose apexes t·(1, 2) lie 7.55 and 12.02 mm along the axis, near the +X toe
// 7.5 and heel 12. Each t is dyadic, so every tool point is exact.
// toolEnd is the end of the tools' axis line, (1, 2) or a multiple of it.
func tiltedSectionFixture(t *testing.T, fitted bool, toolEnd planePoint) sectionAxisFixture {
	t.Helper()
	doc := New()
	s5 := math.Sqrt(5)
	tool := func(apexT float64) *Body {
		return axisConeTool(t, doc, worldX, worldY, toolEnd, planePoint{U: -1, V: -2},
			planePoint{U: -1 - 2*(apexT+1), V: -2 + apexT + 1}, planePoint{U: apexT, V: 2 * apexT})
	}
	return sectionAxisFixture{doc: doc, axis: r3.Vec{X: 1, Y: 2},
		hub: axisRevolve(t, doc, worldX, worldY, planePoint{U: 1, V: 2}, tiltedOutline(taperedHubOutline)),
		lug: axisLug(t, doc, r3.Vec{X: 10 / s5, Y: 20 / s5}, r3.Vec{X: 2 / s5, Y: -1 / s5}, r3.Vec{Z: -1},
			fitted, tool(3.375), tool(5.375))}
}

func (f sectionAxisFixture) certify(t *testing.T) *facetedrecord.RevolveLoftSectorCertificate {
	t.Helper()
	cert, ok := revolveshell.CertifySector(bodyNode(f.hub), bodyNode(f.lug))
	require.True(t, ok, "the sector must certify about %v", f.axis)
	return cert
}

// rootAngles returns the two root-boundary node angles of the certificate's
// exterior path.
func rootAngles(t *testing.T, cert *facetedrecord.RevolveLoftSectorCertificate) [2]float64 {
	t.Helper()
	path, _, err := revolveproof.RevolveExteriorPath(cert.Base, cert.Boundary, cert.Base.PointSection.Source,
		cert.Section)
	require.NoError(t, err)
	lo, hi, ok := revolveproof.RevolveRootAngles(path)
	require.True(t, ok)
	return [2]float64{lo, hi}
}

// The congruent fixture about (0, 1, 0) certifies the same root span with the
// same boundary roles as +X, reads the same tool gap, and its root contacts
// have the same angles in the blank's own basis.
func TestCertifySectorAboutPermutedAxisMatchesX(t *testing.T) {
	t.Parallel()
	for _, fitted := range []bool{false, true} {
		x := xSectionFixture(t, fitted).certify(t)
		y := permutedSectionFixture(t, fitted).certify(t)
		require.Nil(t, x.Section.General)
		require.NotNil(t, y.Section.General)
		require.Equal(t, worldY, y.Section.Axis.Vec())
		require.Equal(t, sketch.ConeSection{Axis: worldY, Slope: 0.5}, y.Section.ConeSection())
		require.Zero(t, big.NewRat(1, 2).Cmp(y.Section.General.Slope))
		require.Zero(t, y.Section.General.Apex.Sign())
		require.Equal(t, x.Section.RootSpan, y.Section.RootSpan)
		require.Equal(t, x.Section.RootStart, y.Section.RootStart)
		require.Equal(t, x.Section.RootEnd, y.Section.RootEnd)
		require.Equal(t, x.Boundary.FitRoute, y.Boundary.FitRoute)
		require.Equal(t, fitted, y.Boundary.FitRoute)
		require.Equal(t, x.Boundary.InsideSegment, y.Boundary.InsideSegment)
		require.Equal(t, x.Boundary.OutsideWitness, y.Boundary.OutsideWitness)
		for k := range x.Boundary.Contacts {
			require.Equal(t, x.Boundary.Contacts[k].Segment, y.Boundary.Contacts[k].Segment)
		}
		xGap, ok := revolveshell.RevolveSectionToolGap(x.Section, x.Trim)
		require.True(t, ok)
		yGap, ok := revolveshell.RevolveSectionToolGap(y.Section, y.Trim)
		require.True(t, ok)
		require.Equal(t, xGap, yGap, "A = (0, 1, 0) reads every tool-gap rational of +X")
		xAngles, yAngles := rootAngles(t, x), rootAngles(t, y)
		for k := range xAngles {
			require.InDelta(t, xAngles[k], yAngles[k], 1e-12, "fitted %v", fitted)
		}
	}
}

// The hub and lug about (1, 2, 0) certify exactly one root face, read the
// exact root cone, and place the root contacts at the +X angles in the blank's
// basis.
func TestCertifySectorAboutTiltedAxis(t *testing.T) {
	t.Parallel()
	for _, fitted := range []bool{false, true} {
		f := tiltedSectionFixture(t, fitted, planePoint{U: 1, V: 2})
		cert := f.certify(t)
		certified := 0
		for _, section := range revolveshell.RootCandidates(bodyNode(f.hub)) {
			base := cert.Base
			_, connector := revolveproof.CertifyRevolveConnectorBoundary(base.PointSection, section, base)
			_, fit := revolveproof.CertifyRevolveFitBoundary(base.PointSection, section, base)
			if connector || fit {
				certified++
			}
		}
		require.Equal(t, 1, certified)
		general := cert.Section.General
		require.NotNil(t, general)
		require.Equal(t, f.axis, cert.Section.Axis.Vec())
		require.Equal(t, sketch.ConeSection{Axis: f.axis, Slope: 0.5}, cert.Section.ConeSection())
		require.Zero(t, big.NewRat(1, 2).Cmp(general.Slope))
		require.Zero(t, general.Apex.Sign())
		require.Equal(t, tiltedPoint(planePoint{U: 5, V: 2.5}), cert.Section.RootStart)
		require.Equal(t, tiltedPoint(planePoint{U: 8, V: 4}), cert.Section.RootEnd)
		require.Equal(t, fitted, cert.Boundary.FitRoute)
		x := xSectionFixture(t, fitted).certify(t)
		xAngles, angles := rootAngles(t, x), rootAngles(t, cert)
		for k := range xAngles {
			require.InDelta(t, xAngles[k], angles[k], 1e-9, "fitted %v", fitted)
		}
	}
}

// coneGapHub revolves, on the world XY plane about the line to (1, 2), a hub
// whose corners are given as exact A-scaled (Z, R) pairs. Each pair has
// Z − 2R divisible by 5, so its plane point ((Z − 2R)/5, (2Z + R)/5) is exact.
func coneGapHub(t *testing.T, doc *Document, scaled [][2]float64) *Body {
	t.Helper()
	outline := make([]planePoint, len(scaled))
	for i, p := range scaled {
		outline[i] = planePoint{U: (p[0] - 2*p[1]) / 5, V: (2*p[0] + p[1]) / 5}
	}
	return axisRevolve(t, doc, worldX, worldY, planePoint{U: 1, V: 2}, outline)
}

// rootCandidate returns the candidate whose root span runs between the plane
// points a and b.
func rootCandidate(t *testing.T, hub *Body, a, b planePoint) facetedrecord.RevolveSectionCertificate {
	t.Helper()
	for _, c := range revolveshell.RootCandidates(bodyNode(hub)) {
		if c.RootStart == a && c.RootEnd == b {
			return c
		}
	}
	require.FailNow(t, "no root candidate", "%v to %v", a, b)
	return facetedrecord.RevolveSectionCertificate{}
}

const coneGapPrec = 512

func bigVec(v r3.Vec) [3]*big.Float {
	return [3]*big.Float{big.NewFloat(v.X).SetPrec(coneGapPrec), big.NewFloat(v.Y).SetPrec(coneGapPrec),
		big.NewFloat(v.Z).SetPrec(coneGapPrec)}
}

func bigDot(a, b [3]*big.Float) *big.Float {
	out := new(big.Float).SetPrec(coneGapPrec)
	for i := range a {
		out.Add(out, new(big.Float).SetPrec(coneGapPrec).Mul(a[i], b[i]))
	}
	return out
}

func bigAxpy(x [3]*big.Float, a *big.Float, y [3]*big.Float) [3]*big.Float {
	var out [3]*big.Float
	for i := range x {
		out[i] = new(big.Float).SetPrec(coneGapPrec).Add(x[i], new(big.Float).SetPrec(coneGapPrec).Mul(a, y[i]))
	}
	return out
}

func bigUnit(v [3]*big.Float) [3]*big.Float {
	n := new(big.Float).SetPrec(coneGapPrec).Sqrt(bigDot(v, v))
	var out [3]*big.Float
	for i := range v {
		out[i] = new(big.Float).SetPrec(coneGapPrec).Quo(v[i], n)
	}
	return out
}

// coneDistance is the distance from x to the nappe with apex o, unit axis
// unit and slope k, |ρ − k·h|/√(1+k²), for a point whose foot lies on the
// nappe (h + k·ρ > 0).
func coneDistance(t *testing.T, x, o, unit [3]*big.Float, k *big.Float) *big.Float {
	t.Helper()
	w := bigAxpy(x, big.NewFloat(-1), o)
	h := bigDot(w, unit)
	radial := bigAxpy(w, new(big.Float).Neg(h), unit)
	rho := new(big.Float).SetPrec(coneGapPrec).Sqrt(bigDot(radial, radial))
	require.Positive(t, new(big.Float).Add(h, new(big.Float).Mul(k, rho)).Sign())
	d := new(big.Float).SetPrec(coneGapPrec).Sub(rho, new(big.Float).Mul(k, h))
	den := new(big.Float).SetPrec(coneGapPrec).Add(big.NewFloat(1), new(big.Float).Mul(k, k))
	return d.Abs(d).Quo(d, den.Sqrt(den))
}

// sampledConeDistance returns the largest distance, sampled over the root
// span's axial range from P* and eight directions about A, from the true root
// cone K* to the reported cone K_r and back. It is computed independently of
// the certificate's bounds, in 512-bit arithmetic.
func sampledConeDistance(t *testing.T, section facetedrecord.RevolveSectionCertificate) *big.Float {
	t.Helper()
	general := section.General
	a := bigVec(section.Axis.Vec())
	unit := bigUnit(a)
	// The fixtures' axes lie in the XY plane, so (−A_y, A_x, 0) and Z complete
	// an orthogonal frame about A.
	v := section.Axis.Vec()
	e0 := bigUnit(bigVec(r3.Vec{X: -v.Y, Y: v.X}))
	e1 := bigVec(r3.Vec{Z: 1})
	normSq := new(big.Float).SetPrec(coneGapPrec).SetRat(section.Axis.NormSq().Rat())
	norm := new(big.Float).SetPrec(coneGapPrec).Sqrt(normSq)
	scale := new(big.Float).SetPrec(coneGapPrec).Quo(new(big.Float).SetPrec(coneGapPrec).SetRat(general.Apex), normSq)
	var exactApex [3]*big.Float
	for i := range a {
		exactApex[i] = new(big.Float).SetPrec(coneGapPrec).Mul(scale, a[i])
	}
	reported := bigVec(general.Cone.Apex)
	kStar := new(big.Float).SetPrec(coneGapPrec).SetRat(general.Slope)
	s := big.NewFloat(general.Cone.Slope).SetPrec(coneGapPrec)
	axial := func(z *big.Rat) *big.Float {
		d := new(big.Float).SetPrec(coneGapPrec).SetRat(new(big.Rat).Sub(z, general.Apex))
		return d.Quo(d, norm)
	}
	zLo, zHi := axial(general.RootZ[0]), axial(general.RootZ[1])
	worst := new(big.Float).SetPrec(coneGapPrec)
	const steps = 8
	for i := 0; i <= steps; i++ {
		z := new(big.Float).SetPrec(coneGapPrec).Sub(zHi, zLo)
		z.Mul(z, big.NewFloat(float64(i)/steps)).Add(z, zLo)
		for j := range 8 {
			sin, cos := math.Sincos(float64(j) * math.Pi / 4.3)
			dir := bigUnit(bigAxpy(bigAxpy([3]*big.Float{new(big.Float), new(big.Float), new(big.Float)},
				big.NewFloat(cos), e0), big.NewFloat(sin), e1))
			for _, c := range []struct {
				from, to   [3]*big.Float
				fromK, toK *big.Float
			}{{exactApex, reported, kStar, s}, {reported, exactApex, s, kStar}} {
				onCone := bigAxpy(bigAxpy(c.from, z, unit), new(big.Float).Mul(c.fromK, z), dir)
				if d := coneDistance(t, onCone, c.to, unit, c.toK); d.Cmp(worst) > 0 {
					worst = d
				}
			}
		}
	}
	return worst
}

// ConeGap covers the distance between the true and the reported root cone
// over the span (agent-docs/revolve-axis-design.md §5.2). The first hub's root
// apex P* = (1.2, 2.4, 0) rounds and its slope 1 is exact, so only δ_a covers
// the distance; the second's apex is the origin and its slope 1/3 rounds, so
// only the |k*−s|·reach term covers it.
func TestRootConeGapAboutAxis(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name      string
		hub       [][2]float64
		a, b      planePoint
		apexExact bool
	}{
		{"rounded apex", [][2]float64{{5, 0}, {15, 0}, {15, 10}, {12, 6}, {7, 1}},
			planePoint{U: 1, V: 3}, planePoint{V: 6}, false},
		{"rounded slope", [][2]float64{{10, 0}, {40, 0}, {40, 15}, {30, 10}, {15, 5}},
			planePoint{U: 1, V: 7}, planePoint{U: 2, V: 14}, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			section := rootCandidate(t, coneGapHub(t, New(), tc.hub), tc.a, tc.b)
			general := section.General
			require.NotNil(t, general)
			require.Equal(t, tc.apexExact, general.ApexGap == 0, "apex gap %g", general.ApexGap)
			require.Equal(t, !tc.apexExact, general.SlopeGap == 0, "slope gap %g", general.SlopeGap)
			worst := sampledConeDistance(t, section)
			require.Positive(t, worst.Sign(), "the fixture must separate the two cones")
			require.GreaterOrEqual(t, big.NewFloat(section.ConeGap).Cmp(worst), 0,
				"ConeGap %g must cover the sampled distance %s", section.ConeGap, worst.Text('g', 6))
		})
	}
}

// A Sketch margin about A = (1, 2, 0) is in |A|²·mm² = 5·mm². With the mesh
// bound raised so that the implicit cost lies between a fifth of the smallest
// side margin and the margin itself, every side margin exceeds the cost read in
// mm², yet the boundary refuses because it compares the margin with cost·A·A
// (agent-docs/revolve-axis-design.md §5.3, §5.9 item 2).
func TestImplicitMarginAboutAxisScalesCost(t *testing.T) {
	t.Parallel()
	cert := tiltedSectionFixture(t, false, planePoint{U: 1, V: 2}).certify(t)
	record, section, base := cert.Base.PointSection, cert.Section, cert.Base
	var minMargin *big.Rat
	for _, side := range cert.Boundary.Sides {
		if m := side.ImplicitMargin(); m != nil && (minMargin == nil || m.Cmp(minMargin) < 0) {
			minMargin = m
		}
	}
	require.NotNil(t, minMargin)
	costAt := func(meshBound float64) float64 {
		b := base
		b.MeshBound = meshBound
		cost, ok := revolveproof.RevolveImplicitDisplacement(section, b)
		require.True(t, ok)
		return cost
	}
	target, _ := new(big.Rat).Quo(minMargin, big.NewRat(5, 2)).Float64()
	lo, hi := base.MeshBound, 1.0
	require.Less(t, costAt(lo), target)
	require.Greater(t, costAt(hi), target)
	for range 200 {
		mid := (lo + hi) / 2
		if costAt(mid) < target {
			lo = mid
		} else {
			hi = mid
		}
	}
	cost := costAt(lo)
	costRat := new(big.Rat).SetFloat64(cost)
	require.Negative(t, costRat.Cmp(minMargin), "every side margin exceeds the cost in mm²")
	require.GreaterOrEqual(t, new(big.Rat).Mul(costRat, big.NewRat(5, 1)).Cmp(minMargin), 0)
	_, ok := revolveproof.CertifyRevolveConnectorBoundary(record, section, base)
	require.True(t, ok, "the unchanged fan is the control")
	raised := base
	raised.MeshBound = lo
	_, ok = revolveproof.CertifyRevolveConnectorBoundary(record, section, raised)
	require.False(t, ok, "a margin below cost·A·A must refuse")
}

// A section about a general axis refuses inexact or mismatched inputs
// (agent-docs/revolve-axis-design.md §4.1, §4.4, §4.6).
func TestSectionAboutAxisRefusals(t *testing.T) {
	t.Parallel()
	t.Run("snapped axis point", func(t *testing.T) {
		t.Parallel()
		// The axis corner (3, 6) one unit in the last place off the line: the
		// Revolve snaps it onto the axis and sweeps no face there, but its exact
		// R is not zero, so the span is not the axis span.
		doc := New()
		exact := axisRevolve(t, doc, worldX, worldY, planePoint{U: 1, V: 2}, tiltedOutline(taperedHubOutline))
		_, _, _, ok := revolveshell.RecordedSection(bodyNode(exact))
		require.True(t, ok, "the exact hub is the control")
		outline := tiltedOutline(taperedHubOutline)
		require.Equal(t, planePoint{U: 3, V: 6}, outline[0])
		outline[0].V = math.Nextafter(6, 7)
		snapped := axisRevolve(t, doc, worldX, worldY, planePoint{U: 1, V: 2}, outline)
		require.Len(t, snapped.Faces(), len(exact.Faces()), "the Revolve must snap the corner onto the axis")
		_, _, _, ok = revolveshell.RecordedSection(bodyNode(snapped))
		require.False(t, ok)
		require.Empty(t, revolveshell.RootCandidates(bodyNode(snapped)))
	})
	t.Run("tool axis 2A", func(t *testing.T) {
		t.Parallel()
		f := tiltedSectionFixture(t, false, planePoint{U: 2, V: 4})
		trim := bodyNode(f.lug).Payload.(facetedPayload).PointCone
		require.NotNil(t, trim)
		require.Equal(t, r3.Vec{X: 2, Y: 4}, trim.Lower.Axis.Vec())
		require.Equal(t, r3.Vec{X: 2, Y: 4}, trim.Upper.Axis.Vec())
		require.NotEmpty(t, revolveshell.RootCandidates(bodyNode(f.hub)))
		_, ok := revolveshell.CertifySector(bodyNode(f.hub), bodyNode(f.lug))
		require.False(t, ok, "a tooth trimmed about 2·A must not certify against a blank about A")
	})
	t.Run("profile behind the origin", func(t *testing.T) {
		t.Parallel()
		// The tapered hub without its split cap point, moved back along the
		// axis by shift: at shift 7 its axis span starts at z = -1, where
		// Z = p·A is negative. Shift 0 is the control.
		outline := []planePoint{{U: 6}, {U: 9}, {U: 9, V: 3}, {U: 8, V: 4}, {U: 5, V: 2.5}, {U: 6, V: 1.5}}
		for _, shift := range []float64{0, 7} {
			moved := make([]planePoint, len(outline))
			for i, p := range outline {
				moved[i] = planePoint{U: p.U - shift, V: p.V}
			}
			hub := axisRevolve(t, New(), worldX, worldY, planePoint{U: 1, V: 2}, tiltedOutline(moved))
			_, _, _, ok := revolveshell.RecordedSection(bodyNode(hub))
			require.Equal(t, shift == 0, ok, "shift %g", shift)
		}
	})
	t.Run("ConeFromFace on a general blank", func(t *testing.T) {
		t.Parallel()
		// The root span runs from (1, 3) to (0, 6). Read as (x, r) about world
		// X it would be a valid cone side, so only the frame check refuses.
		doc := New()
		blank := coneGapHub(t, doc, [][2]float64{{5, 0}, {15, 0}, {15, 10}, {12, 6}, {7, 1}})
		before, producer := doc.Bodies(), doc.nextProducerID()
		_, err := ConeFromFace(t.Context(), blank, coneFromFaceSelector(t, blank, planePoint{U: 1, V: 3},
			planePoint{V: 6}), units.Millimeters(-1))
		require.ErrorIs(t, err, ErrUnsupported)
		require.ErrorContains(t, err, "about X")
		require.Equal(t, before, doc.Bodies())
		require.Equal(t, producer, doc.nextProducerID())
	})
}

// Every Union entry declines a certificate about a general axis until the
// general join is admitted (agent-docs/revolve-axis-design.md §4.6), so the
// pair keeps the mesh Boolean route and its refusal.
func TestSectorUnionDeclinesGeneralCertificate(t *testing.T) {
	t.Parallel()
	f := tiltedSectionFixture(t, false, planePoint{U: 1, V: 2})
	cert := f.certify(t)
	require.True(t, revolveshell.GeneralCertificate(cert))
	hub, lug := bodyNode(f.hub), bodyNode(f.lug)
	for _, pair := range [][2]*topo.Body{{hub, lug}, {lug, hub}} {
		_, ok, err := revolveshell.SectorUnion(t.Context(), meshbool.OpUnion, topo.Env{}, pair[0], pair[1])
		require.NoError(t, err)
		require.False(t, ok)
	}
	// The copy turns about the certificate's own axis, which
	// massmoment.AxisRotation admits, so only the general-certificate decline
	// keeps NextSectorUnion from the join.
	s5 := math.Sqrt(5)
	turn, err := r3.Rotation(r3.Vec{X: 1 / s5, Y: 2 / s5}, units.Degrees(180))
	require.NoError(t, err)
	_, _, admitted := massmoment.AxisRotation(turn, cert.Section.Axis)
	require.True(t, admitted)
	second, err := Copy(t.Context(), f.lug, WithCopyPlacement(turn))
	require.NoError(t, err)
	_, ok, err := revolveshell.SectorUnionMany(t.Context(), topo.Env{}, []*topo.Body{lug, hub, bodyNode(second)})
	require.NoError(t, err)
	require.False(t, ok)
	joined := &topo.Body{Payload: facetedrecord.Payload{Xform: r3.Identity(),
		RevolveLoftJoin: &facetedrecord.RevolveLoftJoinRecord{Certificate: cert, Blank: hub, First: cert.Trimmed}}}
	_, ok, err = revolveshell.NextSectorUnion(t.Context(), meshbool.OpUnion, topo.Env{}, joined, bodyNode(second))
	require.NoError(t, err)
	require.False(t, ok)

	_, err = Union(t.Context(), f.hub, f.lug)
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, "the chain cannot continue")
}

// A general certificate's sector plan reads the root radius about the axis,
// min(R0, R1)/|A|↑, and charges τ from the blank frame's tilt. The congruent
// fixture's frame is an exact permutation, so its tilt is zero; the tilted
// fixture's held direction (1, 2)/√5 rounds, so its tilt is a few units in the
// last place.
func TestGeneralPlanInputReadsRadiusAndFrameAboutAxis(t *testing.T) {
	t.Parallel()
	for name, tc := range map[string]struct {
		fixture         sectionAxisFixture
		radius, maxTilt float64
	}{
		"(0,1,0)": {permutedSectionFixture(t, false), 2.5, 0},
		"(1,2,0)": {tiltedSectionFixture(t, false, planePoint{U: 1, V: 2}), 2.5 * math.Sqrt(5) / 2, 0x1p-48},
	} {
		cert := tc.fixture.certify(t)
		var input revolveshell.SectorJoinInput
		require.NoError(t, revolveshell.GeneralPlanInput(&input, cert.Section), name)
		require.True(t, input.Tilted, name)
		require.LessOrEqual(t, input.FrameTilt, tc.maxTilt, name)
		require.GreaterOrEqual(t, input.FrameTilt, 0.0, name)
		// The root ends' smallest radius about the axis is 2.5 in design
		// units, which the tilted fixture scales by √5/2.
		require.LessOrEqual(t, input.MinRadius, tc.radius, name)
		require.InEpsilon(t, tc.radius, input.MinRadius, 1e-15, name)
	}
}
