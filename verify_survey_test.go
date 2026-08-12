package decad_test

import (
	"context"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The increment-5 analytic surveys, tested against the worked tables of
// docs/verification-design.md §6: the wall rule's spanning balls, the
// undercut membership rule, and the MinRadius measurement — all Exact on
// this evaluator's prism and revolve bodies.

// rectPrism extrudes a solved w×d rectangle by h.
func rectPrism(t *testing.T, w, d, h float64) *decad.Document {
	t.Helper()
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, w, d)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	doc := decad.New()
	_, err = doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return doc
}

// polygonSketch builds a solved polygon from its vertices, in order.
func polygonSketch(t *testing.T, pts [][2]float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	require.NoError(t, err)
	sp := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		sp[i] = s.CreatePoint(p[0], p[1])
	}
	s.Fix(sp[0])
	for i := range sp {
		s.CreateLine(sp[i], sp[(i+1)%len(sp)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

// requireWall asserts a MinWallThickness reading in millimetres and the
// PROVEN exactness its own arm computed: each call site states the
// exactness its own geometry proves, never a blanket assumption.
func requireWall(t *testing.T, br *decad.BodyReport, wantExact decad.Exactness, want float64) {
	t.Helper()
	require.NotNil(t, br.MinWallThickness)
	require.Equal(t, wantExact, br.MinWallThickness.Exactness)
	require.True(t, br.MinWallThickness.Value.Equal(units.Millimeters(want), 1e-9),
		`want %v mm, got %s`, want, br.MinWallThickness.Value)
}

func TestWallThinPlateViolating(t *testing.T) {
	// The 10×10×0.5 mm plate of verification §6's worked table: the two
	// 10×10 skins oppose at 180°, the mid-plane ball spans them, and the
	// proven interval [0.5, 0.5] sits below the 1 mm tool at any coarseness.
	doc := rectPrism(t, 10, 10, 0.5)
	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)

	br := report.Bodies[0]
	requireWall(t, br, decad.Exact, 0.5)
	require.Equal(t, decad.Violating, br.Status)
	require.Equal(t, decad.Violating, report.Status)
	require.False(t, report.Trustworthy())
}

func TestWallCubeSound(t *testing.T) {
	// The 100 mm cube: its edges sit at 90°, past every legal allowance, so
	// its only spanning ball is its center's — the 100 mm slab — and a 1 mm
	// tool finds nothing to violate.
	doc := rectPrism(t, 100, 100, 100)
	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)

	br := report.Bodies[0]
	requireWall(t, br, decad.Exact, 100)
	require.Equal(t, decad.Sound, br.Status)
	require.True(t, report.Trustworthy())
}

func TestWallWedgePrismNoWall(t *testing.T) {
	// The equilateral wedge prism of verification §6: side pairs at 60°,
	// caps at 90° to them, and the two parallel caps beyond any inscribed
	// ball's reach (the cross-section's inradius is 8.66 mm, the caps 80 mm
	// apart). No spanning ball anywhere: MinWallThickness nil is the PROVEN
	// determination that no wall exists — nothing for the tool to violate.
	s, p := polygonSketch(t, [][2]float64{{0, 0}, {30, 0}, {15, 15 * math.Sqrt(3)}})
	doc := decad.New()
	_, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(80), Dir: decad.Along})
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	br := report.Bodies[0]
	require.Nil(t, br.MinWallThickness)
	require.Equal(t, decad.Sound, br.Status)
	require.True(t, report.Trustworthy())
}

func TestWallConeNoWall(t *testing.T) {
	// The revolved tetrahedron-analog: a solid cone whose apex angle is 60°
	// and whose base meets the wall at 60° — every skin meets its neighbours
	// past the allowance and opposes none. Proven no wall, Sound.
	s, p := polygonSketch(t, [][2]float64{{0, 0}, {30, 0}, {30, 30 * math.Tan(30*math.Pi/180)}})
	doc := decad.New()
	_, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	br := report.Bodies[0]
	require.Nil(t, br.MinWallThickness)
	require.Equal(t, decad.Suspect, br.Status)
	require.False(t, report.Trustworthy())
}

func TestWallAnnularPrism(t *testing.T) {
	// A hole-wall annulus: concentric R20/R15 circles extruded 30 mm. The
	// ball between the hole wall and the outer wall reads the 5 mm ring
	// thickness — met against a 1 mm tool, proven thin against a 6 mm one.
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	require.NoError(t, err)
	c := s.CreatePoint(0, 0)
	s.Fix(c)
	s.CreateCircle(c, 20)
	s.CreateCircle(c, 15)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			prof = p
		}
	}
	require.NotNil(t, prof)

	doc := decad.New()
	_, err = doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(30), Dir: decad.Along})
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	requireWall(t, report.Bodies[0], decad.Exact, 5)
	require.Equal(t, decad.Sound, report.Status)

	report, err = doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(6)))
	require.NoError(t, err)
	requireWall(t, report.Bodies[0], decad.Exact, 5)
	require.Equal(t, decad.Violating, report.Status)
}

func TestWallDraftAllowanceBoundary(t *testing.T) {
	// A wall drafted at exactly the allowance spans — within is inclusive
	// (verification §6). A trapezoid blade 1 mm at the base whose skins
	// lean 7.5° each (dihedral exactly 15°): at the default allowance the
	// infimum is the base-tangent spanning ball, diameter
	// cos β / (1 − sin β) ≈ 1.1403 mm, proven thin against a 1.2 mm tool.
	// One degree under the dihedral the skins are edges, the parallel
	// base/top pair is beyond any inscribed ball's reach, and the blade has
	// no wall at all.
	beta := 7.5 * math.Pi / 180
	top := 0.5 + 20*math.Tan(beta)
	s, p := polygonSketch(t, [][2]float64{{-0.5, 0}, {0.5, 0}, {top, 20}, {-top, 20}})
	doc := decad.New()
	_, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(30), Dir: decad.Along})
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1.2)))
	require.NoError(t, err)
	requireWall(t, report.Bodies[0], decad.Approximate, math.Cos(beta)/(1-math.Sin(beta)))
	require.Equal(t, decad.Violating, report.Status)

	report, err = doc.Verify(t.Context(),
		decad.WithMinWallThickness(units.Millimeters(1.2), decad.WithDraftAllowance(units.Degrees(14))))
	require.NoError(t, err)
	require.Nil(t, report.Bodies[0].MinWallThickness)
	require.Equal(t, decad.Sound, report.Status)
}

func TestWallKnifeEdgeExactZero(t *testing.T) {
	// A circular-segment prism: the chord meets the arc at a 10° tangent-
	// chord angle — a wedge within the allowance, a wall ground to nothing.
	// The reading is a genuine Exact 0 on a proven solid, and [0, 0] sits
	// below any real tool: Violating at any coarseness (the proven-thin arm
	// of the interval rule).
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	require.NoError(t, err)
	half := 20 * math.Sin(10*math.Pi/180)
	d := 20 * math.Cos(10*math.Pi/180)
	a := s.CreatePoint(-half, d)
	b := s.CreatePoint(half, d)
	c := s.CreatePoint(0, 0)
	s.Fix(c)
	s.CreateLine(a, b)
	s.CreateArc(c, b, a)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)

	doc := decad.New()
	_, err = doc.Extrude(s, profiles[0], decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(0.001)))
	require.NoError(t, err)
	br := report.Bodies[0]
	requireWall(t, br, decad.Exact, 0)
	require.Equal(t, decad.Violating, br.Status)
	require.False(t, report.Trustworthy())
}

func TestWallPartialRevolve(t *testing.T) {
	// A quarter-turn annular ring: the caps sit 90° apart — far past the
	// allowance — and the ring's own cross-section reads its 10 mm walls.
	s, p := annularSketch(t)
	doc := decad.New()
	_, err := doc.Revolve(s, p, uAxis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	require.NoError(t, err)
	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	requireWall(t, report.Bodies[0], decad.Exact, 10)
	require.Equal(t, decad.Suspect, report.Status)
}

func TestVerifyWallCancellationStopsCandidateWork(t *testing.T) {
	const sides = 40
	pts := make([][2]float64, sides)
	for i := range pts {
		th := 2 * math.Pi * float64(i) / sides
		pts[i] = [2]float64{20 * math.Cos(th), 20 * math.Sin(th)}
	}
	s, p := polygonSketch(t, pts)
	doc := decad.New()
	_, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	before := snapshotDocument(t, doc)
	ctx := newCancelAfterContext(t.Context(), 5)

	report, err := doc.Verify(ctx, decad.WithMinWallThickness(units.Millimeters(1)))

	require.ErrorIs(t, err, context.Canceled)
	require.Nil(t, report)
	requireDocumentUnchanged(t, doc, before)
}

func TestUndercutNearAntiparallelPull(t *testing.T) {
	// The antiparallel carve-out is EXACT: a pull tilted by 1e-5 hooks
	// under the base ever so slightly, so the base face opposes — along
	// with the one side face leaning against the tilt — and the body is
	// Violating with exactly those two faces listed.
	doc := rectPrism(t, 100, 60, 8)
	report, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(1e-5, 0, 1)))
	require.NoError(t, err)
	br := report.Bodies[0]
	require.Len(t, br.Undercuts, 2)
	require.Equal(t, decad.Violating, br.Status)
}

// TestUndercutExactlyPerpendicularWallIsNotOpposed is fu155's own repro
// (docs/verification-design.md §6): a square whose edge tangents are exactly
// (3,9), (-9,3), (-3,-9), (9,-3), extruded and pulled along (3,9,0), carries
// one wall EXACTLY perpendicular to the pull (its outward normal dotted with
// the unit pull is exactly 0) and one wall EXACTLY antiparallel to it
// (dot exactly -1). Before the fix both were listed as proven undercuts, read
// off the raw float components -4.6811112914356013e-17 and
// -0.99999999999999989 — neither of which is 0 or -1, but both fell inside
// opposesPull's open interval. The exact answer is the empty proven all-clear.
func TestUndercutExactlyPerpendicularWallIsNotOpposed(t *testing.T) {
	s, p := polygonSketch(t, [][2]float64{{0, 0}, {3, 9}, {-6, 12}, {-9, 3}})
	doc := decad.New()
	_, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	report, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(3, 9, 0)))
	require.NoError(t, err)
	br := report.Bodies[0]
	require.NotNil(t, br.Undercuts)
	require.Empty(t, br.Undercuts)
	require.Equal(t, decad.Sound, br.Status)

	unit, ok := r3.NewVec(3, 9, 0).Normalize()
	require.True(t, ok)
	var sawPerp, sawAnti bool
	for _, f := range br.Body.Faces() {
		if f.Surface().Kind() != decad.KindPlane {
			continue
		}
		n, err := f.NormalAt(f.Loops()[0].CoEdges()[0].Start().Position().Value)
		require.NoError(t, err)
		dot := n.Value.Dot(unit)
		switch {
		case math.Abs(dot) < 1e-9:
			sawPerp = true
			require.InDelta(t, 0, dot, 1e-15)
		case math.Abs(dot+1) < 1e-9:
			sawAnti = true
			require.InDelta(t, -1, dot, 1e-15)
		}
	}
	require.True(t, sawPerp, "the square must carry a wall exactly perpendicular to the pull")
	require.True(t, sawAnti, "the square must carry a wall exactly antiparallel to the pull")
}

// TestUndercutUnseparablePullIsUndecided proves the new reader answers
// undecided instead of guessing on a receiver's own circular wall, when the
// wall's outward-normal component against the pull genuinely straddles zero
// within the reader's own trig enclosure (survey_undercut.go's
// circularNormalRange) rather than merely landing near it in float64.
//
// The body is a narrow circular-arc wall (radius 10 mm, a 0.1-radian sweep at
// a generic — not a "nice" fraction of π — start angle) closed by one straight
// chord. The pull's in-plane components are chosen as a large rational
// continued-fraction convergent of the arc's own exact cosine and sine at its
// start angle: the residual between the pull and the arc's tangent there is
// many orders of magnitude below the width radSinCosInterval's own certified
// enclosure carries at that angle, so the enclosure cannot separate the
// wall's true (exactly zero) component from a genuine straddle. Reproduced
// through a small continued-fraction search (recorded as this test's own
// derivation, not asserted): du, dv below satisfy
// |dv*cos(th0) - du*sin(th0)| ~ 1e-16 while the wall's own trig enclosure at
// th0 is only accurate to ~1e-13 — an enclosure width larger than the true
// value it is asked to separate from zero.
//
// Isolating this to the WHOLE body (an empty Undercuts and a Suspect status)
// is not reachable with any two-wall closed profile: the same boundary that
// carries the near-zero-component arc must close on the far side too, and
// straight walls are decided EXACTLY, with no undecided outcome available to
// them (decideRationalComponent, survey_undercut.go) — so the chord is a
// second, independently and correctly proven undercut. That is not a defect;
// it is the same reject-only rule applied to a wall the reader CAN decide.
// The assertions below are what the fix actually proves: the ARC's own
// verdict is undecided — neither listed nor cleared — not the coarser claim
// that nothing else in the body opposes.
func TestUndercutUnseparablePullIsUndecided(t *testing.T) {
	const r, th0, delta = 10.0, 0.91, 0.1
	th1 := th0 + delta
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	require.NoError(t, err)
	pA := s.CreatePoint(r*math.Cos(th0), r*math.Sin(th0))
	pB := s.CreatePoint(r*math.Cos(th1), r*math.Sin(th1))
	pC := s.CreatePoint(0, 0)
	s.CreateArc(pC, pA, pB)
	s.CreateLine(pB, pA)
	s.Fix(pA)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	var arc *decad.Face
	for _, f := range body.Faces() {
		if f.Surface().Kind() == decad.KindCylinder {
			arc = f
		}
	}
	require.NotNil(t, arc, "the profile must build one circular wall")

	// du, dv: a continued-fraction convergent of (cos(th0), sin(th0)) large
	// enough that dv*cos(th0) - du*sin(th0) sits far inside the reader's own
	// trig enclosure width at th0.
	const du, dv = -2559100094135641.0, 1989397549793721.0
	report, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(du, dv, 0)))
	require.NoError(t, err)
	require.Len(t, report.Bodies, 1)
	br := report.Bodies[0]

	require.NotContains(t, br.Undercuts, arc, "the arc's own component is genuinely undecided, not a proven undercut")
	require.True(t, hasDiagnostic(report, decad.DiagUndecidedUndercut))
}

func TestWallReflexSweep(t *testing.T) {
	// A 270° sector of a length-7, radius-8 solid cylinder. The sweep is
	// reflex, so a mid-sweep ball's clearance to each cap HALF-plane is its
	// own axis distance (the perpendicular foot leaves the half-plane and
	// the nearest cap point is the axis edge): the length-spanning ball of
	// radius 3.5 at meridian radius 4 is legal — hand-checked, its angular
	// footprint [74°, 196°] sits inside the sweep — and the flats' 7 mm is
	// a real wall. A rule that kept shrinking the clearance past 90°
	// (ρ·sin(Δφ/2) ≈ 2.83 < 3.5) would erase it.
	//
	// The reading is Approximate, not Exact: a partial sweep's wedge factor is
	// a SINE, and the spanning candidates the wedge and the Apollonius triples
	// produce carry that sine's proven error plus their own Cramer arithmetic.
	// The interval still contains the true 7 mm, which is what the test pins.
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 7, 8)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	doc := decad.New()
	_, err = doc.Revolve(s, s.Profiles()[0], uAxis, decad.AngleExtent{A: units.Degrees(270), Dir: decad.Along})
	require.NoError(t, err)
	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	requireWall(t, report.Bodies[0], decad.Approximate, 7)
	requireWallBoundContains(t, report.Bodies[0], 7, 1e-9)
	require.Equal(t, decad.Suspect, report.Status)
}

// requireWallBoundContains asserts that a wall reading's published interval is
// strictly positive, no wider than maxBound, and actually contains the truth
// the fixture's own geometry states. A bound that failed to contain its own
// value is exactly the defect this guard exists for.
func requireWallBoundContains(t *testing.T, br *decad.BodyReport, truth, maxBound float64) {
	t.Helper()
	require.NotNil(t, br.MinWallThickness)
	value, err := br.MinWallThickness.Value.In(units.Millimeter)
	require.NoError(t, err)
	bound, err := br.MinWallThickness.Bound.In(units.Millimeter)
	require.NoError(t, err)
	require.Greater(t, bound, 0.0)
	require.LessOrEqual(t, bound, maxBound)
	require.LessOrEqual(t, value-bound, truth)
	require.GreaterOrEqual(t, value+bound, truth)
}

func TestWallSectorTooTightForItsFlats(t *testing.T) {
	// The same sector one size longer (length 10, radius 8): the
	// length-spanning ball of radius 5 needs meridian radius ≥ 5 to clear
	// the caps but ≤ 3 to stay inside the skin — infeasible — and no other
	// pair opposes within the allowance. §6's wall is read by the ball that
	// spans it, so this body has NO wall: nil, a proven absence, Sound.
	s, p := solidSketch(t)
	doc := decad.New()
	_, err := doc.Revolve(s, p, uAxis, decad.AngleExtent{A: units.Degrees(270), Dir: decad.Along})
	require.NoError(t, err)
	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	require.Nil(t, report.Bodies[0].MinWallThickness)
	require.Equal(t, decad.Suspect, report.Status)
}

func TestWallThinPieWedge(t *testing.T) {
	// A 10° pie wedge of a solid cylinder: its two caps meet along the axis
	// at the sweep angle itself — a dihedral within the allowance, a wall
	// ground to zero at every size. Exact 0, Violating against any tool.
	s, p := solidSketch(t)
	doc := decad.New()
	_, err := doc.Revolve(s, p, uAxis, decad.AngleExtent{A: units.Degrees(10), Dir: decad.Along})
	require.NoError(t, err)
	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	requireWall(t, report.Bodies[0], decad.Exact, 0)
	require.Equal(t, decad.Violating, report.Status)
}

func TestUndercutsPrismClear(t *testing.T) {
	// A straight prism under a +z pull: every side wall is exactly
	// perpendicular — not opposed, the pull slides along it — and the caps
	// separate. Undercuts is the empty PROVEN all-clear, an answer, and the
	// report is Sound.
	doc := rectPrism(t, 100, 60, 10)
	report, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(0, 0, 1)))
	require.NoError(t, err)
	br := report.Bodies[0]
	require.NotNil(t, br.Undercuts)
	require.Empty(t, br.Undercuts)
	require.Equal(t, decad.Sound, br.Status)
	require.True(t, report.Trustworthy())
}

func TestUndercutsTiltedPull(t *testing.T) {
	// The same prism under a tilted pull: the side facing against the pull
	// and the bottom cap both carry normals with a component against it —
	// proven undercuts, Violating.
	doc := rectPrism(t, 100, 60, 10)
	report, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(1, 0, 1)))
	require.NoError(t, err)
	br := report.Bodies[0]
	require.Len(t, br.Undercuts, 2)
	for _, f := range br.Undercuts {
		require.Equal(t, decad.KindPlane, f.Surface().Kind())
	}
	require.Equal(t, decad.Violating, br.Status)
	require.False(t, report.Trustworthy())
}

func TestUndercutsSphereListed(t *testing.T) {
	// A revolved sphere under any pull: its single face sweeps every normal
	// direction, so it provenly opposes past its equator and is listed —
	// the doc's own spherical case. One face, Violating.
	s, p := semicircleSketch(t)
	doc := decad.New()
	body, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(0, 0, 1)))
	require.NoError(t, err)
	br := report.Bodies[0]
	require.Len(t, br.Undercuts, 1)
	require.Equal(t, decad.KindSphere, br.Undercuts[0].Surface().Kind())
	require.Same(t, body.Faces()[0], br.Undercuts[0])
	require.Equal(t, decad.Violating, br.Status)
}

func TestUndercutsVerticalHoleClear(t *testing.T) {
	// A vertical hole wall under a +z pull carries only horizontal normals:
	// exactly perpendicular everywhere, provenly clear.
	doc := holePlate(t)
	report, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(0, 0, 1)))
	require.NoError(t, err)
	br := report.Bodies[0]
	require.NotNil(t, br.Undercuts)
	require.Empty(t, br.Undercuts)
	require.Equal(t, decad.Sound, br.Status)
}

// holePlate extrudes a 100×60×8 plate with a Ø20 through hole at (70, 30).
func holePlate(t *testing.T) *decad.Document {
	t.Helper()
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
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
	doc := decad.New()
	_, err = doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	require.NoError(t, err)
	return doc
}

func TestMinRadiusHolePlate(t *testing.T) {
	// The hole wall is the plate's one concave face: MinRadius reads its
	// 10 mm radius — a measurement, never compared to a spec, so the report
	// stays Sound.
	doc := holePlate(t)
	report, err := doc.Verify(t.Context(), decad.WithMinRadius())
	require.NoError(t, err)
	br := report.Bodies[0]
	require.NotNil(t, br.MinRadius)
	require.Equal(t, decad.Exact, br.MinRadius.Exactness)
	require.True(t, br.MinRadius.Value.Equal(units.Millimeters(10), 1e-9))
	require.Equal(t, decad.Sound, br.Status)
	require.True(t, report.Trustworthy())
}

func TestMinRadiusPlainBlockNil(t *testing.T) {
	// An all-convex block has no concave feature: nil is the proven best
	// case for any endmill, and the report is Sound.
	doc := rectPrism(t, 100, 60, 10)
	report, err := doc.Verify(t.Context(), decad.WithMinRadius())
	require.NoError(t, err)
	br := report.Bodies[0]
	require.Nil(t, br.MinRadius)
	require.Equal(t, decad.Sound, br.Status)
	require.True(t, report.Trustworthy())
}

func TestMinRadiusDonutWaist(t *testing.T) {
	// A solid torus: the tube itself is convex, but around the axis the
	// donut's waist curves away from the material with radius M − m.
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	require.NoError(t, err)
	c := s.CreatePoint(0, 30)
	s.Fix(c)
	s.CreateCircle(c, 5)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	doc := decad.New()
	_, err = doc.Revolve(s, s.Profiles()[0], uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithMinRadius())
	require.NoError(t, err)
	br := report.Bodies[0]
	require.NotNil(t, br.MinRadius)
	require.True(t, br.MinRadius.Value.Equal(units.Millimeters(25), 1e-9))
	require.Equal(t, decad.Suspect, br.Status)
}

func TestWallHolePlateReadsThickness(t *testing.T) {
	// The plate-with-hole's thinnest wall is the 8 mm plate itself: the
	// hole clears every side by 20 mm, so the parallel skins win.
	doc := holePlate(t)
	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	requireWall(t, report.Bodies[0], decad.Exact, 8)
	require.Equal(t, decad.Sound, report.Status)
}

func TestSurveysAnsweredTogether(t *testing.T) {
	// All three asked at once on a sound plate: every question is answered
	// — the old always-Suspect staging is gone — and everything is in spec,
	// so the report reads Sound and Trustworthy.
	doc := rectPrism(t, 100, 60, 10)
	report, err := doc.Verify(t.Context(),
		decad.WithMinWallThickness(units.Millimeters(1)),
		decad.WithPullDirection(r3.NewVec(0, 0, 1)),
		decad.WithMinRadius())
	require.NoError(t, err)

	br := report.Bodies[0]
	requireWall(t, br, decad.Exact, 10)
	require.NotNil(t, br.Undercuts)
	require.Empty(t, br.Undercuts)
	require.Nil(t, br.MinRadius)
	require.Equal(t, decad.Exact, br.Exactness)
	require.Equal(t, decad.Sound, br.Status)
	require.Equal(t, decad.Sound, report.Status)
	require.True(t, report.Trustworthy())
	require.Empty(t, report.Diagnostics, `supported analytic surveys emit no refusal diagnostic`)
}

func TestWallSolidCylinder(t *testing.T) {
	// A solid cylinder R8 × 10 long (full revolve): the caps span across
	// the axis at 10 mm; the 16 mm diametral ball is starved by them.
	//
	// The winning candidate's whole chain is exactly representable, so the
	// reading is Exact: nothing along it rounds, and a zero-bound operand
	// keeps its exactness through boundedSqrt.
	s, p := solidSketch(t)
	doc := decad.New()
	_, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	br := report.Bodies[0]
	requireWall(t, br, decad.Exact, 10)
	bound, err := br.MinWallThickness.Bound.In(units.Millimeter)
	require.NoError(t, err)
	require.Equal(t, 0.0, bound)
	// The report still reads Suspect, and not for the wall: the full
	// revolve's own volume and area bounds are beyond the relative
	// tolerance, which is a separate reading from this one.
	require.Equal(t, decad.Suspect, report.Status)
}

// TestMinRadiusAnnularRevolveStaysExact pins the other public reading whose
// whole chain is exactly representable: the annular cylinder's bore. Its
// meridian is a straight wall whose tangent reaches boundedSqrt through
// boundedHypot as two exact leaves, and the parallel circle's radius is the
// recorded 5 mm, so the survey publishes it Exact with a zero bound.
func TestMinRadiusAnnularRevolveStaysExact(t *testing.T) {
	s, p := annularSketch(t)
	doc := decad.New()
	_, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithMinRadius())
	require.NoError(t, err)
	br := report.Bodies[0]
	require.NotNil(t, br.MinRadius)
	require.Equal(t, decad.Exact, br.MinRadius.Exactness)
	require.True(t, br.MinRadius.Value.Equal(units.Millimeters(5), 1e-9),
		`want 5 mm, got %s`, br.MinRadius.Value)
	bound, err := br.MinRadius.Bound.In(units.Millimeter)
	require.NoError(t, err)
	require.Equal(t, 0.0, bound)
}

func TestWallNarrowConeTaperZero(t *testing.T) {
	// A cone whose apex angle (10°) sits within the allowance is a tapered
	// wall run out to nothing: the balls thin to the apex with contacts
	// inside the allowance, and the closure reads a genuine Exact 0.
	r := 30 * math.Tan(5*math.Pi/180)
	s, p := polygonSketch(t, [][2]float64{{0, 0}, {30, 0}, {30, r}})
	doc := decad.New()
	_, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	requireWall(t, report.Bodies[0], decad.Exact, 0)
	require.Equal(t, decad.Violating, report.Status)
}

func TestWallPlacedBodyReadsThePart(t *testing.T) {
	// The reading is the part's, not the pose's: a rotated, far-translated
	// copy of the thin plate reads the same 0.5 mm wall.
	doc := rectPrism(t, 10, 10, 0.5)
	body := doc.Bodies()[0]
	rot, err := r3.Rotation(r3.NewVec(1, 1, 0), units.Degrees(40))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(500, -200, 90))
	require.NoError(t, err)
	motion, err := rot.Then(shift)
	require.NoError(t, err)
	_, err = body.Placed(motion)
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	requireWall(t, report.Bodies[0], decad.Exact, 0.5)
	require.Equal(t, decad.Violating, report.Status)
}

func TestSurveysSkippedOnUnaskedOptions(t *testing.T) {
	// An option left off states no spec: no reading, no verdict.
	doc := rectPrism(t, 100, 60, 10)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	br := report.Bodies[0]
	require.Nil(t, br.MinWallThickness)
	require.Nil(t, br.Undercuts)
	require.Nil(t, br.MinRadius)
	require.Equal(t, decad.Sound, report.Status)
}

func TestWallReadingBoundEnclosesCurvedWeb(t *testing.T) {
	// The 100×60×20 mm plate with two r=1 holes at (48.5, 28.5) and
	// (51.5, 31.5): the web between them is hypot(3,3) − 2 = 3√2 − 2, which no
	// float64 holds exactly. The reading carries the arcArcCands centerline
	// candidate's own proven bound instead of asserting Exact.
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(48.5, 28.5), 1)
	s.CreateCircle(s.CreatePoint(51.5, 31.5), 1)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 2 {
			prof = p
		}
	}
	require.NotNil(t, prof)

	doc := decad.New()
	_, err = doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	br := report.Bodies[0]
	require.NotNil(t, br.MinWallThickness)
	require.Equal(t, decad.Approximate, br.MinWallThickness.Exactness)
	value, err := br.MinWallThickness.Value.In(units.Millimeter)
	require.NoError(t, err)
	bound, err := br.MinWallThickness.Bound.In(units.Millimeter)
	require.NoError(t, err)
	require.Greater(t, bound, 0.0)
	require.LessOrEqual(t, bound, 1e-9)
	const truth = 2.2426406871192851464 // 3√2 − 2
	require.LessOrEqual(t, value-bound, truth)
	require.GreaterOrEqual(t, value+bound, truth)
	require.True(t, report.Trustworthy())
}

func TestWallReadingBoundIsOnTheSpanningDiameter(t *testing.T) {
	// The same curved-web reading as above, on a fixture whose candidate bound
	// is large enough to tell a radius bound from a diameter one. The plate is
	// 100 mm thick with two equal holes of radius 132665.06800135397 mm whose
	// centres sit (264148.72380984796, 25043.885926669256) apart, so the web
	// between them is ~3.1346451310243 mm while the coordinates feeding
	// arcArcCands' own division are ~1e5 — the bound comes out near 3e-11,
	// five orders of magnitude past the reading's own float64 rounding.
	//
	// survey2d's candidates bound their RADIUS and the reading is the spanning
	// DIAMETER, so a bound published undoubled misses the truth here by a
	// factor of about 1.48: the interval below is the whole point of the test.
	const (
		cx = 264148.72380984796
		cy = 25043.885926669256
		r  = 132665.06800135397
	)
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-4e5, -4e5, 12e5, 12e5)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(0, 0), r)
	s.CreateCircle(s.CreatePoint(cx, cy), r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 2 {
			prof = p
		}
	}
	require.NotNil(t, prof)

	doc := decad.New()
	_, err = doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(100), Dir: decad.Along})
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	br := report.Bodies[0]
	require.NotNil(t, br.MinWallThickness)
	require.Equal(t, decad.Approximate, br.MinWallThickness.Exactness)
	value, err := br.MinWallThickness.Value.In(units.Millimeter)
	require.NoError(t, err)
	bound, err := br.MinWallThickness.Bound.In(units.Millimeter)
	require.NoError(t, err)
	require.Greater(t, bound, 0.0)

	// hypot(cx, cy) − 2r, evaluated at 300 bits: every input is an exact
	// float64, so the only rounding is the final one, far below the bound.
	truth := webGapTruth(cx, cy, r)
	require.LessOrEqual(t, value-bound, truth)
	require.GreaterOrEqual(t, value+bound, truth)
}

// webGapTruth is the exact centre-distance-minus-two-radii gap between two
// equal circles, evaluated at 300 bits so the comparison above is against the
// geometry rather than against another float64 evaluation of it.
func webGapTruth(cx, cy, r float64) float64 {
	const prec = 300
	f := func(v float64) *big.Float { return new(big.Float).SetPrec(prec).SetFloat64(v) }
	x, y, rr := f(cx), f(cy), f(r)
	d2 := new(big.Float).SetPrec(prec).Add(
		new(big.Float).SetPrec(prec).Mul(x, x),
		new(big.Float).SetPrec(prec).Mul(y, y))
	gap := new(big.Float).SetPrec(prec).Sub(
		new(big.Float).SetPrec(prec).Sqrt(d2),
		new(big.Float).SetPrec(prec).Add(rr, rr))
	out, _ := gap.Float64()
	return out
}

func TestWallReadingExactHeightArm(t *testing.T) {
	// The 10×10×0.5 mm plate: the height arm's two axial displacements are
	// both zero (a caller-stated sweep) and 0.5 is exactly representable, so
	// the reading stays Exact with Bound exactly zero — the test that stops a
	// blanket Approximate.
	doc := rectPrism(t, 10, 10, 0.5)
	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	br := report.Bodies[0]
	require.NotNil(t, br.MinWallThickness)
	require.Equal(t, decad.Exact, br.MinWallThickness.Exactness)
	boundMM, err := br.MinWallThickness.Bound.In(units.Millimeter)
	require.NoError(t, err)
	require.Equal(t, 0.0, boundMM)
	value, err := br.MinWallThickness.Value.In(units.Millimeter)
	require.NoError(t, err)
	require.Equal(t, 0.5, value)
	require.Equal(t, decad.Violating, br.Status)
}

func TestWallHeightCarriesAxialDelta(t *testing.T) {
	// A 10×10 mm plate extruded units.Inches(0.1): the sweep level's own unit
	// conversion carries a nonzero displacement (docs/evaluator-design.md §5),
	// which prismWall's height arm composes into the reading instead of
	// dropping. Without that composition the reported height with an
	// ulp-scale bound fails this by construction.
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	doc := decad.New()
	_, err = doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Inches(0.1), Dir: decad.Along})
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	br := report.Bodies[0]
	require.NotNil(t, br.MinWallThickness)
	require.Equal(t, decad.Approximate, br.MinWallThickness.Exactness)
	value, err := br.MinWallThickness.Value.In(units.Millimeter)
	require.NoError(t, err)
	bound, err := br.MinWallThickness.Bound.In(units.Millimeter)
	require.NoError(t, err)
	require.GreaterOrEqual(t, value+bound, 2.5400000000000001410)
}

func TestMinRadiusArcRadiusBound(t *testing.T) {
	// A prism whose one concave feature is dSectionProfile's √2 ArcSeg hole,
	// cut as a hole so the material-outside convention makes its arc concave.
	// The published interval must contain the truth; a millimetre CircleSeg
	// hole on the same fixture family (holePlate) stays Exact with Bound zero.
	s, prof := dSectionProfile(t)
	doc := decad.New()
	_, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithMinRadius())
	require.NoError(t, err)
	br := report.Bodies[0]
	require.NotNil(t, br.MinRadius)
	require.Equal(t, decad.Approximate, br.MinRadius.Exactness)
	value, err := br.MinRadius.Value.In(units.Millimeter)
	require.NoError(t, err)
	bound, err := br.MinRadius.Bound.In(units.Millimeter)
	require.NoError(t, err)
	const truth = 1.4142135623730950488 // √2
	require.LessOrEqual(t, value-bound, truth)
	require.GreaterOrEqual(t, value+bound, truth)

	holeReport, err := holePlate(t).Verify(t.Context(), decad.WithMinRadius())
	require.NoError(t, err)
	holeBR := holeReport.Bodies[0]
	require.NotNil(t, holeBR.MinRadius)
	require.Equal(t, decad.Exact, holeBR.MinRadius.Exactness)
	holeBoundMM, err := holeBR.MinRadius.Bound.In(units.Millimeter)
	require.NoError(t, err)
	require.Equal(t, 0.0, holeBoundMM)
}

// dSectionProfile is the D-shaped ArcSeg fixture both radius-bound tests
// share: a 20×20 outline holding a chord-plus-arc hole whose centre (10, 9) is
// one unit from its start (9, 10) in each of u and v, so the arc's TRUE radius
// is √2 while the walk holds the math.Hypot evaluation of it.
func dSectionProfile(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 20, 20)
	s.Fix(rect.A)
	x := s.CreatePoint(9, 10)
	y := s.CreatePoint(11, 10)
	o := s.CreatePoint(10, 9)
	s.CreateLine(x, y)
	s.CreateArc(o, y, x)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			prof = p
		}
	}
	require.NotNil(t, prof)
	return s, prof
}

// requireContains asserts that truth lies in [value − bound, value + bound],
// comparing over big.Rat so neither endpoint is formed by a float subtraction
// whose own rounding could swamp the bound under test.
func requireContains(t *testing.T, truth *big.Rat, value, bound float64) {
	t.Helper()
	v := new(big.Rat).SetFloat64(value)
	b := new(big.Rat).SetFloat64(bound)
	require.NotNil(t, v)
	require.NotNil(t, b)
	require.LessOrEqual(t, new(big.Rat).Sub(v, b).Cmp(truth), 0,
		`the published interval must reach down to the truth`)
	require.GreaterOrEqual(t, new(big.Rat).Add(v, b).Cmp(truth), 0,
		`the published interval must reach up to the truth`)
}

// TestWallConcentricRingCarriesRadiusDifferenceBound pins the concentric
// annulus candidate: its half-width is |Ra − Rb|/2, and that difference rounds
// whenever the two radii are far apart in magnitude, so the reading is never
// Exact merely because both radii are recorded numbers. R20 with an r0.03 bore
// is the ordinary-numbers case — 20 − 0.03 is not representable, and a zero
// bound publishes an interval that excludes the true ring thickness.
func TestWallConcentricRingCarriesRadiusDifferenceBound(t *testing.T) {
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	require.NoError(t, err)
	c := s.CreatePoint(0, 0)
	s.Fix(c)
	s.CreateCircle(c, 20)
	s.CreateCircle(c, 0.03)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			prof = p
		}
	}
	require.NotNil(t, prof)

	doc := decad.New()
	_, err = doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(500), Dir: decad.Along})
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	br := report.Bodies[0]
	require.NotNil(t, br.MinWallThickness)
	value, err := br.MinWallThickness.Value.In(units.Millimeter)
	require.NoError(t, err)
	bound, err := br.MinWallThickness.Bound.In(units.Millimeter)
	require.NoError(t, err)

	// The truth is the exact difference of the two RECORDED radii, formed over
	// big.Rat: the float 20 − 0.03 the kernel holds is a hair below it.
	truth := new(big.Rat).Sub(new(big.Rat).SetFloat64(20), new(big.Rat).SetFloat64(0.03))
	require.NotEqual(t, 0, new(big.Rat).SetFloat64(value).Cmp(truth),
		`the held difference must miss the truth, or this fixture proves nothing`)
	require.Equal(t, decad.Approximate, br.MinWallThickness.Exactness)
	require.Greater(t, bound, 0.0)
	requireContains(t, truth, value, bound)
}

// TestRevolveMinRadiusArcRadiusBound is TestMinRadiusArcRadiusBound's revolve
// twin: the same √2 ArcSeg meridian, revolved rather than extruded. The
// concave-meridian arm reads the walk's own radius, which for an ArcSeg is a
// math.Hypot evaluation, so it publishes the walk's bound on that radius
// rather than claiming the record stated it.
func TestRevolveMinRadiusArcRadiusBound(t *testing.T) {
	s, prof := dSectionProfile(t)
	doc := decad.New()
	_, err := doc.Revolve(s, prof, uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithMinRadius())
	require.NoError(t, err)
	br := report.Bodies[0]
	require.NotNil(t, br.MinRadius)
	require.Equal(t, decad.Approximate, br.MinRadius.Exactness)
	value, err := br.MinRadius.Value.In(units.Millimeter)
	require.NoError(t, err)
	bound, err := br.MinRadius.Bound.In(units.Millimeter)
	require.NoError(t, err)
	require.Greater(t, bound, 0.0)

	// √2 to 200 bits, so the comparison is against the truth and not against
	// another float64 evaluation of the same square root.
	const prec = 200
	sqrt2 := new(big.Float).SetPrec(prec).Sqrt(new(big.Float).SetPrec(prec).SetInt64(2))
	truth, _ := sqrt2.Rat(nil)
	requireContains(t, truth, value, bound)
}

// The rival-candidate fixtures below pin docs/payload-verification-design.md
// §9.2's aggregate: an arc whose TRUE radius is rivalArcU/rivalArcV's
// hypotenuse, beside a circle whose radius is a recorded millimetre number one
// ulp below the arc's held math.Hypot. The circle holds the smaller value, so a
// winner-only reduction publishes the circle's Exact zero bound — and the arc's
// truth lies BELOW that reading, outside the published interval.
const (
	rivalArcU    = 5.5507050921376759334
	rivalArcV    = 0.24258038266405496097
	rivalCircleR = 5.5560032633122675705
)

// rivalRadiusTruth is √(rivalArcU² + rivalArcV²) to 200 bits: the arc's own
// true radius, computed from the recorded coordinates rather than hardcoded,
// and far finer than any interval under test here.
func rivalRadiusTruth() *big.Rat {
	const prec = 200
	u := new(big.Float).SetPrec(prec).SetFloat64(rivalArcU)
	v := new(big.Float).SetPrec(prec).SetFloat64(rivalArcV)
	sum := new(big.Float).SetPrec(prec).Add(
		new(big.Float).SetPrec(prec).Mul(u, u),
		new(big.Float).SetPrec(prec).Mul(v, v),
	)
	truth, _ := new(big.Float).SetPrec(prec).Sqrt(sum).Rat(nil)
	return truth
}

// rivalRadiusPlate is the 40×40 mm plate holding the two rival concave
// candidates: a half-disc hole whose arc is centred exactly on the sketch
// origin with endpoints ±(rivalArcU, rivalArcV), and a plain circular hole of
// radius rivalCircleR clear of it.
func rivalRadiusPlate(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-20, -20, 20, 20)
	s.Fix(rect.A)
	o := s.CreatePoint(0, 0)
	p := s.CreatePoint(rivalArcU, rivalArcV)
	q := s.CreatePoint(-rivalArcU, -rivalArcV)
	s.CreateArc(o, p, q)
	s.CreateLine(q, p)
	s.CreateCircle(s.CreatePoint(12, 12), rivalCircleR)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, cand := range s.Profiles() {
		if len(cand.Holes) == 2 {
			prof = cand
		}
	}
	require.NotNil(t, prof, `the plate profile carries both holes`)
	return s, prof
}

// TestMinRadiusAggregatesLosingCandidate pins the prism site of the aggregate:
// the circle hole wins the comparison of held values, but the arc hole's own
// interval reaches lower, so the published interval must reach down past the
// arc's true radius. A winner-only reduction publishes the circle's Exact zero
// bound and excludes that truth.
func TestMinRadiusAggregatesLosingCandidate(t *testing.T) {
	s, prof := rivalRadiusPlate(t)
	doc := decad.New()
	_, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithMinRadius())
	require.NoError(t, err)
	br := report.Bodies[0]
	require.NotNil(t, br.MinRadius)
	value, err := br.MinRadius.Value.In(units.Millimeter)
	require.NoError(t, err)
	bound, err := br.MinRadius.Bound.In(units.Millimeter)
	require.NoError(t, err)
	truth := rivalRadiusTruth()
	t.Logf("min radius %.20g ± %.20g (%s); truth %s",
		value, bound, br.MinRadius.Exactness, new(big.Float).SetPrec(200).SetRat(truth).Text('g', 21))
	require.Less(t, new(big.Float).SetPrec(200).SetRat(truth).Cmp(big.NewFloat(rivalCircleR)), 0,
		`the arc's truth must sit below the circle's recorded radius, or this fixture proves nothing`)
	requireContains(t, truth, value, bound)
	require.Equal(t, decad.Approximate, br.MinRadius.Exactness,
		`an interval reaching past an inexact rival is not an exact reading`)
}

// TestMinRadiusExactCandidatesStayExact holds the other side of the aggregate
// at the prism site: two recorded CircleSeg holes are both exactly
// representable, so the interval collapses onto the smaller radius and the
// reading stays Exact with a zero bound.
func TestMinRadiusExactCandidatesStayExact(t *testing.T) {
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(30, 30), 10)
	s.CreateCircle(s.CreatePoint(70, 30), 7)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, cand := range s.Profiles() {
		if len(cand.Holes) == 2 {
			prof = cand
		}
	}
	require.NotNil(t, prof)

	doc := decad.New()
	_, err = doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	require.NoError(t, err)
	report, err := doc.Verify(t.Context(), decad.WithMinRadius())
	require.NoError(t, err)
	br := report.Bodies[0]
	require.NotNil(t, br.MinRadius)
	value, err := br.MinRadius.Value.In(units.Millimeter)
	require.NoError(t, err)
	bound, err := br.MinRadius.Bound.In(units.Millimeter)
	require.NoError(t, err)
	t.Logf("min radius %.20g ± %.20g (%s)", value, bound, br.MinRadius.Exactness)
	require.Equal(t, 7.0, value, `the tighter hole is the reading`)
	require.Equal(t, 0.0, bound)
	require.Equal(t, decad.Exact, br.MinRadius.Exactness)
}

// rivalRadiusMeridian is the revolve twin of rivalRadiusPlate: a meridian
// rectangle whose inner wall sits at exactly rivalCircleR from the axis — an
// exact parallel-circle candidate — around a half-disc meridian hole whose arc
// carries the same ±(rivalArcU, rivalArcV) endpoints, and so the same true
// radius one ulp below that wall.
func rivalRadiusMeridian(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, rivalCircleR, 30, 20)
	s.Fix(rect.A)
	o := s.CreatePoint(15, 12)
	p := s.CreatePoint(15+rivalArcU, 12+rivalArcV)
	q := s.CreatePoint(15-rivalArcU, 12-rivalArcV)
	s.CreateArc(o, p, q)
	s.CreateLine(q, p)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, cand := range s.Profiles() {
		if len(cand.Holes) == 1 {
			prof = cand
		}
	}
	require.NotNil(t, prof, `the meridian profile carries the half-disc hole`)
	return s, prof
}

// TestRevolveMinRadiusAggregatesLosingCandidate is the revolve site of the same
// rule: the bore wall's parallel circle wins on held value with an exact zero
// bound, while the meridian arc's own interval reaches below it. Both arms feed
// one aggregate, so the reading must reach down past the arc's true radius.
func TestRevolveMinRadiusAggregatesLosingCandidate(t *testing.T) {
	s, prof := rivalRadiusMeridian(t)
	doc := decad.New()
	_, err := doc.Revolve(s, prof, uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithMinRadius())
	require.NoError(t, err)
	br := report.Bodies[0]
	require.NotNil(t, br.MinRadius)
	value, err := br.MinRadius.Value.In(units.Millimeter)
	require.NoError(t, err)
	bound, err := br.MinRadius.Bound.In(units.Millimeter)
	require.NoError(t, err)
	truth := rivalRadiusTruth()
	t.Logf("min radius %.20g ± %.20g (%s); truth %s",
		value, bound, br.MinRadius.Exactness, new(big.Float).SetPrec(200).SetRat(truth).Text('g', 21))
	requireContains(t, truth, value, bound)
	require.Equal(t, decad.Approximate, br.MinRadius.Exactness,
		`an interval reaching past an inexact rival is not an exact reading`)
}

// TestRevolveMinRadiusExactCandidatesStayExact is the revolve twin of
// TestMinRadiusExactCandidatesStayExact: a stepped bore offers two
// parallel-circle candidates, both read off recorded coordinates through an
// exact unit tangent, so the aggregate publishes the tighter one Exact.
func TestRevolveMinRadiusExactCandidatesStayExact(t *testing.T) {
	s, p := polygonSketch(t, [][2]float64{{0, 5}, {10, 5}, {10, 8}, {20, 8}, {20, 20}, {0, 20}})
	doc := decad.New()
	_, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithMinRadius())
	require.NoError(t, err)
	br := report.Bodies[0]
	require.NotNil(t, br.MinRadius)
	value, err := br.MinRadius.Value.In(units.Millimeter)
	require.NoError(t, err)
	bound, err := br.MinRadius.Bound.In(units.Millimeter)
	require.NoError(t, err)
	t.Logf("min radius %.20g ± %.20g (%s)", value, bound, br.MinRadius.Exactness)
	require.Equal(t, 5.0, value, `the tighter bore step is the reading`)
	require.Equal(t, 0.0, bound)
	require.Equal(t, decad.Exact, br.MinRadius.Exactness)
}
