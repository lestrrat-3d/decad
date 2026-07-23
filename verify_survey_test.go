package decad_test

import (
	"math"
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

// requireWall asserts an Exact MinWallThickness reading in millimetres.
func requireWall(t *testing.T, br *decad.BodyReport, want float64) {
	t.Helper()
	require.NotNil(t, br.MinWallThickness)
	require.Equal(t, decad.Exact, br.MinWallThickness.Exactness)
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
	requireWall(t, br, 0.5)
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
	requireWall(t, br, 100)
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
	require.Equal(t, decad.Sound, br.Status)
	require.True(t, report.Trustworthy())
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
	requireWall(t, report.Bodies[0], 5)
	require.Equal(t, decad.Sound, report.Status)

	report, err = doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(6)))
	require.NoError(t, err)
	requireWall(t, report.Bodies[0], 5)
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
	requireWall(t, report.Bodies[0], math.Cos(beta)/(1-math.Sin(beta)))
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
	requireWall(t, br, 0)
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
	requireWall(t, report.Bodies[0], 10)
	require.Equal(t, decad.Sound, report.Status)
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

func TestWallReflexSweep(t *testing.T) {
	// A 270° sector of a length-7, radius-8 solid cylinder. The sweep is
	// reflex, so a mid-sweep ball's clearance to each cap HALF-plane is its
	// own axis distance (the perpendicular foot leaves the half-plane and
	// the nearest cap point is the axis edge): the length-spanning ball of
	// radius 3.5 at meridian radius 4 is legal — hand-checked, its angular
	// footprint [74°, 196°] sits inside the sweep — and the flats' 7 mm is
	// a real wall. A rule that kept shrinking the clearance past 90°
	// (ρ·sin(Δφ/2) ≈ 2.83 < 3.5) would erase it.
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
	requireWall(t, report.Bodies[0], 7)
	require.Equal(t, decad.Sound, report.Status)
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
	require.Equal(t, decad.Sound, report.Status)
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
	requireWall(t, report.Bodies[0], 0)
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
	require.Equal(t, decad.Sound, br.Status)
}

func TestWallHolePlateReadsThickness(t *testing.T) {
	// The plate-with-hole's thinnest wall is the 8 mm plate itself: the
	// hole clears every side by 20 mm, so the parallel skins win.
	doc := holePlate(t)
	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	requireWall(t, report.Bodies[0], 8)
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
	requireWall(t, br, 10)
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
	s, p := solidSketch(t)
	doc := decad.New()
	_, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	requireWall(t, report.Bodies[0], 10)
	require.Equal(t, decad.Sound, report.Status)
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
	requireWall(t, report.Bodies[0], 0)
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
	requireWall(t, report.Bodies[0], 0.5)
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
