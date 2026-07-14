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

// The worked examples of docs/clearance-design.md §4/§7 are this file's test
// oracle: the diagonal cube pair, the stacked cubes, the stop-built stack's
// coplanar contact, the coaxial peg-in-hole reading, the parallel-axis torus
// pair, and the PR 1 staging behavior — cone-involved pairs coarse, touching
// pairs without a certificate Suspect, nested pairs undecided.

// boxBody extrudes an axis-aligned rectangle into doc.
func boxBody(t *testing.T, doc *decad.Document, x0, y0, x1, y1, h float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// ballBody revolves a semicircle of radius r centered at the origin into a
// ball at the origin.
func ballBody(t *testing.T, doc *decad.Document, r float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(-r, 0)
	s.Fix(o)
	end := s.CreatePoint(r, 0)
	c := s.CreatePoint(0, 0)
	s.CreateLine(o, end)
	s.CreateArc(c, end, o)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Revolve(s, s.Profiles()[0], uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

// torusBody revolves a circle at (0, major) of radius minor about the u
// axis: a full torus about the world X axis centered at the origin.
func torusBody(t *testing.T, doc *decad.Document, major, minor float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, major)
	s.Fix(center)
	s.CreateCircle(center, minor)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Revolve(s, s.Profiles()[0], uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

// requireGap asserts the report carries exactly one clearance row with the
// given exact gap.
func requireExactGap(t *testing.T, report *decad.Report, mm float64) {
	t.Helper()
	require.Len(t, report.Clearances, 1)
	row := report.Clearances[0]
	require.Equal(t, decad.Exact, row.Gap.Exactness)
	require.InDelta(t, mm, row.Gap.Value.Mag(), 1e-9)
	require.Equal(t, 0.0, row.Gap.Bound.Mag())
}

func TestClearanceCubesDiagonalOffset(t *testing.T) {
	// The §7 worked pair: a 10 mm cube at the origin and one at x∈[13,23],
	// y∈[12,22] — the facing-face plateaus are discarded (the trims clear in
	// projection) and the minimum falls to two parallel vertical edges:
	// √13 ≈ 3.606 mm, Exact.
	doc := decad.New()
	boxBody(t, doc, 0, 0, 10, 10, 10)
	boxBody(t, doc, 13, 12, 23, 22, 10)
	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	require.True(t, report.Trustworthy())
	requireExactGap(t, report, math.Sqrt(13))
}

func TestClearanceStackedCubes(t *testing.T) {
	// The same cubes stacked 2 mm apart: the facing caps' trims overlap in
	// projection, so the face × face plateau carries the answer — 2 mm,
	// Exact (§7's second worked row). The second cube arrives via Placed, so
	// the pair also covers the placed-pose path.
	doc := decad.New()
	boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 0, 0, 10, 10, 10)
	shift, err := r3.Translation(r3.NewVec(0, 0, 12))
	require.NoError(t, err)
	_, err = b.Placed(shift)
	require.NoError(t, err)
	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	requireExactGap(t, report, 2)
}

func TestClearanceStopBuiltStackTouching(t *testing.T) {
	// A stack sharing its cap plane: coplanar caps with opposing normals and
	// a positive-area trim overlap — the §6 coplanar Plane × Plane contact
	// certificate. The gap is a measured Exact zero and passes the near-zero
	// gate on its own terms (§1; verification §5).
	doc := decad.New()
	boxBody(t, doc, 0, 0, 100, 60, 10)
	b := boxBody(t, doc, 30, 20, 50, 40, 10)
	shift, err := r3.Translation(r3.NewVec(0, 0, 10))
	require.NoError(t, err)
	_, err = b.Placed(shift)
	require.NoError(t, err)

	// The partition is owed unasked, and the touching boxes already prove
	// the interiors disjoint.
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)

	report, err = doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	require.True(t, report.Trustworthy())
	requireExactGap(t, report, 0)
}

func TestClearanceTangentSpheresSuspect(t *testing.T) {
	// Two balls exactly tangent along a diagonal: the strict exterior branch
	// fails at equality, which routes to §6 — and the Sphere × Sphere
	// contact certificate is PR 3, so the touching pair yields NO row and
	// stays Suspect (§4/§8), asked or unasked.
	doc := decad.New()
	ballBody(t, doc, 10)
	b := ballBody(t, doc, 5)
	step := 15 / math.Sqrt(3)
	shift, err := r3.Translation(r3.NewVec(step, step, step))
	require.NoError(t, err)
	_, err = b.Placed(shift)
	require.NoError(t, err)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, report.Status, `an uncertified touching pair joins neither list`)
	require.Empty(t, report.Interferences)

	report, err = doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, report.Status)
	require.NotNil(t, report.Clearances)
	require.Empty(t, report.Clearances, `a touching pair's zero is Exact or no answer at all (§1)`)
}

func TestClearanceBeyondBoxesInHole(t *testing.T) {
	// A cube standing inside a ring's hole: the bounding boxes overlap, so
	// box separation proves nothing — the always-on kernel proves the
	// partition by boundary clearance plus the §2 nesting-exclusion casts.
	// The gap is the hole wall against the cube's corner edges:
	// 10 − 2.5·√2, Exact.
	doc := decad.New()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-20, -20, 20, 20)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(0, 0), 10)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			prof = p
		}
	}
	require.NotNil(t, prof)
	_, err = doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	boxBody(t, doc, -2.5, -2.5, 2.5, 2.5, 5)

	before := doc.Recipe()
	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, before, doc.Recipe(), `Verify never mutates the document`)
	require.Len(t, doc.Bodies(), 2)
	require.Equal(t, decad.Sound, report.Status)
	requireExactGap(t, report, 10-2.5*math.Sqrt2)
}

func TestClearanceCoaxialPegInTube(t *testing.T) {
	// The §4 worked containment numbers: two coaxial cylinders of radii 10
	// and 5 have spine distance 0 and a genuine 5 mm gap — the peg-in-hole
	// clearance a subtraction-only rule would misread as "carriers meet".
	// The tube is a revolve and the peg a prism, so the pair also covers the
	// mixed-payload path.
	doc := decad.New()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 10, 10, 15)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	_, err = doc.Revolve(s, s.Profiles()[0], uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	w2 := sketch.NewWorld()
	s2, err := w2.CreateSketch(w2.YZ())
	require.NoError(t, err)
	c := s2.CreatePoint(0, 0)
	s2.Fix(c)
	s2.CreateCircle(c, 5)
	_, err = s2.Solve(t.Context())
	require.NoError(t, err)
	_, err = doc.Extrude(s2, s2.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	requireExactGap(t, report, 5)
}

func TestClearanceToriP8(t *testing.T) {
	// The §7 worked torus pair: parallel axes 30 mm apart, major 10,
	// minor 2 — tube to tube through the circle × circle P8 bracket, ⊕ the
	// minor radii: 6 mm with a vanishing certified bound, Approximate, and
	// well within the default gate.
	doc := decad.New()
	torusBody(t, doc, 10, 2)
	b := torusBody(t, doc, 10, 2)
	shift, err := r3.Translation(r3.NewVec(0, 30, 0))
	require.NoError(t, err)
	_, err = b.Placed(shift)
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status, `5e-10-order bound ≤ the 6e-3 gate`)
	require.Len(t, report.Clearances, 1)
	row := report.Clearances[0]
	require.Equal(t, decad.Approximate, row.Gap.Exactness, `a bracketed winner is honest-Approximate`)
	require.InDelta(t, 6.0, row.Gap.Value.Mag(), 1e-6)
	bound := row.Gap.Bound.Mag()
	require.Greater(t, bound, 0.0)
	require.Less(t, bound, 1e-5)
}

func TestClearanceConeCoarseRow(t *testing.T) {
	// PR 1's cone staging (§8): cone-involved face pairs carry only a coarse
	// enclosure interval. Far enough that even the coarse lower bound clears
	// zero, the pair is proven disjoint with a wide honest row — and the §7
	// gate reads the wide bound Suspect, never a silent pass.
	doc := decad.New()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	s.Fix(o)
	apex := s.CreatePoint(10, 0)
	top := s.CreatePoint(0, 10)
	s.CreateLine(o, apex)
	s.CreateLine(apex, top)
	s.CreateLine(top, o)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	_, err = doc.Revolve(s, s.Profiles()[0], uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	boxBody(t, doc, 30, 12, 40, 22, 5)

	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, report.Status, `a row measured coarser than the tolerance reads Suspect`)
	require.Len(t, report.Clearances, 1)
	row := report.Clearances[0]
	require.Equal(t, decad.Approximate, row.Gap.Exactness)
	require.Greater(t, row.Gap.Value.Mag()-row.Gap.Bound.Mag(), 0.0, `the row exists only because lo cleared zero`)
	require.Greater(t, row.Gap.Bound.Mag(), 1e-3*row.Gap.Value.Mag(), `the honest wide interval fails the gate`)
}

func TestClearanceConeCoarseUndecided(t *testing.T) {
	// The other half of the cone staging: with the face enclosures
	// overlapping, the coarse lower bound cannot clear zero and the pair is
	// undecided — Suspect unasked, and no fabricated row when asked.
	doc := decad.New()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	s.Fix(o)
	apex := s.CreatePoint(10, 0)
	top := s.CreatePoint(0, 10)
	s.CreateLine(o, apex)
	s.CreateLine(apex, top)
	s.CreateLine(top, o)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	_, err = doc.Revolve(s, s.Profiles()[0], uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	boxBody(t, doc, 7, 4, 9, 6, 2)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, report.Status)

	report, err = doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, report.Status)
	require.Empty(t, report.Clearances)
}

func TestClearanceNestedPairUndecided(t *testing.T) {
	// A ball wholly inside another: the boundaries never meet, so boundary
	// clearance alone would prove the wrong thing (§2) — the nesting witness
	// cast finds the inner shell inside the outer body, the pair is proven
	// NOT disjoint, and per §7 the proven overlap follows the parent's
	// staging: Suspect, with no row in either list.
	doc := decad.New()
	ballBody(t, doc, 10)
	ballBody(t, doc, 2)

	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, report.Status)
	require.Empty(t, report.Interferences, `an Interference row needs a volume this evaluator cannot bound`)
	require.Empty(t, report.Clearances)
	for _, br := range report.Bodies {
		require.Equal(t, decad.Sound, br.Status, `the bodies are sound; the pair is what is undecided`)
	}
}

func TestClearancePlacedRotatedCube(t *testing.T) {
	// A pair with a genuinely rotated pose: cube B is rotated 45° about Z
	// and translated, so its nearest feature to cube A is a vertical corner
	// edge — edge × edge, closed form, Exact.
	doc := decad.New()
	boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 0, 0, 10, 10, 10)
	rot, err := r3.Rotation(r3.NewVec(0, 0, 1), units.Degrees(45))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(25, 5, 0))
	require.NoError(t, err)
	motion, err := rot.Then(shift)
	require.NoError(t, err)
	_, err = b.Placed(motion)
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	// B's nearest corner edge sits at (25 − 10/√2, 5 + 10/√2); A's nearest
	// vertical edge at (10, 10).
	dx := 25 - 10/math.Sqrt2 - 10
	dy := 5 + 10/math.Sqrt2 - 10
	requireExactGap(t, report, math.Hypot(dx, dy))
}

func TestClearanceSubTolOverlapNotCertified(t *testing.T) {
	// Two boxes overlapping by 5e-9 mm: not a touching contact, and the
	// exact coplanar certificate must NOT bless it — the pair is neither
	// proven disjoint nor certified touching, so it reads Suspect.
	doc := decad.New()
	boxBody(t, doc, 0, 0, 10, 10, 8)
	other := boxBody(t, doc, 0, 0, 10, 10, 8)
	shift, err := r3.Translation(r3.NewVec(0, 0, 8-5e-9))
	require.NoError(t, err)
	_, err = other.Placed(shift)
	require.NoError(t, err)
	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Empty(t, report.Clearances, `no fabricated zero row over a real overlap`)
	require.Equal(t, decad.Suspect, report.Status)
}

func TestClearanceBallCenteredInHole(t *testing.T) {
	// A radius-5 ball centered on the axis of a radius-10 through-hole: the
	// point-spine d_sup branch is trivially constant against the hole wall,
	// and the 5 mm ring gap is proven Exact.
	doc := decad.New()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-20, -20, 20, 20)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(0, 0), 10)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			prof = p
		}
	}
	require.NotNil(t, prof)
	_, err = doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	// The ball: revolved about the u axis at the origin, then placed so its
	// center sits on the hole axis at mid-height.
	ball := ballBody(t, doc, 5)
	shift, err := r3.Translation(r3.NewVec(0, 0, 5))
	require.NoError(t, err)
	_, err = ball.Placed(shift)
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	requireExactGap(t, report, 5)
}
