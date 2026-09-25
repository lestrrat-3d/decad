package decad_test

import (
	"context"
	"math"
	"runtime"
	"strings"
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

// boxBodyAtZ is boxBody's own z-offset sibling: an axis-aligned rectangle
// extruded from an offset plane, spanning z ∈ [z0, z0+h]. A fixture that
// needs a clean transversal crossing — no operand face landing exactly on
// the other operand's own face plane — reaches for this rather than boxBody,
// whose z0 is always 0.
func boxBodyAtZ(t *testing.T, doc *decad.Document, x0, y0, x1, y1, z0, h float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z0)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
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

// requireBoundedGapContains asserts the report carries exactly one clearance
// row whose interval CONTAINS the true gap mm and is honestly Approximate —
// never a tightened number, never a silent Exact over a widened body
// (bodyGeom.delta, docs/clearance-design.md §2). Bound is never pinned to a
// literal: it differs between amd64 and arm64 (both charge ulp-scale
// roundoff, at different rounding), so this only asserts enclosure and
// positivity.
func requireBoundedGapContains(t *testing.T, report *decad.Report, mm float64) {
	t.Helper()
	require.Len(t, report.Clearances, 1)
	row := report.Clearances[0]
	require.Equal(t, decad.Approximate, row.Gap.Exactness)
	bound := row.Gap.Bound.Mag()
	require.Greater(t, bound, 0.0)
	value := row.Gap.Value.Mag()
	require.LessOrEqual(t, value-bound, mm, `the interval's low end must not exclude the truth`)
	require.GreaterOrEqual(t, value+bound, mm, `the interval's high end must not exclude the truth`)
}

// TestClearanceTiltedPlanePairIntervalContainsTruth is this file's flagship
// widening regression, and the single assertion bodyGeom.delta exists for.
//
// Two plain extrudes, no placement anywhere, off ONE tilted sketch plane whose
// axes are a 37° rotation about the normalized (0.3, 0.7, 0.55) and whose
// origin sits far from the world origin. The two rectangles are separated by
// exactly 300 mm in the plane's OWN local u, so the true gap between the
// facing side faces is exactly 300 mm, whatever the plane's pose: the lift is
// an isometry.
//
// Before the widening landed, the kernel published 299.99999999999989 mm with
// a Bound of 2.84e-14 mm and an Exact-adjacent Sound report — an interval that
// EXCLUDED the truth by about four times its own width, because the carriers
// were lifted through `prismPayload.point`/`dir` in raw float64 and the
// rounding that lift commits was charged nowhere. The axis-aligned control
// below returns exactly 300 as Exact through the same code, which is why a
// single axis-aligned fixture never caught this: the frame's own zero fast
// path (bounds.go's frameAndPlacementRoundAllow) makes every term vanish.
//
// The assertion is enclosure, never a pinned Bound literal — the charge is
// ulp-scale and rounds differently on amd64 and arm64.
func TestClearanceTiltedPlanePairIntervalContainsTruth(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	tiltedPlanePair(t, doc, r3.NewVec(311, 207, 405))

	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	requireBoundedGapContains(t, report, 300)
}

// tiltedPlanePair builds the flagship fixture's two extrudes off ONE tilted
// sketch plane at the given origin: rectangles at u ∈ [0, 100] and
// u ∈ [400, 500], both v ∈ [0, 60], each extruded 40 mm. The facing side
// faces are exactly 300 mm apart in the plane's own local u, and the lift is
// an isometry, so the TRUE gap is exactly 300 mm at every origin.
func tiltedPlanePair(t *testing.T, doc *decad.Document, origin r3.Vec) {
	t.Helper()
	axis, ok := r3.NewVec(0.3, 0.7, 0.55).Normalize()
	require.True(t, ok)
	rot, err := r3.Rotation(axis, units.Degrees(37))
	require.NoError(t, err)
	frame, err := r3.NewFrame(origin, rot.ApplyDir(r3.NewVec(1, 0, 0)), rot.ApplyDir(r3.NewVec(0, 1, 0)))
	require.NoError(t, err)

	w := sketch.NewWorld()
	plane, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	near := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(near.A)
	far := s.CreateRectangle(400, 0, 500, 60)
	s.Fix(far.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 2)
	for _, p := range s.Profiles() {
		_, err = doc.Extrude(s, p, decad.Distance{D: units.Millimeters(40), Dir: decad.Along})
		require.NoError(t, err)
	}
}

func TestClearanceFarOriginTiltedPlanePairContainsTruth(t *testing.T) {
	t.Parallel()
	// The flagship fixture's own far-origin sibling, and the leg that makes
	// bodyGeom.delta's POINT term load-bearing on its own. At an origin of
	// (1e6, 2e6, 3e6) the frame lift rounds at the scale of ulp(3e6), so the
	// held gap misses the truth by ~1.7e-10 mm — two orders of magnitude more
	// than the per-face tilt term alone charges (~3.2e-11 mm). Dropping
	// frameAndPlacementRoundAllow from addPrismFaces' own g.delta therefore
	// turns THIS pair red while leaving the flagship above green, which is
	// why both fixtures exist: at a near origin the tilt term covers for the
	// point term, and one combined leg would prove neither.
	doc := decad.New()
	tiltedPlanePair(t, doc, r3.NewVec(1e6, 2e6, 3e6))

	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	requireBoundedGapContains(t, report, 300)
}

func TestClearanceInchExtrudeAxialGapContainsTruth(t *testing.T) {
	t.Parallel()
	// The leg that makes bodyGeom.delta's AXIAL term load-bearing on its own.
	// Both sketch planes here are axis-aligned and unplaced, so the point and
	// tilt terms are exactly zero under the frame's own exemption and only
	// prismPayload.axialDelta can widen anything.
	//
	// The lower box is swept 1.7 in. That is exactly 43.18 mm, which has no
	// float64 representation, so the held top cap sits a few ulp off the level
	// the record denotes; the upper box starts at z = 50, which is exact. The
	// true gap is therefore exactly 50 − 43.18 = 6.82 mm.
	//
	// Uncharged, the held reading publishes as a POINT interval labelled Exact
	// that EXCLUDES 6.82 mm — the same defect class as the tilted pair above,
	// reached by the other term. Charged, the row is honestly Approximate and
	// encloses it with about six ulp to spare at each end, which is the margin
	// that keeps this assertion stable on arm64 as well as amd64.
	doc := decad.New()

	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	base := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(base.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	_, err = doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Inches(1.7), Dir: decad.Along})
	require.NoError(t, err)

	boxBodyAtZ(t, doc, 20, 10, 80, 50, 50, 10)

	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	requireBoundedGapContains(t, report, 6.82)
}

func TestClearanceAxisAlignedPairOf300IsExact(t *testing.T) {
	t.Parallel()
	// The tilted pair's own axis-aligned control, on the world XY plane with
	// the identical 300 mm plane-local separation: every bodyGeom.delta term
	// collapses to exactly zero under the frame's own axis-aligned exemption,
	// so this pair keeps publishing an Exact 300 mm. It is what proves the
	// widening COSTS nothing where nothing was rounded — the tilted case
	// above widens because its lift genuinely rounds, not because the
	// widening fires indiscriminately.
	doc := decad.New()
	boxBody(t, doc, 0, 0, 100, 60, 40)
	boxBody(t, doc, 400, 0, 500, 60, 40)

	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	requireExactGap(t, report, 300)
}

func TestClearanceCubesDiagonalOffset(t *testing.T) {
	t.Parallel()
	// The §7 worked pair: a 10 mm cube at the origin and one at x∈[13,23],
	// y∈[12,22] — the facing-face plateaus are discarded (the trims clear in
	// projection) and the minimum falls to two parallel vertical edges:
	// √13 ≈ 3.606 mm. Its float64 representation needs a proven bound.
	doc := decad.New()
	boxBody(t, doc, 0, 0, 10, 10, 10)
	boxBody(t, doc, 13, 12, 23, 22, 10)
	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	require.True(t, report.Passed())
	requireBoundedGapContains(t, report, math.Sqrt(13))
}

func TestClearanceAxisBoxesNearTolerance(t *testing.T) {
	t.Parallel()
	const gap = 3e-8
	x0 := float64(10 + gap)
	doc := decad.New()
	boxBody(t, doc, 0, 0, 10, 10, 10)
	boxBody(t, doc, x0, 0, 20+gap, 10, 10)

	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	require.Len(t, report.Clearances, 1)
	row := report.Clearances[0]
	truth := x0 - 10
	require.LessOrEqual(t, row.Gap.Value.Mag()-row.Gap.Bound.Mag(), truth)
	require.GreaterOrEqual(t, row.Gap.Value.Mag()+row.Gap.Bound.Mag(), truth)
}

func TestClearanceStackedCubes(t *testing.T) {
	t.Parallel()
	// The same cubes stacked 2 mm apart: the facing caps' trims overlap in
	// projection, so the face × face plateau carries the answer, 2 mm (§7's
	// second worked row). The second cube arrives via Placed, so the pair
	// also covers the placed-pose path — which now widens the proven
	// interval by the placement's own frame/placement rounding
	// (bodyGeom.delta), so the row reads honest-Approximate rather than the
	// Exact this pair published before the widening existed to charge it.
	doc := decad.New()
	boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 0, 0, 10, 10, 10)
	shift, err := r3.Translation(r3.NewVec(0, 0, 12))
	require.NoError(t, err)
	_, err = b.Placed(t.Context(), shift)
	require.NoError(t, err)
	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	requireBoundedGapContains(t, report, 2)
}

func TestClearanceStopBuiltStackTouching(t *testing.T) {
	t.Parallel()
	// A stack sharing its cap plane, built directly at the shared level
	// (boxBodyAtZ — an offset sketch plane, never Placed): coplanar caps
	// with opposing normals and a positive-area trim overlap — the §6
	// coplanar Plane × Plane contact certificate. Both bodies' bodyGeom.delta
	// stay exactly zero (an offset plane keeps the frame's own U/V axis-
	// aligned and the placement Identity), so the certificate still fires
	// and the gap is a measured Exact zero, passing the near-zero gate on
	// its own terms (§1; verification §5).
	doc := decad.New()
	boxBody(t, doc, 0, 0, 100, 60, 10)
	boxBodyAtZ(t, doc, 30, 20, 50, 40, 10, 10)

	// The partition is owed unasked, and the touching boxes already prove
	// the interiors disjoint.
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)

	report, err = doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	require.True(t, report.Passed())
	requireExactGap(t, report, 0)
}

func TestClearancePlacedStopBuiltStackTouchingIsUndecided(t *testing.T) {
	t.Parallel()
	// The identical stop-built stack, with the smaller box arriving through
	// Placed (a pure translation) instead of a direct offset-plane
	// construction: the placement's own frame/placement rounding
	// (bounds.go's frameAndPlacementRoundAllow) makes the placed box's
	// bodyGeom.delta nonzero, so the §6 coplanar contact certificate — an
	// exact material-side claim — refuses to fire (clearancePair's own doc
	// comment). The partition is still proven disjoint through the
	// mesh-boolean fallback (verification §1's own "no fabricated rows"
	// alternative route), but the requested gap stays unmeasured: Suspect,
	// DiagUndecidedClearance, no Clearance row. This is the honest cost of
	// refusing to certify a touch a rounded carrier plane cannot actually
	// prove — never a narrower answer, an ABSENT one.
	doc := decad.New()
	boxBody(t, doc, 0, 0, 100, 60, 10)
	b := boxBody(t, doc, 30, 20, 50, 40, 10)
	shift, err := r3.Translation(r3.NewVec(0, 0, 10))
	require.NoError(t, err)
	_, err = b.Placed(t.Context(), shift)
	require.NoError(t, err)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status, `the partition is still proven by the mesh-boolean fallback`)

	report, err = doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, report.Status)
	require.Empty(t, report.Clearances, `a touching pair's answer is Exact zero or no answer at all (§1)`)
}

func TestClearanceTangentSpheresSuspect(t *testing.T) {
	t.Parallel()
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
	_, err = b.Placed(t.Context(), shift)
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
	t.Parallel()
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
	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 2)
	require.Equal(t, decad.Sound, report.Status)
	requireExactGap(t, report, 10-2.5*math.Sqrt2)
}

func TestClearanceCoaxialPegInTube(t *testing.T) {
	t.Parallel()
	// The §4 worked containment numbers: two coaxial cylinders of radii 10
	// and 5 have spine distance 0 and a genuine 5 mm gap — the peg-in-hole
	// clearance a subtraction-only rule would misread as "carriers meet".
	// The tube is a revolve and the peg a prism sketched on the YZ plane, so
	// the pair also covers the mixed-payload path — a plane the frame's own
	// zero fast path does not cover (bounds.go's frameAndPlacementRoundAllow
	// only exempts EXACTLY U=(1,0,0), V=(0,1,0)), and a full turn's own
	// angular displacement never collapses to exactly zero (2π has no exact
	// rational value), so both bodyGeom.delta are nonzero and the row reads
	// honest-Approximate rather than the Exact this pair published before
	// the widening existed to charge either term.
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
	requireBoundedGapContains(t, report, 5)
}

func TestClearanceToriP8(t *testing.T) {
	t.Parallel()
	// The §7 worked torus pair: parallel axes 30 mm apart, major 10,
	// minor 2 — tube to tube through the circle × circle P8 bracket, ⊕ the
	// minor radii: 6 mm with a vanishing certified bound, Approximate, and
	// well within the default gate.
	doc := decad.New()
	torusBody(t, doc, 10, 2)
	b := torusBody(t, doc, 10, 2)
	shift, err := r3.Translation(r3.NewVec(0, 30, 0))
	require.NoError(t, err)
	_, err = b.Placed(t.Context(), shift)
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, report.Status, `bounded mass results remain visible`)
	require.Len(t, report.Clearances, 1)
	row := report.Clearances[0]
	require.Equal(t, decad.Approximate, row.Gap.Exactness, `a bracketed winner is honest-Approximate`)
	require.InDelta(t, 6.0, row.Gap.Value.Mag(), 1e-6)
	bound := row.Gap.Bound.Mag()
	require.Greater(t, bound, 0.0)
	require.Less(t, bound, 1e-5)
}

func TestClearanceNonFinitePolynomialIsUndecided(t *testing.T) {
	t.Parallel()
	// Every input is finite, but the P4 squared-distance coefficients overflow.
	// The kernel must refuse the gap instead of solving a zero-substituted
	// polynomial and publishing a false Exact clearance.
	const (
		offset         = 1.4e154
		major          = 1.1e50
		minor          = 1e50
		cylinderRadius = 1e50
		height         = 4e154
	)

	doc := decad.New()
	cylinderWorld := sketch.NewWorld()
	cylinderSketch, err := cylinderWorld.CreateSketch(cylinderWorld.XY())
	require.NoError(t, err)
	cylinderCenter := cylinderSketch.CreatePoint(0, 0)
	cylinderSketch.Fix(cylinderCenter)
	cylinderSketch.CreateCircle(cylinderCenter, cylinderRadius)
	_, err = cylinderSketch.Solve(t.Context())
	require.NoError(t, err)
	_, err = doc.Extrude(
		cylinderSketch,
		cylinderSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(height), Dir: decad.Along},
	)
	require.NoError(t, err)

	torusWorld := sketch.NewWorld()
	torusSketch, err := torusWorld.CreateSketch(torusWorld.XY())
	require.NoError(t, err)
	torusCenter := torusSketch.CreatePoint(major, 0)
	torusSketch.Fix(torusCenter)
	torusSketch.CreateCircle(torusCenter, minor)
	_, err = torusSketch.Solve(t.Context())
	require.NoError(t, err)
	torus, err := doc.Revolve(
		torusSketch,
		torusSketch.Profiles()[0],
		decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{V: 1}},
		decad.FullRevolution{},
	)
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(offset, 0, height/2))
	require.NoError(t, err)
	_, err = torus.Placed(t.Context(), shift)
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, report.Status)
	require.Empty(t, report.Clearances)
	diag, ok := findDiagnostic(report.Diagnostics, decad.DiagUndecidedClearance)
	require.True(t, ok)
	require.Equal(t, decad.Suspect, diag.Status)
}

func TestClearanceConeCoarseRow(t *testing.T) {
	t.Parallel()
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
	t.Parallel()
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

func TestClearanceNestedPairReportsContainedVolume(t *testing.T) {
	t.Parallel()
	// A ball wholly inside another: the boundaries never meet, so boundary
	// clearance alone would prove the wrong thing (§2). One witness from the
	// inner body's material lump proves strict full containment, so Verify
	// reuses the complete inner-body volume.
	doc := decad.New()
	ballBody(t, doc, 10)
	inner := ballBody(t, doc, 2)
	want, err := inner.Volume()
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Interfering, report.Status)
	require.Len(t, report.Interferences, 1)
	require.Equal(t, want, report.Interferences[0].Volume)
	require.Empty(t, report.Clearances)
	for _, br := range report.Bodies {
		require.Equal(t, decad.Suspect, br.Status, `the bodies carry bounded mass results`)
	}
}

func TestClearancePlacedRotatedCube(t *testing.T) {
	t.Parallel()
	// A pair with a genuinely rotated pose: cube B is rotated 45° about Z
	// and translated, so its nearest feature to cube A is a vertical corner
	// edge — edge × edge, closed form. The rotation's own frame/placement
	// rounding (bodyGeom.delta) now widens the row to honest-Approximate.
	doc := decad.New()
	boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 0, 0, 10, 10, 10)
	rot, err := r3.Rotation(r3.NewVec(0, 0, 1), units.Degrees(45))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(25, 5, 0))
	require.NoError(t, err)
	motion, err := rot.Then(shift)
	require.NoError(t, err)
	_, err = b.Placed(t.Context(), motion)
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	// B's nearest corner edge sits at (25 − 10/√2, 5 + 10/√2); A's nearest
	// vertical edge at (10, 10).
	dx := 25 - 10/math.Sqrt2 - 10
	dy := 5 + 10/math.Sqrt2 - 10
	requireBoundedGapContains(t, report, math.Hypot(dx, dy))
}

func TestClearanceSubTolOverlapNotCertified(t *testing.T) {
	t.Parallel()
	// Two boxes overlapping by 5e-9 mm: not a touching contact, and the
	// exact coplanar certificate must NOT bless it — the pair is neither
	// proven disjoint nor certified touching, so it reads Suspect.
	doc := decad.New()
	boxBody(t, doc, 0, 0, 10, 10, 8)
	other := boxBody(t, doc, 0, 0, 10, 10, 8)
	shift, err := r3.Translation(r3.NewVec(0, 0, 8-5e-9))
	require.NoError(t, err)
	_, err = other.Placed(t.Context(), shift)
	require.NoError(t, err)
	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Empty(t, report.Clearances, `no fabricated zero row over a real overlap`)
	require.Equal(t, decad.Suspect, report.Status)
}

func TestClearanceBallCenteredInHole(t *testing.T) {
	t.Parallel()
	// A radius-5 ball centered on the axis of a radius-10 through-hole: the
	// point-spine d_sup branch is trivially constant against the hole wall,
	// and the true 5 mm ring gap is enclosed by the proven interval. The
	// ball is both a full-turn revolve (angularDelta never collapses to
	// exactly zero — 2π has no exact rational value) and placed, so
	// bodyGeom.delta is nonzero and the row reads honest-Approximate.
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
	_, err = ball.Placed(t.Context(), shift)
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	requireBoundedGapContains(t, report, 5)
}

func TestClearanceNearParallelPlateReadsTheCorner(t *testing.T) {
	t.Parallel()
	// A plate pair with the upper plate tilted DOWN by a tiny angle: the
	// planes are near-parallel, and a tolerance-based plateau would bless
	// the anchor height 1.0 as an Exact minimum — but the true minimum is
	// at the dipped edge, 1 − 10·sin(5e-7). The exact-only plateau rule
	// leaves the tiers to find the corner, and any reported row must carry
	// the corner value (or the pair may read Suspect; it must never carry
	// the wrong plateau).
	doc := decad.New()
	boxBody(t, doc, 0, 0, 10, 10, 4)
	other := boxBody(t, doc, 0, 0, 10, 10, 4)
	rot, err := r3.Rotation(r3.NewVec(1, 0, 0), units.Radians(-5e-7))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(0, 0, 5))
	require.NoError(t, err)
	motion, err := rot.Then(shift)
	require.NoError(t, err)
	_, err = other.Placed(t.Context(), motion)
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	want := 1 - 10*math.Sin(5e-7)
	for _, c := range report.Clearances {
		require.InDelta(t, want, c.Gap.Value.Base(), 1e-9,
			`the row must carry the dipped-edge minimum, not the plateau`)
	}
}

// rodBody extrudes a circle of radius r centered at (cx, cy) symmetrically
// about the sketch plane: a cylinder whose axis is the world Z line through
// (cx, cy), spanning z ∈ [−half, +half].
func rodBody(t *testing.T, doc *decad.Document, cx, cy, r, half float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	c := s.CreatePoint(cx, cy)
	s.Fix(c)
	s.CreateCircle(c, r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Symmetric{D: units.Millimeters(half)})
	require.NoError(t, err)
	return body
}

func TestClearanceRodPiercingBallIsNeverSound(t *testing.T) {
	t.Parallel()
	// A rod of radius 1 on the axis (3, 0), running clear through a ball of
	// radius 10 at the origin — roughly 60 mm³ of overlap, and no gap exists at
	// all. The §4 nested branch reads a SUPREMUM over the inner spine, and a
	// cylinder's spine is an infinite LINE: its supremum against the ball's
	// point spine is +∞, so the containment test must FAIL and the crossing
	// must be met. Hand that branch the 3 mm foot distance instead and the
	// crossing check is skipped: the kernel then certifies the piercing pair as
	// nested and reports it Sound with an Exact 0.688779163216 mm clearance —
	// the worst answer a verification oracle can give.
	doc := decad.New()
	ballBody(t, doc, 10)
	rodBody(t, doc, 3, 0, 1, 10.5)

	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, report.Status, `an interpenetrating pair is never Sound`)
	require.False(t, report.Passed())
	require.Empty(t, report.Clearances, `no gap exists, so no row may claim one`)
}

func TestClearanceCoaxialToriReadTheRing(t *testing.T) {
	t.Parallel()
	// The certified constant-distance branch (§4), pinned: two EXACTLY coaxial
	// full tori — spine radii 10 and 5, tubes 2 and 1 — have a constant spine
	// distance of 5 and a true 2 mm ring gap. Neither face carries an edge,
	// so the boundary tiers hold nothing here: this reading stands on the
	// coaxial certificate alone, and it must keep standing. Both bodies are
	// full-turn revolves, and a full turn's own angular displacement never
	// collapses to exactly zero — 2π has no exact rational value — so
	// bodyGeom.delta is nonzero for each and the row reads honest-Approximate
	// rather than Exact.
	doc := decad.New()
	torusBody(t, doc, 10, 2)
	torusBody(t, doc, 5, 1)

	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, report.Status)
	requireBoundedGapContains(t, report, 2)
}

func TestClearanceNearCoaxialToriAreNotExact(t *testing.T) {
	t.Parallel()
	// The same pair with the inner torus 1e-12 mm off the shared axis: NOT
	// coaxial, so the constant-distance closed form does not hold and its 2 mm
	// answer is off by up to the offset. Two full tori have no edges and no
	// vertices — nothing in the lower tiers can out-vote a wrong Exact here —
	// so the cell must decline. Undecided is the honest answer (clearance §4);
	// an Exact 2.0 with a zero bound is a lie.
	doc := decad.New()
	torusBody(t, doc, 10, 2)
	inner := torusBody(t, doc, 5, 1)
	shift, err := r3.Translation(r3.NewVec(0, 0, 1e-12))
	require.NoError(t, err)
	_, err = inner.Placed(t.Context(), shift)
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, report.Status)
	require.Empty(t, report.Clearances)
	for _, c := range report.Clearances {
		require.NotEqual(t, decad.Exact, c.Gap.Exactness)
	}
}

// sturmBuildCancelContext reports cancellation only while the Sturm chain
// build is on the call stack. A test using it proves the poll it observed is
// INSIDE that build, rather than at one of the phase boundaries the clearance
// kernel already polls before and after it.
type sturmBuildCancelContext struct {
	context.Context //nolint:containedctx // deterministic cancellation wrapper used only within one test call.
	entered         bool
}

func (c *sturmBuildCancelContext) Err() error {
	pcs := make([]uintptr, 32)
	frames := runtime.CallersFrames(pcs[:runtime.Callers(2, pcs)])
	for {
		frame, more := frames.Next()
		if strings.HasSuffix(frame.Function, ".sturmChainContext") {
			c.entered = true
			return context.Canceled
		}
		if !more {
			return nil
		}
	}
}

// TestVerifyClearanceCancellationInsideSturmChainBuild is the public half of
// the chain build's cancellation proof: the §7 torus pair drives the degree-8
// circle × circle bracket, whose chain build is the longest single stretch of
// arithmetic in a clearance run, and a context cancelled there surfaces
// context.Canceled from Verify with the document untouched.
func TestVerifyClearanceCancellationInsideSturmChainBuild(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	torusBody(t, doc, 10, 2)
	b := torusBody(t, doc, 10, 2)
	shift, err := r3.Translation(r3.NewVec(0, 30, 0))
	require.NoError(t, err)
	_, err = b.Placed(t.Context(), shift)
	require.NoError(t, err)
	before := snapshotDocument(t, doc)
	ctx := &sturmBuildCancelContext{Context: t.Context()}

	report, err := doc.Verify(ctx, decad.WithClearances())

	require.ErrorIs(t, err, context.Canceled)
	require.Nil(t, report)
	require.True(t, ctx.entered, "the torus pair must reach the Sturm chain build")
	requireDocumentUnchanged(t, doc, before)
}

type vertexTierCancelContext struct {
	context.Context //nolint:containedctx // deterministic cancellation wrapper used only within one test call.
	entered         bool
}

func (c *vertexTierCancelContext) Err() error {
	pcs := make([]uintptr, 32)
	frames := runtime.CallersFrames(pcs[:runtime.Callers(2, pcs)])
	for {
		frame, more := frames.Next()
		if strings.HasSuffix(frame.Function, ".vertexTier") {
			c.entered = true
			return context.Canceled
		}
		if !more {
			return nil
		}
	}
}

func TestVerifyClearanceCancellationInsideVertexTier(t *testing.T) {
	t.Parallel()
	const sides = 16
	polygon := func(cx float64) [][2]float64 {
		pts := make([][2]float64, sides)
		for i := range pts {
			th := 2 * math.Pi * float64(i) / sides
			pts[i] = [2]float64{cx + 10*math.Cos(th), 10 * math.Sin(th)}
		}
		return pts
	}
	doc := decad.New()
	for _, cx := range []float64{0, 50} {
		s, p := polygonSketch(t, polygon(cx))
		_, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
		require.NoError(t, err)
	}
	before := snapshotDocument(t, doc)
	ctx := &vertexTierCancelContext{Context: t.Context()}

	report, err := doc.Verify(ctx, decad.WithClearances())

	require.ErrorIs(t, err, context.Canceled)
	require.Nil(t, report)
	require.True(t, ctx.entered)
	requireDocumentUnchanged(t, doc, before)
}
