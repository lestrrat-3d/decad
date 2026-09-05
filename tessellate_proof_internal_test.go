package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file exercises docs/tessellation-design.md §2's private proof record on
// Mesh — the per-face displacement, the area slack's coordinate term, and the
// occupied-volume bound — as docs/tessellation-reach-design.md §3 composes it
// for the prism, cup and faceted paths. Every assertion here is on computed
// geometry: a published bound against the closed form of the quantity it
// bounds, never merely that a field is set.

// internalHoledPlateBody is tessellate_test.go's holedPlateBody built inside
// the package, so a test can read the private proof record the public surface
// does not expose: the 100×60 plate with a 10 mm-radius hole at (70, 30),
// extruded 8 mm.
func internalHoledPlateBody(t *testing.T, doc *Document) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
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
	body, err := doc.Extrude(s, prof, Distance{D: units.Millimeters(8), Dir: Along})
	require.NoError(t, err)
	return body
}

// roleOf is the first provenance role a face carries, which is how the prism
// build names the walk or cap it came from.
func roleOf(t *testing.T, f *Face) string {
	t.Helper()
	origins := f.Origins()
	require.NotEmpty(t, origins)
	return origins[0].Role
}

func TestPrismProofRecordChargesEveryFaceItsOwnDisplacement(t *testing.T) {
	t.Parallel()
	body := internalHoledPlateBody(t, New())
	mesh, err := tessellateContext(t.Context(), body, units.Millimeters(1))
	require.NoError(t, err)

	// Every source face states a bound, and the mesh's global figure is their
	// maximum — never a number no face accounts for.
	worst := 0.0
	for _, f := range mesh.source {
		d, ok := mesh.sourceBound(f)
		require.True(t, ok, `every source face is present in the proof record`)
		worst = math.Max(worst, d)
	}
	require.Equal(t, worst, mesh.bound, `Bound is the maximum sourceBound over the body`)
	require.True(t, mesh.symDiffOK)

	// The hole's chording: one closed circular walk, so the mesh holds one
	// sample per chord on it beside the outline's four corners.
	stations := len(mesh.vertices)/2 - 4
	require.Greater(t, stations, 2)
	sagitta := chordSagitta(10, 2*math.Pi, stations)
	require.Positive(t, sagitta)

	var sawWall, sawStraight, sawCap int
	seen := map[*Face]struct{}{}
	for _, f := range mesh.source {
		if _, done := seen[f]; done {
			continue
		}
		seen[f] = struct{}{}
		d, _ := mesh.sourceBound(f)
		role := roleOf(t, f)
		switch {
		case f.Surface().Kind() == KindCylinder:
			// The hole wall stands on chords that lie inside its own arc, so
			// its bound covers the sagitta it took.
			require.GreaterOrEqual(t, d, sagitta, `the hole wall %q must charge its own chord sagitta`, role)
			sawWall++
		case role == roleCapStart || role == roleCapEnd:
			// A planar carrier does not make a TRIMMED patch exact: each cap's
			// rim is the same chorded circle, so each cap charges it too.
			require.GreaterOrEqual(t, d, sagitta, `cap %q must charge the trim sagitta of the loop bounding it`, role)
			sawCap++
		default:
			// An outer wall is a plane on straight edges at exactly recorded
			// coordinates under an identity placement: held exactly, and the
			// record says so rather than inferring it from the surface kind.
			require.Zero(t, d, `straight outer wall %q is held exactly`, role)
			sawStraight++
		}
	}
	require.Positive(t, sawWall)
	require.Equal(t, 2, sawCap)
	require.Positive(t, sawStraight)
}

func TestPlacedPrismChargesEveryFaceItsPlacementRounding(t *testing.T) {
	t.Parallel()
	doc := New()
	flat := internalHoledPlateBody(t, doc)
	unplaced, err := tessellateContext(t.Context(), flat, units.Millimeters(1))
	require.NoError(t, err)

	// A rotation about an axis no coordinate lies on: the frame/placement write
	// rounds every coordinate, so no face is held exactly any more.
	motion, err := r3.Rotation(r3.NewVec(1, 1, 1), units.Radians(math.Pi/5))
	require.NoError(t, err)
	turned, err := flat.Placed(motion)
	require.NoError(t, err)
	mesh, err := tessellateContext(t.Context(), turned, units.Millimeters(1))
	require.NoError(t, err)

	for _, f := range mesh.source {
		d, ok := mesh.sourceBound(f)
		require.True(t, ok)
		require.Positive(t, d, `a rounded placement leaves no face held exactly (role %q)`, roleOf(t, f))
	}

	// The placement's own contribution is the rigid-map rounding allowance at
	// the section's own coordinate magnitude — a proven ceiling, not a fitted
	// number, so the assertion is a relation and never a pinned literal.
	require.Greater(t, mesh.bound, unplaced.bound)
	require.LessOrEqual(t, mesh.bound-unplaced.bound, rigidRoundAllow(100, 0),
		`the placement adds no more than the rigid write's own allowance at this coordinate scale`)
}

func TestWalkSegmentAreaIsTheCircularSegmentClosedForm(t *testing.T) {
	t.Parallel()
	// A quarter circle of radius 3, chorded n ways: the omitted area is
	// r²/2 · (θ − n·sin(θ/n)), which is what the caps lose and the
	// occupied-volume term multiplies by the sweep height.
	const r = 3.0
	seg := ArcSeg{
		Start:  Point2{U: r, V: 0},
		End:    Point2{U: 0, V: r},
		Center: Point2{U: 0, V: 0},
		TStart: 0,
		TEnd:   1,
	}
	w, err := walkOf(seg, newFreeformWork())
	require.NoError(t, err)
	require.True(t, w.isCircular())

	for _, n := range []int{1, 2, 5, 32} {
		want := r * r / 2 * (math.Pi/2 - float64(n)*math.Sin(math.Pi/2/float64(n)))
		require.InEpsilon(t, want, walkSegmentArea(w, n), 1e-12, `n=%d`, n)
	}
	// The composed helper is still the padded wall term plus both caps' worth
	// of the segment term, so the existing area slack is unchanged.
	const h = 7.0
	require.InEpsilon(t,
		(walkWallSlack(w, 8, h)+2*walkSegmentArea(w, 8))*(1+1e-9),
		walkAreaSlack(w, 8, h), 1e-15)
	require.Positive(t, walkWallSlack(w, 8, h))

	// Refining the chording shrinks the omission monotonically and never turns
	// it negative — the max(…, 0) arm holds once the difference is all rounding.
	prev := math.Inf(1)
	for n := 1; n <= 1<<20; n *= 4 {
		got := walkSegmentArea(w, n)
		require.GreaterOrEqual(t, got, 0.0, `n=%d`, n)
		require.Less(t, got, prev, `n=%d`, n)
		prev = got
	}
}

func TestPrismVolSymDiffBracketsTheCylindersOwnSegmentDeficit(t *testing.T) {
	t.Parallel()
	const r, h = 4.0, 6.0
	doc := New()
	disc := internalDiscBody(t, doc, r, h)
	mesh, err := tessellateContext(t.Context(), disc, units.Millimeters(0.05))
	require.NoError(t, err)
	require.True(t, mesh.symDiffOK)

	// One closed circular walk, so every mesh sample is one of its chords.
	n := len(mesh.vertices) / 2
	require.Greater(t, n, 2)
	// The exact volume between the true cylinder and the chorded prism: the n
	// circular segments the chords omit, swept over the height.
	deficit := h * (math.Pi*r*r - float64(n)*r*r*math.Sin(2*math.Pi/float64(n))/2)
	require.Positive(t, deficit)
	require.GreaterOrEqual(t, mesh.volSymDiff, deficit,
		`the occupied-volume bound must cover the segments the chording omits`)
	require.LessOrEqual(t, mesh.volSymDiff, 2*deficit,
		`and must not be looser than twice it — a prism with no displacement pays for nothing else`)

	// It is a genuinely tighter proof than the bound × held area substitution
	// docs/tessellation-design.md §11 forbids.
	require.Less(t, mesh.volSymDiff, mesh.bound*meshAreaUpper(mesh.vertices, mesh.triangles))

	// A box chords nothing and displaces nothing, so its mesh IS its body.
	box := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
	flat, err := tessellateContext(t.Context(), box, units.Millimeters(1))
	require.NoError(t, err)
	require.Zero(t, flat.volSymDiff)
	require.True(t, flat.symDiffOK)
	require.Zero(t, flat.bound)
}

func TestCupProofRecordCoversEveryPatchItHolds(t *testing.T) {
	t.Parallel()
	doc := New()
	disc := internalDiscBody(t, doc, 8, 10)
	capSel := Faces(FaceCreatedBy(FeatureRef{Step: disc.Origin().Step, Role: roleCapEnd}))
	cup, err := disc.Shell(capSel, units.Millimeters(1))
	require.NoError(t, err)
	_, ok := cup.payload.(cupPayload)
	require.True(t, ok, `a one-cap shell of a disc builds the cup payload`)

	mesh, err := tessellateContext(t.Context(), cup, units.Millimeters(0.05))
	require.NoError(t, err)
	require.True(t, mesh.symDiffOK)
	require.Positive(t, mesh.volSymDiff)

	worst := 0.0
	for _, f := range mesh.source {
		d, ok := mesh.sourceBound(f)
		require.True(t, ok, `every cup source face states a bound`)
		require.Positive(t, d, `every patch of a chorded cup is bounded by a curved trim (role %q)`, roleOf(t, f))
		worst = math.Max(worst, d)
	}
	require.Equal(t, worst, mesh.bound)

	// Both regions' chorded sections are charged: the outer wall and the cavity
	// wall each omit their own circular segments over their own sweep height.
	outerDeficit := (10.0) * (math.Pi*8*8 - 0.0)
	require.Less(t, mesh.volSymDiff, outerDeficit, `the bound is a segment deficit, not the whole cylinder`)
	require.Less(t, mesh.volSymDiff, mesh.bound*meshAreaUpper(mesh.vertices, mesh.triangles))
}

func TestFacetedRestatementPublishesItsPayloadsOwnProofRecord(t *testing.T) {
	t.Parallel()
	doc := New()
	plate := internalBoxBody(t, doc, 0, 0, 20, 20, 8)
	tool := internalDiscBody(t, doc, 2, 30)
	moved, err := r3.Translation(r3.NewVec(10, 10, -10))
	require.NoError(t, err)
	placed, err := tool.Placed(moved)
	require.NoError(t, err)
	drilled, err := Cut(plate, placed)
	require.NoError(t, err)
	fp, ok := drilled.payload.(facetedPayload)
	require.True(t, ok)
	require.Positive(t, fp.meshBound)
	require.Positive(t, fp.volSymDiff)

	mesh, err := tessellateContext(t.Context(), drilled, units.Millimeters(1))
	require.NoError(t, err)
	require.Equal(t, fp.meshBound, mesh.bound)
	require.Equal(t, fp.volSymDiff, mesh.volSymDiff)
	require.Equal(t, fp.areaSlack, mesh.areaSlack)
	require.True(t, mesh.symDiffOK)

	// The payload holds one global composed Delta and no tighter per-face
	// certificate, so every restated face publishes that Delta — never a zero
	// for a restated planar polygon.
	require.NotEmpty(t, mesh.faceBound)
	for _, f := range mesh.source {
		d, ok := mesh.sourceBound(f)
		require.True(t, ok)
		require.Equal(t, fp.meshBound, d)
	}
}

// internalFreeformArchBody is extrude_freeform_test.go's fit-spline arch built
// inside the package: the hump through (0,0), (4,3), (8,0) closed by a chord,
// extruded by height, whose one free-form wall is the only chorded walk in the
// section. The height is a parameter because the chording's own area loss
// splits between the two caps, which do not scale with it, and the wall, which
// scales with it linearly — so a tall prism is what makes the wall's own term
// the dominant one.
func internalFreeformArchBody(t *testing.T, doc *Document, height float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	start := s.CreatePoint(0, 0)
	mid := s.CreatePoint(4, 3)
	end := s.CreatePoint(8, 0)
	_, err = s.CreateFitSpline(start, mid, end)
	require.NoError(t, err)
	s.CreateLine(end, start)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	body, err := doc.Extrude(s, profiles[0], Distance{D: units.Millimeters(height), Dir: Along})
	require.NoError(t, err)
	return body
}

// meshHeldArea and meshHeldVolume are the mesh's own held figures, summed from
// its facets — the quantities the proof record's two allowances must cover the
// gap between and the body's exactly measured Area and Volume.
func meshHeldArea(m *Mesh) float64 {
	total := 0.0
	for _, tri := range m.triangles {
		a, b, c := m.vertices[tri[0]], m.vertices[tri[1]], m.vertices[tri[2]]
		total += b.Sub(a).Cross(c.Sub(a)).Len() / 2
	}
	return total
}

func meshHeldVolume(m *Mesh) float64 {
	total := 0.0
	for _, tri := range m.triangles {
		a, b, c := m.vertices[tri[0]], m.vertices[tri[1]], m.vertices[tri[2]]
		total += a.Dot(b.Cross(c)) / 6
	}
	return total
}

// TestFreeformPrismProofRecordCoversItsOwnChording is
// docs/tessellation-reach-design.md §5's proof obligation, checked against the
// quantities it bounds rather than against its own formula: the chorded
// free-form wall's area slack must cover the area the chording actually loses,
// and volSymDiff must cover the volume it actually loses, both measured against
// the body's own exactly integrated readings.
func TestFreeformPrismProofRecordCoversItsOwnChording(t *testing.T) {
	t.Parallel()
	// Two heights, because the area the chording loses splits between terms
	// that scale differently: the two caps' loss is fixed by the section, and
	// the wall's loss grows with the sweep. A short prism is dominated by the
	// caps and a tall one by the wall, so covering both pins both terms.
	for _, height := range []float64{10, 2000} {
		t.Run(units.Millimeters(height).String(), func(t *testing.T) {
			body := internalFreeformArchBody(t, New(), height)
			mesh, err := tessellateContext(t.Context(), body, units.Millimeters(0.05))
			require.NoError(t, err)
			require.True(t, mesh.symDiffOK, `a chorded free-form prism still proves its occupied volume`)

			area, err := body.Area()
			require.NoError(t, err)
			held := meshHeldArea(mesh)
			require.Less(t, held, area.Value.Base(), `the chorded facets hold less area than the curved boundary`)
			require.GreaterOrEqual(t, mesh.areaSlack, area.Value.Base()-held,
				`the published area slack must cover the area the chording actually loses`)

			volume, err := body.Volume()
			require.NoError(t, err)
			heldVol := meshHeldVolume(mesh)
			require.Less(t, heldVol, volume.Value.Base(), `the inscribed chorded prism holds less volume than the curved one`)
			require.GreaterOrEqual(t, mesh.volSymDiff, volume.Value.Base()-heldVol,
				`the published occupied-volume bound must cover the volume the chording actually loses`)
		})
	}

	body := internalFreeformArchBody(t, New(), 10)
	mesh, err := tessellateContext(t.Context(), body, units.Millimeters(0.05))
	require.NoError(t, err)

	worst := 0.0
	for _, f := range mesh.source {
		d, ok := mesh.sourceBound(f)
		require.True(t, ok, `every source face is present in the proof record`)
		worst = math.Max(worst, d)
	}
	require.Equal(t, worst, mesh.bound, `Bound is the maximum sourceBound over the body`)

	// The free-form wall pays for its own chording; the straight closing wall
	// chords nothing and, under an unplaced axis-aligned payload, states an
	// exactly held polygon.
	var curved, straight float64
	for _, f := range body.Faces() {
		d, ok := mesh.sourceBound(f)
		require.True(t, ok)
		if _, isNURBS := f.Surface().(NURBSSurface); isNURBS {
			curved = d
			continue
		}
		if _, isPlane := f.Surface().(Plane); isPlane && roleOf(t, f) != roleCapStart && roleOf(t, f) != roleCapEnd {
			straight = d
		}
	}
	require.Positive(t, curved, `a chorded free-form wall never states an exact patch`)
	require.Zero(t, straight, `the straight closing wall chords nothing and stores nothing inexact`)
	require.Greater(t, curved, straight)

	// The curved wall's OWN published displacement must cover its OWN chording,
	// not merely be positive: no point of the recorded curve may sit farther
	// from that face's held chords than the face states. The wall is a vertical
	// ruled strip, so its facets' plane footprints ARE those chords and the
	// whole reading is 2D.
	deviation := freeformWallChordDeviation(t, body, mesh)
	require.Positive(t, deviation, `this fixture genuinely departs from its chords — the check is not vacuous`)
	require.GreaterOrEqual(t, curved, deviation,
		`the free-form wall's own sourceBound must cover the departure its own chords take`)
}

// freeformWallChordDeviation measures, over a dense sampling of the recorded
// free-form curve, the farthest any curve point sits from the chords the mesh
// holds for that wall — the quantity the wall face's own sourceBound must
// cover.
func freeformWallChordDeviation(t *testing.T, body *Body, mesh *Mesh) float64 {
	t.Helper()
	pp, ok := body.payload.(prismPayload)
	require.True(t, ok)

	var spans []bezierSpan
	for _, seg := range pp.profile.Outer.Segments {
		if _, isFree := seg.(FitSplineSeg); !isFree {
			continue
		}
		var err error
		spans, _, err = freeformBezierSpans(seg, newFreeformWork())
		require.NoError(t, err)
	}
	require.NotEmpty(t, spans, `the fixture's own free-form segment`)

	// The chords: each wall facet of the NURBS face projects onto exactly one
	// of them in the sketch plane.
	var chords [][2][2]float64
	for i, f := range mesh.source {
		if _, isNURBS := f.Surface().(NURBSSurface); !isNURBS {
			continue
		}
		tri := mesh.triangles[i]
		flat := map[[2]float64]struct{}{}
		for _, v := range tri {
			flat[[2]float64{mesh.vertices[v].X, mesh.vertices[v].Y}] = struct{}{}
		}
		ends := make([][2]float64, 0, 2)
		for p := range flat {
			ends = append(ends, p)
		}
		require.Len(t, ends, 2, `a vertical wall facet has exactly two distinct plane footprints`)
		chords = append(chords, [2][2]float64{ends[0], ends[1]})
	}
	require.NotEmpty(t, chords)

	worst := 0.0
	const samples = 5000
	for k := range samples + 1 {
		u, v := evalSpans(t, spans, float64(k)/samples)
		best := math.Inf(1)
		for _, c := range chords {
			best = math.Min(best, planeSegmentDistance([2]float64{u, v}, c[0], c[1]))
		}
		worst = math.Max(worst, best)
	}
	return worst
}

// planeSegmentDistance is the distance from p to the segment ab in the plane.
func planeSegmentDistance(p, a, b [2]float64) float64 {
	dx, dy := b[0]-a[0], b[1]-a[1]
	den := dx*dx + dy*dy
	s := 0.0
	if den > 0 {
		s = math.Max(0, math.Min(1, ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/den))
	}
	return math.Hypot(p[0]-(a[0]+s*dx), p[1]-(a[1]+s*dy))
}
