package apitest_test

import (
	"bytes"
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// fixedSketch builds a sketch on the XY plane with build, fixes every point
// it returns and solves it.
func fixedSketch(tb testing.TB, build func(s *sketch.Sketch) []*sketch.Point) (*sketch.Sketch, *sketch.Profile) {
	tb.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(tb, err)
	for _, p := range build(s) {
		s.Fix(p)
	}
	_, err = s.Solve(tb.Context())
	require.NoError(tb, err)
	// The region that carries holes, when there is one; otherwise the one
	// region the sketch states.
	profiles := s.Profiles()
	require.NotEmpty(tb, profiles)
	for _, p := range profiles {
		if len(p.Holes) > 0 {
			return s, p
		}
	}
	require.Len(tb, profiles, 1)
	return s, profiles[0]
}

// crossCentreSketch is the centre cell of two overlapping rectangles: every
// one of its edges is a line trimmed at both ends.
func crossCentreSketch(tb testing.TB) (*sketch.Sketch, *sketch.Profile) {
	tb.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(tb, err)
	a := s.CreateRectangle(-2, -1, 2, 1)
	b := s.CreateRectangle(-1, -2, 1, 2)
	for _, p := range []*sketch.Point{a.A, a.B, a.C, a.D, b.A, b.B, b.C, b.D} {
		s.Fix(p)
	}
	_, err = s.Solve(tb.Context())
	require.NoError(tb, err)
	for _, p := range s.Profiles() {
		trimmed := true
		for _, e := range p.Outer {
			trimmed = trimmed && e.Partial && e.TStart != 0 && e.TEnd != 1
		}
		if trimmed {
			return s, p
		}
	}
	tb.Fatal(`the overlapping rectangles state no centre cell`)
	return nil, nil
}

// squareSweepSketch is a square of side a centred on the origin.
func squareSweepSketch(tb testing.TB, a float64) (*sketch.Sketch, *sketch.Profile) {
	tb.Helper()
	return fixedSketch(tb, func(s *sketch.Sketch) []*sketch.Point {
		r := s.CreateRectangle(-a/2, -a/2, a/2, a/2)
		return []*sketch.Point{r.A, r.B, r.C, r.D}
	})
}

// polygonSweepSketch is a regular n-gon of circumradius r centred on the
// origin, as mtilt sketches a branch section.
func polygonSweepSketch(tb testing.TB, n int, r float64) (*sketch.Sketch, *sketch.Profile) {
	tb.Helper()
	return fixedSketch(tb, func(s *sketch.Sketch) []*sketch.Point {
		poly, err := s.CreatePolygon(0, 0, n, r)
		require.NoError(tb, err)
		return append([]*sketch.Point{poly.Center}, poly.Vertices...)
	})
}

func mustPath(tb testing.TB, points ...r3.Vec) *decad.Path {
	tb.Helper()
	segments := make([]decad.PathSegment, 0, len(points)-1)
	for _, p := range points[1:] {
		segments = append(segments, decad.LineTo{End: p})
	}
	path, err := decad.NewPath(points[0], segments...)
	require.NoError(tb, err)
	return path
}

func scalars(values ...float64) []units.Value {
	out := make([]units.Value, len(values))
	for i, v := range values {
		out[i] = units.Scalar(v)
	}
	return out
}

func faceRoles(body *decad.Body) []string {
	var roles []string
	for _, f := range body.Faces() {
		for _, o := range f.Origins() {
			roles = append(roles, o.Role)
		}
	}
	return roles
}

func requireSweepSound(t *testing.T, doc *decad.Document, body *decad.Body) *decad.BodyReport {
	t.Helper()
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	br, err := report.ForBody(body)
	require.NoError(t, err)
	require.Equal(t, decad.ValidityValid, br.Validity.Outcome)
	require.Equal(t, 1, br.Topology.Lumps)
	require.NotNil(t, br.Region)
	return br
}

func TestSweepMitredLPath(t *testing.T) {
	t.Parallel()
	const a, l1, l2 = 2.0, 10.0, 6.0
	s, profile := squareSweepSketch(t, a)
	doc := decad.New()
	body, err := doc.Sweep(t.Context(), s, profile,
		mustPath(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, l1), r3.NewVec(l2, 0, l1)),
		decad.WithMitredJoins())
	require.NoError(t, err)

	decadtest.MeasuresVolume(t, body, units.CubicMillimeters(a*a*(l1+l2)), decadtest.Exactly())
	require.Len(t, body.Faces(), 2+8)
	require.Len(t, body.Vertices(), 12)
	require.Len(t, body.Edges(), 20)
	for _, e := range body.Edges() {
		require.Len(t, e.Faces(), 2)
	}
	require.ElementsMatch(t, []string{
		"capStart", "capEnd",
		"side(0,0,0)", "side(0,0,1)", "side(0,0,2)", "side(0,0,3)",
		"side(1,0,0)", "side(1,0,1)", "side(1,0,2)", "side(1,0,3)",
	}, faceRoles(body))
	decadtest.HasSurfaceKinds(t, body, map[decad.SurfaceKind]int{decad.KindPlane: 10})
	for _, f := range body.Faces() {
		require.Len(t, f.Loops(), 1)
		if strings.HasPrefix(f.Origins()[0].Role, "side(") {
			require.Len(t, f.Loops()[0].Edges(), 4, `a wall is one four-edge face`)
		}
	}

	// The join section lies on the bisecting plane x = l1 − z through
	// (0, 0, l1) and spans a rectangle of sides a and a·√2.
	var join []r3.Vec
	for _, v := range body.Vertices() {
		p := v.Position().Value
		if p.Z != 0 && p.X != l2 {
			join = append(join, p)
		}
	}
	require.Len(t, join, 4)
	for _, p := range join {
		require.Equal(t, l1-p.Z, p.X)
		require.Equal(t, a/2, math.Abs(p.Y))
		require.Equal(t, a/2, math.Abs(p.X))
	}

	// One join-section edge sits on the inside of the bend; the two
	// coplanar side walls meet flat across the join.
	convex, concave := 0, 0
	for _, e := range body.Edges() {
		if e.IsConvex() {
			convex++
		} else {
			concave++
		}
	}
	require.Equal(t, 17, convex)
	require.Equal(t, 3, concave)

	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Zero(t, mesh.Bound().Base(), `every vertex of this L is a float`)
	require.True(t, mesh.VolumeVerified())
	requireWatertight(t, mesh)
	require.Equal(t, a*a*(l1+l2), meshVolume(mesh))
	requireSweepSound(t, doc, body)
}

func TestSweepMitredOneSpanFrustum(t *testing.T) {
	t.Parallel()
	const a, l, f = 2.0, 3.0, 0.5
	s, profile := squareSweepSketch(t, a)
	doc := decad.New()
	body, err := doc.Sweep(t.Context(), s, profile,
		mustPath(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, l)),
		decad.WithSectionScale(units.Scalar(f)))
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, body, units.CubicMillimeters(l*a*a*(1+f+f*f)/3), decadtest.Exactly())
	require.Len(t, body.Faces(), 6)
	for _, v := range body.Vertices() {
		p := v.Position().Value
		switch p.Z {
		case 0:
			require.Equal(t, a/2, math.Abs(p.X))
			require.Equal(t, a/2, math.Abs(p.Y))
		case l:
			require.Equal(t, f*a/2, math.Abs(p.X), `a capEnd vertex sits at f times its start offset`)
			require.Equal(t, f*a/2, math.Abs(p.Y))
		default:
			t.Fatalf(`vertex %v is on neither cap`, p)
		}
	}
	decadtest.MeasuresBounds(t, body, r3.NewVec(-1, -1, 0), r3.NewVec(1, 1, l), decadtest.Exactly())
	requireSweepSound(t, doc, body)
}

func TestSweepMitredCollinearFrustumChain(t *testing.T) {
	t.Parallel()
	// Two collinear spans scaled 1 → 0.5 → 0.25: the join plane is the
	// perpendicular one, so the body is the analytic frustum chain
	// Σ L_k·A_k·(1 + ρ_k + ρ_k²)/3.
	s, profile := squareSweepSketch(t, 2)
	doc := decad.New()
	body, err := doc.Sweep(t.Context(), s, profile,
		mustPath(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 3), r3.NewVec(0, 0, 9)),
		decad.WithMitredJoins(), decad.WithSectionScale(scalars(0.5, 0.25)...))
	require.NoError(t, err)
	want := 3*4*(1+0.5+0.25)/3 + 6*1*(1+0.5+0.25)/3
	decadtest.MeasuresVolume(t, body, units.CubicMillimeters(want), decadtest.Exactly())
	requireSweepSound(t, doc, body)
}

func TestSweepMitredHoleIsVoidPassage(t *testing.T) {
	t.Parallel()
	s, profile := fixedSketch(t, func(s *sketch.Sketch) []*sketch.Point {
		outer := s.CreateRectangle(-2, -2, 2, 2)
		inner := s.CreateRectangle(-1, -1, 1, 1)
		return []*sketch.Point{outer.A, outer.B, outer.C, outer.D, inner.A, inner.B, inner.C, inner.D}
	})
	doc := decad.New()
	body, err := doc.Sweep(t.Context(), s, profile,
		mustPath(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 10), r3.NewVec(6, 0, 10)),
		decad.WithMitredJoins())
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, body, units.CubicMillimeters((16-4)*16), decadtest.Exactly())
	require.Len(t, body.Faces(), 2+2*8)
	require.Contains(t, faceRoles(body), "side(1,1,3)")
	// The outer tube reads as the plain L does, 17 of its 20 edges convex.
	// The hole tube's walls face the passage, so its rims are concave and
	// one of its 20 edges, on the outside of its bend, reads convex.
	convex := 0
	for _, e := range body.Edges() {
		if e.IsConvex() {
			convex++
		}
	}
	require.Len(t, body.Edges(), 40)
	require.Equal(t, 17+1, convex)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	report := requireSweepSound(t, doc, body)
	require.Equal(t, 0, report.Topology.Voids, `a hole is a passage, never a void shell`)
}

// treeBranchPath leans 0°, 14°, 30° and 45° from the build direction, as one
// mtilt support branch does.
func treeBranchPath(tb testing.TB) *decad.Path {
	tb.Helper()
	return mustPath(tb,
		r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 4), r3.NewVec(1, 0, 8),
		r3.NewVec(3, 0.5, 11.5), r3.NewVec(6, 0.5, 14.5))
}

// treeBranchFactors taper the radius 1.5 → 1 mm linearly over the path points.
func treeBranchFactors() []units.Value {
	return scalars(1.375/1.5, 1.25/1.5, 1.125/1.5, 1/1.5)
}

func TestSweepMitredTreeBranch(t *testing.T) {
	t.Parallel()
	s, profile := polygonSweepSketch(t, 16, 1.5)
	doc := decad.New()
	body, err := doc.Sweep(t.Context(), s, profile, treeBranchPath(t),
		decad.WithMitredJoins(), decad.WithSectionScale(treeBranchFactors()...))
	require.NoError(t, err)

	require.Len(t, body.Faces(), 2+4*16)
	require.Len(t, body.Vertices(), 5*16)
	require.Len(t, body.Edges(), 9*16)
	decadtest.IsManifold(t, body)

	// The frustum chain over the four span lengths with perpendicular ends
	// is the body this approximates; the mitres trade volume between
	// neighbouring spans, so the two agree to a fraction of a percent.
	area := func(r float64) float64 { return 8 * r * r * math.Sin(math.Pi/8) }
	radii := []float64{1.5, 1.375, 1.25, 1.125, 1}
	lengths := []float64{4, math.Sqrt(17), math.Sqrt(16.5), math.Sqrt(18)}
	chain := 0.0
	for k, length := range lengths {
		rho := radii[k+1] / radii[k]
		chain += length * area(radii[k]) * (1 + rho + rho*rho) / 3
	}
	vol, err := body.Volume()
	require.NoError(t, err)
	require.InEpsilon(t, chain, vol.Value.Base(), 2e-3)
	require.LessOrEqual(t, vol.Bound.Base(), math.Nextafter(vol.Value.Base(), math.Inf(1))-vol.Value.Base(),
		`the volume carries one rounding of an exact rational at most`)

	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.01), decad.WithVerification(decad.VerifyBoundary))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.Positive(t, mesh.Bound().Base())
	require.Less(t, mesh.Bound().Base(), 1e-12)
	requireWatertight(t, mesh)
	requireSweepSound(t, doc, body)
}

// parseSTLMesh welds an ASCII STL's facet corners by their printed
// coordinates and returns the facets as index triples.
func parseSTLMesh(t *testing.T, stl string) [][3]int {
	t.Helper()
	index := map[string]int{}
	var corners []int
	for line := range strings.SplitSeq(stl, "\n") {
		fields := strings.Fields(line)
		if len(fields) != 4 || fields[0] != "vertex" {
			continue
		}
		key := strings.Join(fields[1:], " ")
		if _, ok := index[key]; !ok {
			index[key] = len(index)
		}
		corners = append(corners, index[key])
	}
	require.Zero(t, len(corners)%3)
	tris := make([][3]int, 0, len(corners)/3)
	for i := 0; i < len(corners); i += 3 {
		tris = append(tris, [3]int{corners[i], corners[i+1], corners[i+2]})
	}
	return tris
}

func TestSweepMitredZigzagRestatesAndPlaces(t *testing.T) {
	t.Parallel()
	s, profile := polygonSweepSketch(t, 16, 1.5)
	doc := decad.New()
	body, err := doc.Sweep(t.Context(), s, profile,
		mustPath(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 5), r3.NewVec(3, 0, 9), r3.NewVec(0, 1, 13)),
		decad.WithMitredJoins(), decad.WithSectionScale(scalars(1.3/1.5, 1.1/1.5, 1/1.5)...))
	require.NoError(t, err)
	vol, err := body.Volume()
	require.NoError(t, err)

	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.01), decad.WithVerification(decad.VerifyBoundary))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	maxGap := 0.0
	for _, v := range body.Vertices() {
		maxGap = math.Max(maxGap, v.Position().Bound.Base())
	}
	require.Equal(t, maxGap, mesh.Bound().Base(), `the mesh bound is the payload's own delta`)
	requireWatertight(t, mesh)
	for i, f := range mesh.SourceFaces() {
		require.NotNil(t, f, `triangle %d has a source face`, i)
	}

	var buf bytes.Buffer
	require.NoError(t, export.STL(t.Context(), &buf, body, units.Millimeters(0.01)))
	stlTris := parseSTLMesh(t, buf.String())
	require.Len(t, stlTris, len(mesh.Triangles()))
	directed := map[[2]int]int{}
	for _, tri := range stlTris {
		for k := range 3 {
			directed[[2]int{tri[k], tri[(k+1)%3]}]++
		}
	}
	for e, n := range directed {
		require.Equal(t, 1, n)
		require.Equal(t, 1, directed[[2]int{e[1], e[0]}], `STL edge %v is watertight`, e)
	}

	quarter, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(-1, 0, 0))
	require.NoError(t, err)
	turn, err := r3.FromFrame(quarter)
	require.NoError(t, err)
	placed, err := body.Placed(t.Context(), turn)
	require.NoError(t, err)
	placedVol, err := placed.Volume()
	require.NoError(t, err)
	require.Equal(t, vol, placedVol, `an exact quarter turn reproduces the volume exactly`)
	requireSweepSound(t, doc, placed)

	mirror, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	reflect, err := r3.Reflection(mirror)
	require.NoError(t, err)
	mirrored, err := placed.Placed(t.Context(), reflect)
	require.NoError(t, err)
	mirroredVol, err := mirrored.Volume()
	require.NoError(t, err)
	require.Equal(t, vol, mirroredVol, `a mirror re-orients the shell once and keeps the volume`)
	mirroredMesh, err := mirrored.Tessellate(t.Context(), units.Millimeters(0.01))
	require.NoError(t, err)
	requireWatertight(t, mirroredMesh)
	require.Positive(t, meshVolume(mirroredMesh))
	requireSweepSound(t, doc, mirrored)

	tilt, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(30))
	require.NoError(t, err)
	tilted, err := mirrored.Placed(t.Context(), tilt)
	require.NoError(t, err)
	tiltedVol, err := tilted.Volume()
	require.NoError(t, err)
	require.InEpsilon(t, vol.Value.Base(), tiltedVol.Value.Base(), 1e-14)
	requireSweepSound(t, doc, tilted)
}

func TestSweepMitredTessellationTolerance(t *testing.T) {
	t.Parallel()
	s, profile := polygonSweepSketch(t, 16, 1.5)
	doc := decad.New()
	body, err := doc.Sweep(t.Context(), s, profile, treeBranchPath(t),
		decad.WithMitredJoins(), decad.WithSectionScale(treeBranchFactors()...))
	require.NoError(t, err)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(1))
	require.NoError(t, err)
	_, err = body.Tessellate(t.Context(), units.Millimeters(mesh.Bound().Base()/2))
	require.ErrorIs(t, err, decad.ErrUnsupported, `a tolerance finer than the held vertices is refused`)
}

// TestSweepMitredRefusals covers every row of docs/sweep-design.md Table SM
// a public call can reach, plus S12's repeated options. Each refusal leaves
// the document's live set and next producer identity unchanged.
func TestSweepMitredRefusals(t *testing.T) {
	t.Parallel()
	square := func(tb testing.TB) (*sketch.Sketch, *sketch.Profile) { return squareSweepSketch(tb, 2) }
	up := mustPath(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 10))
	ell := mustPath(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 10), r3.NewVec(6, 0, 10))
	arcPath, err := decad.NewPath(r3.NewVec(0, 0, 0),
		decad.LineTo{End: r3.NewVec(0, 0, 10)},
		decad.ArcThrough{Through: r3.NewVec(3, 0, 13), End: r3.NewVec(6, 0, 10)})
	require.NoError(t, err)
	circle := func(tb testing.TB) (*sketch.Sketch, *sketch.Profile) {
		return fixedSketch(tb, func(s *sketch.Sketch) []*sketch.Point {
			c := s.CreatePoint(0, 0)
			s.CreateCircle(c, 1)
			return []*sketch.Point{c}
		})
	}
	zigzag := []r3.Vec{r3.NewVec(0, 0, 0)}
	for k := 1; k <= 257; k++ {
		zigzag = append(zigzag, r3.NewVec(float64((k+1)%2), 0, float64(10*k)))
	}
	mitre := []decad.SweepOption{decad.WithMitredJoins()}

	cases := []struct {
		name    string
		profile func(testing.TB) (*sketch.Sketch, *sketch.Profile)
		path    *decad.Path
		opts    []decad.SweepOption
		want    error
		message string
	}{
		{"SM1 arc span", square, arcPath, mitre, decad.ErrUnsupported, "LineTo"},
		{"SM1 arc span scaled", square, arcPath, []decad.SweepOption{decad.WithMitredJoins(), decad.WithSectionScale(scalars(1, 1)...)}, decad.ErrUnsupported, "LineTo"},
		{"SM2 circle profile", circle, up, mitre, decad.ErrUnsupported, "line profile segments"},
		{"SM2 trimmed lines", crossCentreSketch, up, mitre, decad.ErrUnsupported, "trimmed"},
		{"S15 rounding collapses a cap", func(tb testing.TB) (*sketch.Sketch, *sketch.Profile) {
			return fixedSketch(tb, func(s *sketch.Sketch) []*sketch.Point {
				r := s.CreateRectangle(999, 999, 1001, 1001)
				return []*sketch.Point{r.A, r.B, r.C, r.D}
			})
		}, mustPath(t, r3.NewVec(1000, 1000, 0), r3.NewVec(1000, 1000, 10)), []decad.SweepOption{decad.WithSectionScale(units.Scalar(1e-17))}, decad.ErrUnsupported, "rounding the mitred sweep's vertices collapsed"},
		{"SM3 wrong kind", square, up, []decad.SweepOption{decad.WithSectionScale(units.Millimeters(1))}, decad.ErrUnitKind, "dimensionless"},
		{"SM3 non-finite", square, up, []decad.SweepOption{decad.WithSectionScale(units.Scalar(math.NaN()))}, decad.ErrNotFinite, "not finite"},
		{"SM3 zero", square, up, []decad.SweepOption{decad.WithSectionScale(units.Scalar(0))}, decad.ErrDegenerate, "must be positive"},
		{"SM3 negative", square, up, []decad.SweepOption{decad.WithSectionScale(units.Scalar(-1))}, decad.ErrDegenerate, "must be positive"},
		{"SM3 count", square, ell, []decad.SweepOption{decad.WithMitredJoins(), decad.WithSectionScale(units.Scalar(1))}, decad.ErrDegenerate, "1 factors for a path of 2"},
		{"SM4 scale without mitre", square, ell, []decad.SweepOption{decad.WithSectionScale(scalars(1, 1)...)}, decad.ErrUnsupported, "requires WithMitredJoins"},
		{"SM5 reversed spans", square, mustPath(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 5), r3.NewVec(0, 0, 2)), mitre, decad.ErrDegenerate, "run exactly back"},
		{"SM6 join plane behind span", square, mustPath(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1), r3.NewVec(1e-300, 0, 0)), mitre, decad.ErrDegenerate, "does not reach its end section plane forward"},
		{"SM6 wall line parallel to join plane", func(tb testing.TB) (*sketch.Sketch, *sketch.Profile) { return squareSweepSketch(tb, 4) }, mustPath(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1), r3.NewVec(-10, 0, 1)), []decad.SweepOption{decad.WithMitredJoins(), decad.WithSectionScale(scalars(0.5, 0.5)...)}, decad.ErrDegenerate, "parallel to the end section plane"},
		{"SM9 surface result", square, ell, []decad.SweepOption{decad.WithMitredJoins(), decad.WithSurfaceResult()}, decad.ErrUnsupported, "SM9"},
		{"SM9 closed path", square, mustPath(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 10), r3.NewVec(10, 0, 10), r3.NewVec(10, 0, 0), r3.NewVec(0, 0, 0)), mitre, decad.ErrUnsupported, "closed"},
		{"SM10 span ceiling", square, mustPath(t, zigzag...), mitre, decad.ErrUnsupported, "span ceiling"},
		{"SM10 facet pairs", func(tb testing.TB) (*sketch.Sketch, *sketch.Profile) { return polygonSweepSketch(tb, 64, 2) }, mustPath(t, zigzag[:41]...), mitre, decad.ErrUnsupported, "facet-pair ceiling"},
		{"S12 repeated mitre", square, ell, []decad.SweepOption{decad.WithMitredJoins(), decad.WithMitredJoins()}, decad.ErrDegenerate, "more than once"},
		{"S12 repeated scale", square, up, []decad.SweepOption{decad.WithSectionScale(units.Scalar(1)), decad.WithSectionScale(units.Scalar(1))}, decad.ErrDegenerate, "more than once"},
		{"S5 off-plane start", square, mustPath(t, r3.NewVec(0, 0, 1), r3.NewVec(0, 0, 10), r3.NewVec(6, 0, 10)), mitre, decad.ErrDegenerate, "start in the profile plane"},
		{"S6 corner without mitre", square, ell, nil, decad.ErrUnsupported, "WithMitredJoins admits a corner"},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			s, profile := tc.profile(t)
			doc := decad.New()
			before := doc.Bodies()
			_, err := doc.Sweep(t.Context(), s, profile, tc.path, tc.opts...)
			require.ErrorIs(t, err, tc.want)
			require.ErrorContains(t, err, tc.message)
			require.Equal(t, before, doc.Bodies())
			requireNextProducerUnchanged(t, doc)
		})
	}
}

// requireNextProducerUnchanged builds one plain sweep in doc and in a fresh
// document: equal provenance proves the earlier refusal minted no producer.
func requireNextProducerUnchanged(t *testing.T, doc *decad.Document) {
	t.Helper()
	s, profile := squareSweepSketch(t, 2)
	path := mustPath(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1))
	got, err := doc.Sweep(t.Context(), s, profile, path, decad.WithMitredJoins())
	require.NoError(t, err)
	want, err := decad.New().Sweep(t.Context(), s, profile, path, decad.WithMitredJoins())
	require.NoError(t, err)
	require.Equal(t, want.Faces()[0].Origins(), got.Faces()[0].Origins())
}

func TestSweepMitredSharpTaperedBendNamesSpanAndVertex(t *testing.T) {
	t.Parallel()
	// A 1 mm span tapered to a tenth has its apex 1/0.9 mm out. The 45° join
	// plane of a bend toward −x passes 2.5 mm beyond the section's inner
	// side, farther out than the apex, so that side's wall lines meet it
	// only past the apex. Untapered, the same side's wall lines meet it
	// behind the section.
	s, profile := squareSweepSketch(t, 3)
	bend := mustPath(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1), r3.NewVec(-10, 0, 1))
	doc := decad.New()
	_, err := doc.Sweep(t.Context(), s, profile, bend,
		decad.WithMitredJoins(), decad.WithSectionScale(scalars(0.1, 0.1)...))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.ErrorContains(t, err, "span 0, loop 0, vertex")
	require.ErrorContains(t, err, "past the span's apex")
	require.Empty(t, doc.Bodies())

	_, err = doc.Sweep(t.Context(), s, profile, bend, decad.WithMitredJoins())
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.ErrorContains(t, err, "span 0, loop 0, vertex")
	require.ErrorContains(t, err, "behind its start")
	require.Empty(t, doc.Bodies())
}

func TestSweepMitredSelfCrossingNamesFaces(t *testing.T) {
	t.Parallel()
	s, profile := squareSweepSketch(t, 2)
	doc := decad.New()
	_, err := doc.Sweep(t.Context(), s, profile,
		mustPath(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 10), r3.NewVec(10, 0, 10),
			r3.NewVec(10, 0, 5), r3.NewVec(-5, 0, 5)),
		decad.WithMitredJoins())
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.ErrorContains(t, err, "mitred sweep faces side(0,")
	require.ErrorContains(t, err, "side(3,")
	require.Empty(t, doc.Bodies())
}
