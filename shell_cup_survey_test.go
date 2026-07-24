package decad_test

import (
	"bytes"
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// cylinderCup extrudes a disk of radius R to height h and shells its top cap
// inward by th — a cup with a cylindrical outer wall, a cylindrical cavity wall
// of radius R − th, and a circular floor.
func cylinderCup(t *testing.T, R, h, th float64) (*decad.Document, *decad.Body) {
	t.Helper()
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	require.NoError(t, err)
	s.CreateCircle(s.CreatePoint(0, 0), R)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	doc := decad.New()
	disk, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	cup, err := disk.Shell(topCap(disk), units.Millimeters(th))
	require.NoError(t, err)
	return doc, cup
}

// polyArea is the area of the regular n-gon inscribed in a circle of radius r —
// the exact area a cup's chorded circular region encloses.
func polyArea(r float64, n int) float64 {
	return float64(n) / 2 * r * r * math.Sin(2*math.Pi/float64(n))
}

func requireCupWall(t *testing.T, doc *decad.Document, tool, want float64, status decad.Status) {
	t.Helper()
	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(tool)))
	require.NoError(t, err)
	require.Len(t, report.Bodies, 1)
	requireWall(t, report.Bodies[0], want)
	if status == decad.Sound {
		status = decad.Suspect
	}
	require.Equal(t, status, report.Bodies[0].Status)
	require.Equal(t, status, report.Status)
}

func TestShellCupWallContextCancellationDuringOffset(t *testing.T) {
	doc, box := shellBox(t)
	_, err := box.Shell(topCap(box), units.Millimeters(5))
	require.NoError(t, err)

	ctx := &offsetPreprocessingCancelContext{Context: t.Context()}
	report, err := doc.Verify(ctx, decad.WithMinWallThickness(units.Millimeters(1)))

	require.Nil(t, report)
	require.ErrorIs(t, err, context.Canceled)
	require.True(t, ctx.entered)
}

func TestShellCupWallThickness(t *testing.T) {
	const th = 5.0

	t.Run("inward box", func(t *testing.T) {
		doc, box := shellBox(t)
		_, err := box.Shell(topCap(box), units.Millimeters(th))
		require.NoError(t, err)
		requireCupWall(t, doc, th, th, decad.Sound)
	})

	t.Run("bottom-open mirror", func(t *testing.T) {
		const mirrorThickness = 0.1
		doc, box := shellBox(t)
		bottom := decad.Faces(decad.FaceCreatedBy(decad.CapStart(box)))
		_, err := box.Shell(bottom, units.Millimeters(mirrorThickness))
		require.NoError(t, err)
		requireCupWall(t, doc, mirrorThickness, mirrorThickness, decad.Sound)
	})

	t.Run("outward bottom-open mirror", func(t *testing.T) {
		const mirrorThickness = 0.1
		doc, box := shellBox(t)
		bottom := decad.Faces(decad.FaceCreatedBy(decad.CapStart(box)))
		_, err := box.Shell(
			bottom,
			units.Millimeters(mirrorThickness),
			decad.WithShellSense(decad.Outward),
		)
		require.NoError(t, err)
		requireCupWall(t, doc, mirrorThickness, mirrorThickness, decad.Sound)
	})

	t.Run("cylinder", func(t *testing.T) {
		doc, _ := cylinderCup(t, 20, 12, th)
		requireCupWall(t, doc, th, th, decad.Sound)
	})

	t.Run("outward rounded box", func(t *testing.T) {
		doc, box := shellBox(t)
		_, err := box.Shell(topCap(box), units.Millimeters(th), decad.WithShellSense(decad.Outward))
		require.NoError(t, err)
		requireCupWall(t, doc, th, th, decad.Sound)
	})

	t.Run("holed cup", func(t *testing.T) {
		doc, box := circleHoledBox(t, [3]float64{50, 30, 8})
		_, err := box.Shell(topCap(box), units.Millimeters(th))
		require.NoError(t, err)
		requireCupWall(t, doc, th, th, decad.Sound)
	})
}

func TestShellCupWallToolVerdict(t *testing.T) {
	const th = 5.0
	doc, box := shellBox(t)
	_, err := box.Shell(topCap(box), units.Millimeters(th))
	require.NoError(t, err)

	requireCupWall(t, doc, th-1, th, decad.Sound)
	requireCupWall(t, doc, th, th, decad.Sound)
	requireCupWall(t, doc, th+1, th, decad.Violating)
}

func TestShellCupWallQualifyingPinch(t *testing.T) {
	// The narrow right triangle has a roughly 5.7 degree material corner.
	// Under the default 15 degree allowance that junction admits spanning
	// balls tending to zero, so the wall is exactly zero and every legal tool
	// proves a violation.
	s, p := polygonSketch(t, [][2]float64{{0, 0}, {100, 0}, {100, 10}})
	doc := decad.New()
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)
	_, err = body.Shell(topCap(body), units.Millimeters(1))
	require.NoError(t, err)
	requireCupWall(t, doc, 0.001, 0, decad.Violating)

	// Below the actual corner angle, the junction is an edge rather than a
	// wall pinch, so the exact shell thickness is the answer.
	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(
		units.Millimeters(1),
		decad.WithDraftAllowance(units.Degrees(5)),
	))
	require.NoError(t, err)
	requireWall(t, report.Bodies[0], 1)
	require.Equal(t, decad.Suspect, report.Status)
}

func TestShellCupWallOutwardCavityQualifyingPinch(t *testing.T) {
	const th = 0.3
	// The V-notch has a roughly 7.15 degree apex. An outward shell keeps that
	// notch on the cavity boundary, where its material-side corner qualifies
	// as a wall pinch under the default 15 degree allowance.
	s, p := polygonSketch(t, [][2]float64{
		{0, 0},
		{200, 0},
		{200, 100},
		{105, 100},
		{100, 20},
		{95, 100},
		{0, 100},
	})
	doc := decad.New()
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)
	_, err = body.Shell(
		topCap(body),
		units.Millimeters(th),
		decad.WithShellSense(decad.Outward),
	)
	require.NoError(t, err)
	requireCupWall(t, doc, th, 0, decad.Violating)
}

func TestShellCupTessellateBox(t *testing.T) {
	// A box cup is all planar: it triangulates exactly (bound zero), its mesh
	// is watertight, and the enclosed volume is the cup's own — proof the three
	// caps and both wall bands are wound outward and share their rings.
	const th = 5.0
	h := shellBoxHeight
	_, box := shellBox(t)
	cup, err := box.Shell(topCap(box), units.Millimeters(th))
	require.NoError(t, err)

	mesh, err := cup.Tessellate(units.Millimeters(0.1))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	require.True(t, mesh.Bound().Equal(units.Millimeters(0), 1e-12), `a box cup chords nothing, got %s`, mesh.Bound())

	aP := 100.0 * 60.0
	aQ := (100 - 2*th) * (60 - 2*th)
	wantVol := aP*h - aQ*(h-th)
	require.InDelta(t, wantVol, meshVolume(mesh), 1e-9, `the mesh encloses exactly the cup volume`)

	// Every facet's source is a live face, and all eleven faces are covered.
	live := map[*decad.Face]struct{}{}
	for _, f := range cup.Faces() {
		live[f] = struct{}{}
	}
	covered := map[*decad.Face]struct{}{}
	for _, f := range mesh.SourceFaces() {
		_, ok := live[f]
		require.True(t, ok, `a facet's source is one of the cup's own faces`)
		covered[f] = struct{}{}
	}
	require.Len(t, covered, 11, `every cup face carries at least one facet`)

	// STL and OBJ round-trip deterministically — the writers are byte-stable.
	var stl1, stl2 bytes.Buffer
	require.NoError(t, cup.STL(&stl1))
	require.NoError(t, cup.STL(&stl2))
	require.Equal(t, stl1.Bytes(), stl2.Bytes(), `STL export is deterministic`)
	require.Positive(t, stl1.Len())
	var obj1, obj2 bytes.Buffer
	require.NoError(t, cup.OBJ(&obj1))
	require.NoError(t, cup.OBJ(&obj2))
	require.Equal(t, obj1.Bytes(), obj2.Bytes(), `OBJ export is deterministic`)
	require.Positive(t, obj1.Len())
}

func TestShellCupTessellateCylinder(t *testing.T) {
	// A cylindrical cup: outer and cavity walls chord their circles, and the
	// cap and rim share that chording. The mesh is watertight, its bound is a
	// positive sagitta within tolerance, and its volume is exactly the chorded
	// outer prism less the chorded cavity prism.
	const R, h, th = 20.0, 12.0, 4.0
	tol := 0.5
	_, cup := cylinderCup(t, R, h, th)

	mesh, err := cup.Tessellate(units.Millimeters(tol))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	require.Positive(t, mesh.Bound().Mag(), `a chorded circle carries a real sagitta`)
	require.LessOrEqual(t, mesh.Bound().Mag(), tol)

	// Separate the two rings by their radius from the axis: the outer ring sits
	// at R, the cavity ring at R − th. Each sample owns two vertices.
	nO, nC := 0, 0
	for _, v := range mesh.Vertices() {
		switch rho := math.Hypot(v.X, v.Y); {
		case math.Abs(rho-R) < 1e-6:
			nO++
		case math.Abs(rho-(R-th)) < 1e-6:
			nC++
		default:
			t.Fatalf("a cup vertex sits off both rings, ρ = %g", rho)
		}
	}
	require.Positive(t, nO)
	require.Positive(t, nC)
	nO /= 2
	nC /= 2

	// The outer prism over [0, h] less the cavity prism over [th, h], each on
	// its inscribed polygon — exact.
	wantVol := polyArea(R, nO)*h - polyArea(R-th, nC)*(h-th)
	require.InDelta(t, wantVol, meshVolume(mesh), 1e-9)

	// And the chorded volume converges on the true cup as the tolerance shrinks.
	trueVol := math.Pi*R*R*h - math.Pi*(R-th)*(R-th)*(h-th)
	require.InDelta(t, trueVol, meshVolume(mesh), trueVol*0.05, `the chorded cup approaches the analytic one`)
}

func TestShellCupTessellateReflected(t *testing.T) {
	// A reflected placement flips handedness; the cup mesh must still come out
	// wound outward, enclosing the same positive volume.
	const th = 5.0
	h := shellBoxHeight
	_, box := shellBox(t)
	cup, err := box.Shell(topCap(box), units.Millimeters(th))
	require.NoError(t, err)

	mirror, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	refl, err := r3.Reflection(mirror)
	require.NoError(t, err)
	placed, err := cup.Placed(refl)
	require.NoError(t, err)

	mesh, err := placed.Tessellate(units.Millimeters(0.1))
	require.NoError(t, err)
	requireWatertight(t, mesh)

	aP := 100.0 * 60.0
	aQ := (100 - 2*th) * (60 - 2*th)
	wantVol := aP*h - aQ*(h-th)
	require.InDelta(t, wantVol, meshVolume(mesh), 1e-9, `outward winding survives a reflected placement`)
}

func TestShellCupTessellateOutwardBox(t *testing.T) {
	// An outward box cup dilates its rectangular section, landing the rounded
	// outer corners' tangent points on the cavity's own top and bottom edge
	// lines — the bridge-collinear rim band. It is a normal 5 mm-wide frame
	// (rounded outer, sharp inner), so it triangulates into a watertight mesh;
	// the only chorded feature is the four dilated outer corners, over the
	// outer prism's full height h + th.
	const th = 5.0
	h := shellBoxHeight
	_, box := shellBox(t)
	cup, err := box.Shell(topCap(box), units.Millimeters(th), decad.WithShellSense(decad.Outward))
	require.NoError(t, err)

	tol := 0.1
	mesh, err := cup.Tessellate(units.Millimeters(tol))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	require.LessOrEqual(t, mesh.Bound().Mag(), tol)

	vol, err := cup.Volume()
	require.NoError(t, err)
	exact, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	slack := 2 * math.Pi * th * mesh.Bound().Mag() * (h + th)
	require.InDelta(t, exact, meshVolume(mesh), slack, `the mesh encloses the cup volume up to the chorded outer corners`)
}

func TestShellCupUndercutsBox(t *testing.T) {
	const th = 5.0

	// Pulled straight out of its open top, a box cup has no undercut: every
	// wall is exactly perpendicular, the pocket floor and rim face the pull,
	// and the outer floor is exactly antiparallel (it separates, not hooks).
	t.Run("clear under an axial pull", func(t *testing.T) {
		doc, box := shellBox(t)
		_, err := box.Shell(topCap(box), units.Millimeters(th))
		require.NoError(t, err)
		report, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(0, 0, 1)))
		require.NoError(t, err)
		br := report.Bodies[0]
		require.NotNil(t, br.Undercuts)
		require.Empty(t, br.Undercuts, `an axial pull frees the whole cup`)
		require.Equal(t, decad.Suspect, br.Status)
		require.False(t, report.Trustworthy())
	})

	// Tilt the pull and three faces hook against it: the outer wall facing the
	// tilt, the matching cavity wall, and the outer floor.
	t.Run("tilt hooks three faces", func(t *testing.T) {
		doc, box := shellBox(t)
		cup, err := box.Shell(topCap(box), units.Millimeters(th))
		require.NoError(t, err)
		report, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(1, 0, 1)))
		require.NoError(t, err)
		br := report.Bodies[0]
		require.Len(t, br.Undercuts, 3)
		for _, f := range br.Undercuts {
			require.Equal(t, decad.KindPlane, f.Surface().Kind())
		}
		// The kept outer floor (capStart) is one of them.
		capStartRef := decad.FeatureRef{Step: cup.Origin().Step, Role: "capStart"}
		found := false
		for _, f := range br.Undercuts {
			for _, o := range f.Origins() {
				if o == capStartRef {
					found = true
				}
			}
		}
		require.True(t, found, `the tilted-away outer floor is an undercut`)
		require.Equal(t, decad.Violating, br.Status)
	})
}

func TestShellCupMinRadius(t *testing.T) {
	// A cylindrical cup's cavity wall curves away from the material: its radius
	// R − th is the tightest concave one. The convex outer cylinder is not a
	// concave feature and does not appear.
	t.Run("cylindrical cavity reads its radius", func(t *testing.T) {
		const R, h, th = 20.0, 12.0, 4.0
		doc, _ := cylinderCup(t, R, h, th)
		report, err := doc.Verify(t.Context(), decad.WithMinRadius())
		require.NoError(t, err)
		br := report.Bodies[0]
		require.NotNil(t, br.MinRadius)
		require.Equal(t, decad.Exact, br.MinRadius.Exactness)
		require.True(t, br.MinRadius.Value.Equal(units.Millimeters(R-th), 1e-9),
			`the cavity cylinder is the cup's one concave face, got %s`, br.MinRadius.Value)
		require.Equal(t, decad.Suspect, br.Status, `bounded mass results need a nonzero tolerance`)
		require.False(t, report.Trustworthy())
	})

	// A box cup has no curved face — its concave wall/floor edges carry no
	// radius, so the proven answer is nil.
	t.Run("box cup has no concave radius", func(t *testing.T) {
		const th = 5.0
		doc, box := shellBox(t)
		_, err := box.Shell(topCap(box), units.Millimeters(th))
		require.NoError(t, err)
		report, err := doc.Verify(t.Context(), decad.WithMinRadius())
		require.NoError(t, err)
		br := report.Bodies[0]
		require.Nil(t, br.MinRadius, `a box cup has no concave curvature`)
		require.Equal(t, decad.Suspect, br.Status)
		require.False(t, report.Trustworthy())
	})
}

// diskHoledCup extrudes an annular section (outer circle R, concentric hole rh)
// to height h — a straight prism with one circular hole, whose outward cup
// wraps into a cylindrical post. (The rectangular post, whose rim is
// bridge-collinear, is covered by TestShellCupHoledRectangularPost.)
func diskHoledCup(t *testing.T, R, rh, h float64) (*decad.Document, *decad.Body) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	s.CreateCircle(s.CreatePoint(0, 0), R)
	s.CreateCircle(s.CreatePoint(0, 0), rh)
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
	body, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return doc, body
}

// TestShellCupHoledTessellate meshes a HOLED cup (modify §9, D4): a box with one
// central circular hole, top cap shelled inward. The result wraps a wall around
// the pocket AND a POST (a tube wall) around the hole — the tunnel through the
// body and the post around it must both be present, or the enclosed volume is
// wrong. The mesh is watertight, its bound within tolerance, and its enclosed
// volume equals the SAME A_O·h_o − A_C·h_c the build asserts, evaluated on the
// chorded circles the mesh actually holds.
func TestShellCupHoledTessellate(t *testing.T) {
	const th, rh = 5.0, 8.0
	h := shellBoxHeight
	_, box := circleHoledBox(t, [3]float64{50, 30, rh})
	cup, err := box.Shell(topCap(box), units.Millimeters(th))
	require.NoError(t, err)

	tol := 0.1
	mesh, err := cup.Tessellate(units.Millimeters(tol))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	require.Positive(t, mesh.Bound().Mag(), `the tunnel and post circles carry a real sagitta`)
	require.LessOrEqual(t, mesh.Bound().Mag(), tol)

	// The tunnel ring sits at radius rh, the post ring at rh + th, both about the
	// hole centre — a post-missing mesh would carry neither. Count each; every
	// sample owns a lo and a hi vertex.
	nTunnel, nPost := 0, 0
	for _, v := range mesh.Vertices() {
		switch rho := math.Hypot(v.X-50, v.Y-30); {
		case math.Abs(rho-rh) < 1e-6:
			nTunnel++
		case math.Abs(rho-(rh+th)) < 1e-6:
			nPost++
		}
	}
	require.Positive(t, nTunnel, `the tunnel through the body is meshed`)
	require.Positive(t, nPost, `the post around the tunnel is meshed`)
	nTunnel /= 2
	nPost /= 2

	// Volume = A_O·h − A_C·(h − t): the outer box less its chorded tunnel over
	// the full height, less the inner box less its chorded post over the pocket.
	aO := 100.0*60.0 - polyArea(rh, nTunnel)
	aC := (100-2*th)*(60-2*th) - polyArea(rh+th, nPost)
	wantVol := aO*h - aC*(h-th)
	require.InDelta(t, wantVol, meshVolume(mesh), 1e-6, `the mesh encloses exactly the chorded holed-cup volume, post included`)

	// Every one of the fourteen faces carries at least one facet, and every
	// facet's source is a live face.
	live := map[*decad.Face]struct{}{}
	for _, f := range cup.Faces() {
		live[f] = struct{}{}
	}
	covered := map[*decad.Face]struct{}{}
	for _, f := range mesh.SourceFaces() {
		_, ok := live[f]
		require.True(t, ok, `a facet's source is one of the cup's own faces`)
		covered[f] = struct{}{}
	}
	require.Len(t, covered, 14, `every face of the holed cup carries a facet`)

	// STL and OBJ export deterministically.
	var stl1, stl2 bytes.Buffer
	require.NoError(t, cup.STL(&stl1))
	require.NoError(t, cup.STL(&stl2))
	require.Equal(t, stl1.Bytes(), stl2.Bytes(), `STL export is deterministic`)
	require.Positive(t, stl1.Len())
	var obj1, obj2 bytes.Buffer
	require.NoError(t, cup.OBJ(&obj1))
	require.NoError(t, cup.OBJ(&obj2))
	require.Equal(t, obj1.Bytes(), obj2.Bytes(), `OBJ export is deterministic`)
}

// TestShellCupHoledTessellateOutward meshes an OUTWARD holed cup: an annular
// disk (outer R, hole rh) shelled outward on its top. Outward, the outer grows
// to R + th and the section hole shrinks to rh − th (the tunnel), while the
// original circles R and rh become the pocket wall and the post. All four rings
// are circular, so the rim bands close cleanly and the mesh is watertight with
// the post present.
func TestShellCupHoledTessellateOutward(t *testing.T) {
	const R, rh, th = 30.0, 8.0, 4.0
	h := shellBoxHeight
	_, disk := diskHoledCup(t, R, rh, h)
	cup, err := disk.Shell(topCap(disk), units.Millimeters(th), decad.WithShellSense(decad.Outward))
	require.NoError(t, err)

	tol := 0.2
	mesh, err := cup.Tessellate(units.Millimeters(tol))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	require.LessOrEqual(t, mesh.Bound().Mag(), tol)

	// Bucket the vertices by radius about the axis: the four rings at R + th
	// (outer), R (pocket wall), rh (post), rh − th (tunnel).
	counts := map[float64]int{}
	for _, v := range mesh.Vertices() {
		rho := math.Hypot(v.X, v.Y)
		for _, want := range []float64{R + th, R, rh, rh - th} {
			if math.Abs(rho-want) < 1e-6 {
				counts[want]++
			}
		}
	}
	for _, want := range []float64{R + th, R, rh, rh - th} {
		require.Positive(t, counts[want], `ring at radius %g is meshed`, want)
	}
	nOouter := counts[R+th] / 2
	nOtunnel := counts[rh-th] / 2
	nCouter := counts[R] / 2
	nCpost := counts[rh] / 2

	// Outward, the outer prism runs the full height plus the added floor below
	// (h + th) and the cavity runs h — the mirror of the inward heights.
	aO := polyArea(R+th, nOouter) - polyArea(rh-th, nOtunnel)
	aC := polyArea(R, nCouter) - polyArea(rh, nCpost)
	wantVol := aO*(h+th) - aC*h
	require.InDelta(t, wantVol, meshVolume(mesh), 1e-6, `an outward holed cup encloses its chorded volume, post included`)
}

// TestShellCupHoledTessellateOutwardBox meshes a circle-holed OUTWARD box cup:
// the outer region dilates to a rounded rectangle whose corner tangent points
// land on the cavity's own edge lines — the bridge-collinear rim — and the
// circular hole grows a post. It is a normal band, so it triangulates into a
// watertight mesh; the chorded features are the four outer corners plus the
// tunnel and post circles.
func TestShellCupHoledTessellateOutwardBox(t *testing.T) {
	const th, rh = 5.0, 8.0
	h := shellBoxHeight
	_, box := circleHoledBox(t, [3]float64{50, 30, rh})
	cup, err := box.Shell(topCap(box), units.Millimeters(th), decad.WithShellSense(decad.Outward))
	require.NoError(t, err)

	tol := 0.1
	mesh, err := cup.Tessellate(units.Millimeters(tol))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	require.LessOrEqual(t, mesh.Bound().Mag(), tol)

	vol, err := cup.Volume()
	require.NoError(t, err)
	exact, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	b := mesh.Bound().Mag()
	// Curved features chorded: the four outer corners (radius th) and tunnel
	// circle (radius rh − th) over the outer prism height h + th, and the post
	// circle (radius rh) over the cavity height h.
	slack := 2 * math.Pi * b * (th*(h+th) + (rh-th)*(h+th) + rh*h)
	require.InDelta(t, exact, meshVolume(mesh), slack, `the mesh encloses the holed cup volume up to its chorded arcs`)
}

// TestShellCupHoledUndercuts surveys a HOLED cup against a pull (modify §9, D2).
// The post walls (side(i,j)/shellSide(i,j), i ≥ 1) are now walked, so a post
// undercut is caught rather than dropped into a false all-clear.
func TestShellCupHoledUndercuts(t *testing.T) {
	const th, rh = 5.0, 8.0

	// Pulled straight out of the open top, every wall is perpendicular (the
	// cylinders' normals are radial) and the outer floor exactly antiparallel:
	// no undercut, Sound.
	t.Run("clear under an axial pull", func(t *testing.T) {
		doc, box := circleHoledBox(t, [3]float64{50, 30, rh})
		_, err := box.Shell(topCap(box), units.Millimeters(th))
		require.NoError(t, err)
		report, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(0, 0, 1)))
		require.NoError(t, err)
		br := report.Bodies[0]
		require.NotNil(t, br.Undercuts)
		require.Empty(t, br.Undercuts, `an axial pull frees the whole holed cup`)
		require.Equal(t, decad.Suspect, br.Status)
		require.False(t, report.Trustworthy())
	})

	// Tilt the pull and the post cylinder hooks against it — a full cylinder
	// sweeps every lateral normal, so part of the post opposes any non-axial
	// pull. The reading is Violating and names the post wall shellSide(1,0).
	t.Run("tilt hooks the post wall", func(t *testing.T) {
		doc, box := circleHoledBox(t, [3]float64{50, 30, rh})
		cup, err := box.Shell(topCap(box), units.Millimeters(th))
		require.NoError(t, err)
		report, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(1, 0, 1)))
		require.NoError(t, err)
		br := report.Bodies[0]
		require.Equal(t, decad.Violating, br.Status)

		postRef := decad.FeatureRef{Step: cup.Origin().Step, Role: "shellSide(1,0)"}
		var post *decad.Face
		for _, f := range br.Undercuts {
			for _, o := range f.Origins() {
				if o == postRef {
					post = f
				}
			}
		}
		require.NotNil(t, post, `the post wall is caught as an undercut`)
		require.Equal(t, decad.KindCylinder, post.Surface().Kind(), `the post is a cylinder`)
	})
}

// TestShellCupHoledMinRadius reads the tightest concave radius of a HOLED cup
// (modify §9, D3): the tunnel the post wraps is a hole through the body, radius
// rh, the one concave cylindrical face. The post's own outer cylinder curves
// TOWARD the material and is not concave, so the reading is rh, exact.
func TestShellCupHoledMinRadius(t *testing.T) {
	const th, rh = 5.0, 8.0
	doc, box := circleHoledBox(t, [3]float64{50, 30, rh})
	_, err := box.Shell(topCap(box), units.Millimeters(th))
	require.NoError(t, err)
	report, err := doc.Verify(t.Context(), decad.WithMinRadius())
	require.NoError(t, err)
	br := report.Bodies[0]
	require.NotNil(t, br.MinRadius, `the tunnel is a concave cylindrical face`)
	require.Equal(t, decad.Exact, br.MinRadius.Exactness)
	require.True(t, br.MinRadius.Value.Equal(units.Millimeters(rh), 1e-9),
		`the tunnel wall the post wraps is the tightest concave radius, got %s`, br.MinRadius.Value)
	require.Equal(t, decad.Suspect, br.Status, `bounded mass results need a nonzero tolerance`)
	require.False(t, report.Trustworthy())
}
