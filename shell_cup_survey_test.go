package decad_test

import (
	"bytes"
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

func TestShellCupTessellateDegenerateRimRefused(t *testing.T) {
	// An outward box cup dilates its rectangular section, landing the rounded
	// outer corners' tangent points on the cavity's own top and bottom edge
	// lines — a rim band whose loops are collinear on both sides, a degeneracy
	// ear clipping cannot resolve. The evaluator proves the mesh would be
	// cracked and refuses it, never returning a non-watertight mesh.
	const th = 5.0
	_, box := shellBox(t)
	cup, err := box.Shell(topCap(box), units.Millimeters(th), decad.WithShellSense(decad.Outward))
	require.NoError(t, err)

	_, err = cup.Tessellate(units.Millimeters(0.2))
	require.ErrorIs(t, err, decad.ErrDegenerate, `a rim the triangulator cannot close is refused, not returned cracked`)
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
		require.Equal(t, decad.Sound, br.Status)
		require.True(t, report.Trustworthy())
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
		require.Equal(t, decad.Sound, br.Status, `a min-radius reading is a measurement, never a verdict`)
		require.True(t, report.Trustworthy())
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
		require.Equal(t, decad.Sound, br.Status)
		require.True(t, report.Trustworthy())
	})
}

// TestShellCupHoledDownstreamStaged pins the honest staging of a HOLED cup's
// hole-sensitive downstream readers (modify §9, D2/D3/D4). The build itself is
// correct (a post per hole, one lump, Exact mass properties), but tessellation
// and the undercut/min-radius surveys still walk only loop 0, so on a holed cup
// they would answer hole-blind — a hole-free mesh, a false all-clear, a too-large
// radius. Each stages instead: tessellation refuses (ErrUnsupported) and the two
// surveys read undecided (Suspect). A HOLE-FREE cup is unaffected (the box and
// cylinder tests above still answer outright).
func TestShellCupHoledDownstreamStaged(t *testing.T) {
	const th, rh = 5.0, 8.0
	doc, box := circleHoledBox(t, [3]float64{50, 30, rh})
	cup, err := box.Shell(topCap(box), units.Millimeters(th))
	require.NoError(t, err)

	// Tessellate / STL / OBJ all honestly refuse a holed cup.
	_, err = cup.Tessellate(units.Millimeters(0.1))
	require.ErrorIs(t, err, decad.ErrUnsupported, `a holed cup's tessellation is staged`)
	var buf bytes.Buffer
	require.ErrorIs(t, cup.STL(&buf), decad.ErrUnsupported, `STL inherits the staged tessellation`)
	buf.Reset()
	require.ErrorIs(t, cup.OBJ(&buf), decad.ErrUnsupported, `OBJ inherits the staged tessellation`)

	// The undercut survey cannot see a post's walls, so it reads Suspect
	// rather than dropping a post undercut into a false Sound.
	report, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(0, 0, 1)))
	require.NoError(t, err)
	br := report.Bodies[0]
	require.Nil(t, br.Undercuts, `a staged undercut survey lists no faces`)
	require.Equal(t, decad.Suspect, br.Status)
	require.False(t, report.Trustworthy())

	// The min-radius survey misses a post's concave radius, so it too stages.
	report, err = doc.Verify(t.Context(), decad.WithMinRadius())
	require.NoError(t, err)
	br = report.Bodies[0]
	require.Nil(t, br.MinRadius, `a staged min-radius survey reads nothing`)
	require.Equal(t, decad.Suspect, br.Status)
	require.False(t, report.Trustworthy())
}
