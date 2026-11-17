package retainedblend

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// planeTag is a plane group through o with unit normal n (perpendicular to the frame's two axes).
func planeTag(t *testing.T, o, n r3.Vec, delta float64) facetedrecord.Group {
	t.Helper()
	u, ok := perpendicular(n)
	require.True(t, ok)
	f, err := r3.NewFrame(o, u, n.Cross(u))
	require.NoError(t, err)
	return facetedrecord.Group{Surface: surfacegeom.Plane{Frame: f}, HasAxialDelta: delta != 0, AxialDelta: delta}
}

// slabVolume is the volume between the triangle abc and the plane through o with normal n, summed by midpoint
// quadrature over a fine barycentric subdivision of the triangle: the facet's height over the plane is linear, so
// the quadrature of |height| is exact but for the crossing, and the grid error is far below the bound's margin. It
// shares no code with planeFacetVolume.
func slabVolume(a, b, c, o, n r3.Vec) float64 {
	const grid = 400
	area := b.Sub(a).Cross(c.Sub(a)).Len() / 2
	sum, count := 0.0, 0
	for i := range grid {
		for j := range grid - i {
			for _, cell := range [][2]float64{{float64(i) + 1.0/3, float64(j) + 1.0/3}, {float64(i) + 2.0/3, float64(j) + 2.0/3}} {
				if k := cell[0] + cell[1]; k >= grid {
					continue
				}
				p := a.Add(b.Sub(a).Scale(cell[0] / grid)).Add(c.Sub(a).Scale(cell[1] / grid))
				sum += math.Abs(n.Dot(p.Sub(o)))
				count++
			}
		}
	}
	return area * sum / float64(count)
}

// TestPlaneFacetVolume checks the slab term of a re-triangulated facet against hand-computed values: a right
// triangle of legs 4 and 3 whose far corner stands 0.5 above the tag plane z = 0 (area ½·|(0, −2, 12)| = √37,
// height 0.5), and a facet over the slanted plane x + y = 2.
func TestPlaneFacetVolume(t *testing.T) {
	t.Parallel()
	t.Run("a tilted facet over a coordinate plane", func(t *testing.T) {
		t.Parallel()
		verts := []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(4, 0, 0), r3.NewVec(0, 3, 0.5)}
		tris := [][3]int{{0, 1, 2}}
		pl := plane{n: r3.NewVec(0, 0, 1), o: r3.NewVec(0, 0, 0)}
		area := math.Sqrt(37)
		want := 0.5 * area
		got := planeFacetVolume(verts, tris, pl, 0)
		require.GreaterOrEqual(t, got, want*(1-1e-15))
		require.LessOrEqual(t, got, want*(1+1e-12))
		// The displacement adds its own slab: (0.5 + 0.01) · area.
		want = 0.51 * area
		got = planeFacetVolume(verts, tris, pl, 0.01)
		require.GreaterOrEqual(t, got, want*(1-1e-15))
		require.LessOrEqual(t, got, want*(1+1e-12))
		// It is at least the slab the facet really bounds against the plane moved the other way.
		for _, off := range []float64{-0.01, 0.01} {
			truth := slabVolume(verts[0], verts[1], verts[2], r3.NewVec(0, 0, off), pl.n)
			require.GreaterOrEqual(t, got, truth)
		}
	})
	t.Run("a facet over a slanted plane", func(t *testing.T) {
		t.Parallel()
		n, ok := r3.NewVec(1, 1, 0).Normalize()
		require.True(t, ok)
		verts := []r3.Vec{r3.NewVec(1, 1, 0), r3.NewVec(2, 1, 0), r3.NewVec(1, 2, 0)}
		pl := plane{n: n, o: r3.NewVec(1, 1, 0)}
		// Corners at 0, 1/√2 and 1/√2 from the plane; area 1/2.
		want := math.Sqrt2 / 2 * 0.5
		got := planeFacetVolume(verts, [][3]int{{0, 1, 2}}, pl, 0)
		require.GreaterOrEqual(t, got, want*(1-1e-15))
		require.LessOrEqual(t, got, want*(1+1e-12))
		require.GreaterOrEqual(t, got, slabVolume(verts[0], verts[1], verts[2], pl.o, pl.n))
	})
	t.Run("a facet on the plane adds only the displacement", func(t *testing.T) {
		t.Parallel()
		verts := []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(4, 0, 0), r3.NewVec(0, 3, 0)}
		got := planeFacetVolume(verts, [][3]int{{0, 1, 2}}, plane{n: r3.NewVec(0, 0, 1)}, 0.25)
		require.GreaterOrEqual(t, got, 1.5)
		require.LessOrEqual(t, got, 1.5*(1+1e-12))
		require.Zero(t, planeFacetVolume(verts, [][3]int{{0, 1, 2}}, plane{n: r3.NewVec(0, 0, 1)}, 0))
	})
}

// TestMeasureChangeChargesRetriangulatedPlaneFacets re-triangulates a plane face whose facet stands 0.5 above its
// tag plane, over a tag that states a displacement, and reads the volume bound measureChange returns. The strip is
// empty, so every other term is zero, and the bound must hold the slab between the new facet and the true plane
// on either side of the tag (Claim K).
func TestMeasureChangeChargesRetriangulatedPlaneFacets(t *testing.T) {
	t.Parallel()
	const delta = 0.01
	verts := []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(4, 0, 0), r3.NewVec(0, 3, 0.5)}
	n := r3.NewVec(0, 0, 1)
	fp := facetedrecord.Payload{Groups: []facetedrecord.Group{planeTag(t, r3.Vec{}, n, delta)}}
	w := newWork(fp, rim{scale: 1}, blendSpec{dA: 1, dB: 1})
	for _, v := range verts {
		w.verts = append(w.verts, v)
		w.beta = append(w.beta, 0)
	}
	require.NoError(t, w.retriangulate(n, [][]int{{0, 1, 2}}, 0))
	require.Len(t, w.out, 1)
	_, vol := w.measureChange(blendTag{}, len(w.out))
	for _, side := range []float64{-delta, delta} {
		truth := slabVolume(verts[0], verts[1], verts[2], r3.NewVec(0, 0, side), n)
		require.GreaterOrEqual(t, vol, truth, "the true plane at %g", side)
	}
	require.LessOrEqual(t, vol, (0.5+delta)*math.Sqrt(37)*(1+1e-12), "the term is the facet's height plus the displacement over its area")
}

// TestRetriangulateRefusesFaceWithoutPlaneTag shows that a face whose group states no plane gets no volume term
// and so refuses: nothing bounds its facets' distance from a plane.
func TestRetriangulateRefusesFaceWithoutPlaneTag(t *testing.T) {
	t.Parallel()
	w := newWork(facetedrecord.Payload{Groups: []facetedrecord.Group{{}}}, rim{scale: 1}, blendSpec{dA: 1, dB: 1})
	w.verts = []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(4, 0, 0), r3.NewVec(0, 3, 0)}
	w.beta = make([]float64, 3)
	err := w.retriangulate(r3.NewVec(0, 0, 1), [][]int{{0, 1, 2}}, 0)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "states no plane tag")
}

// arcAreaMove blends the open arc of a rim of radius 10 about the z axis, whose plane lies inside it (kappa −1),
// between the end planes x = h0 and y = h1 with a fillet of radius 0.5, and returns the exact area change the
// route computes for those tags (the blend face less what the faces lose) and the term a displacement of each end
// plane would add: the route's own pieces and charge.
func arcAreaMove(t *testing.T, h0, h1, delta float64) (change, term float64) {
	t.Helper()
	const radius = 10.0
	a := arcRim{sign: 1}
	a.n, a.u, a.w = r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)
	a.radius, a.kappa, a.side, a.convex, a.scale = radius, -1, 1, true, 20
	a.ends[0].g = plane{n: r3.NewVec(1, 0, 0), o: r3.NewVec(h0, 0, 0), delta: delta}
	a.ends[1].g = plane{n: r3.NewVec(0, 1, 0), o: r3.NewVec(0, h1, 0), delta: delta}
	a.ends[0].exact = r3.NewVec(h0, math.Sqrt(radius*radius-h0*h0), 0)
	a.ends[1].exact = r3.NewVec(math.Sqrt(radius*radius-h1*h1), h1, 0)
	a.theta0 = math.Atan2(a.ends[0].exact.Y, a.ends[0].exact.X)
	a.span = math.Atan2(a.ends[1].exact.Y, a.ends[1].exact.X) - a.theta0
	spec := blendSpec{fillet: true, r: 0.5}
	profile, _ := a.profile(spec)
	var ends []endSection
	for i := range 2 {
		sec, err := a.section(spec, a.ends[i].g)
		require.NoError(t, err)
		ends = append(ends, sec)
	}
	ss, err := arcRows(spec, 2e-3, 2e-3, ends)
	require.NoError(t, err)
	ref := [2]float64{a.theta0, a.theta0 + a.span}
	var rows []arcRow
	for _, s := range ss {
		th, err := a.rowAngles(profile, s, ref)
		require.NoError(t, err)
		rho, z := profile(s)
		rows = append(rows, arcRow{rho: rho, z: z, theta: th})
		ref = th
	}
	removed, blend := a.arcAreas(profile, rows, spec.r)
	c := addCharge(planeCharge(a.ends[0].g), a.ends[1].g)
	w := &work{r: rim{charge: c}}
	w.arcPlanePieces(a, rows, ends, spec.r, blend, spec)
	return blend - removed, planeAreaCharge(w.planePieces, c)
}

// TestArcAreaTermCoversDisplacedEndPlanes moves both end planes of an arc whose rows meet them at a sine of 0.1
// to 0.33 by delta along their normals and compares the exact area change the route computes there with the one on
// the tags. That difference is what the displacement does to the exact pieces, and the area term must cover it.
func TestArcAreaTermCoversDisplacedEndPlanes(t *testing.T) {
	t.Parallel()
	const h, delta = 9.45, 1e-3
	tagChange, term := arcAreaMove(t, h, h, delta)
	for _, sign := range []float64{-1, 1} {
		moved, _ := arcAreaMove(t, h+sign*delta, h+sign*delta, delta)
		move := math.Abs(moved - tagChange)
		// The move is real: far above the rounding of the area sums.
		require.Greater(t, move, 1e-6, "the displacement %+g moves the exact area change", sign*delta)
		require.GreaterOrEqual(t, term, move, "the term covers the move of the exact pieces")
		require.Less(t, term, 1e9*move, "the term is finite and not absurdly loose")
	}
}

// TestSphereRimAreaTermCoversDisplacedPlane moves the plane of a plane–sphere rim by delta along its normal and
// compares the route's exact area change there with the one on the tag, as TestArcAreaTermCoversDisplacedEndPlanes
// does for an arc.
func TestSphereRimAreaTermCoversDisplacedPlane(t *testing.T) {
	t.Parallel()
	const radius, plane0, delta, r = 2.0, -1.0, 1e-4, 0.4
	spec := blendSpec{fillet: true, r: r}
	measure := func(t *testing.T, z, charge float64) (change, term float64) {
		t.Helper()
		s := sphereRim{sphereRadius: radius}
		s.center, s.n, s.radius = r3.NewVec(0, 0, z), r3.NewVec(0, 0, 1), math.Sqrt(radius*radius-z*z)
		s.u, _ = perpendicular(s.n)
		s.w = s.n.Cross(s.u)
		s.kappa, s.side, s.convex, s.scale = -1, -1, true, radius
		m, err := s.section(spec, 2e-3)
		require.NoError(t, err)
		annulus := math.Pi * math.Abs(m.rhoA*m.rhoA-s.radius*s.radius)
		band := 2 * math.Pi * radius * math.Abs(s.side*m.zB)
		w := &work{r: rim{charge: charge}}
		w.sphereRimPieces(s, spec, m, s.side*m.zB)
		return m.trueArea - annulus - band, planeAreaCharge(w.planePieces, charge)
	}
	charge := planeCharge(plane{delta: delta})
	tagChange, term := measure(t, plane0, charge)
	for _, sign := range []float64{-1, 1} {
		moved, _ := measure(t, plane0+sign*delta, charge)
		move := math.Abs(moved - tagChange)
		require.Greater(t, move, 1e-7, "the displacement %+g moves the exact area change", sign*delta)
		require.GreaterOrEqual(t, term, move)
		require.Less(t, term, 1e9*move)
	}
}

// TestArcRefusesDisplacedEndPlaneBelowMinSine meets the rows of a κ = −1 arc and an end plane at a sine of 1e-4
// at the rim's inner row. With the plane's displacement stated the arc refuses (a row at sine q moves its end by
// delta/q, which the area and volume terms do not charge); the same arc on the plane stating none builds.
func TestArcRefusesDisplacedEndPlaneBelowMinSine(t *testing.T) {
	t.Parallel()
	const radius, rhoMin = 10.0, 9.0
	a := arcRim{sign: 1}
	a.n, a.u, a.w = r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)
	a.radius, a.kappa, a.side = radius, -1, 1
	spec := blendSpec{fillet: true, r: 1}
	// The inner row, radius 9, meets x = h at the sine sqrt(1 − (h/9)²) = 1e-4.
	h := rhoMin * math.Sqrt(1-1e-8)
	at := func(h, delta float64) plane { return plane{n: r3.NewVec(1, 0, 0), o: r3.NewVec(h, 0, 0), delta: delta} }
	_, err := a.section(spec, at(h, 0))
	require.NoError(t, err, "the undisplaced twin builds")
	_, err = a.section(spec, at(h, 1e-9))
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "an arc's rows meet a displaced end plane below minSine")
	// At a sine of minSine and above the displaced plane is read.
	_, err = a.section(spec, at(rhoMin*math.Sqrt(1-4e-6), 1e-9))
	require.NoError(t, err)
}
