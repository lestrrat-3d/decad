package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// twistedRibbon describes a ribbon: a rectangle whose one long side is a fit
// spline through a leaned cosine tooth edge, turned about the plane origin by
// station/lead radians per millimetre.
type twistedRibbon struct {
	sections     int
	step         float64 // Z spacing
	width, thick float64
	depth, pitch float64 // tooth height and pitch
	slant, bow   float64 // tan of the lean, bow coefficient
	lead         float64 // mm of height per radian of turn
	fitPoints    int
	originShift  float64 // moves the section off the plane origin
	skipLast     int     // fit points dropped from the last section
}

func (r twistedRibbon) defaults() twistedRibbon {
	r.step, r.width, r.thick = 0.26, 15.5, 3.8
	r.depth, r.pitch = 2.6, 2.62
	r.slant, r.bow = math.Tan(25.8*math.Pi/180), 0.05
	r.lead = 49.5 / (2 * math.Pi)
	r.fitPoints = 11
	return r
}

// toothU is the leaned cosine edge's position across the width.
func (r twistedRibbon) toothU(v, station float64) float64 {
	return r.width/2 - r.depth/2 + r.depth/2*math.Cos(2*math.Pi*(station+r.slant*v)/r.pitch) - r.bow*v*v
}

func (r twistedRibbon) turn(i int, x, y float64) (float64, float64) {
	angle := float64(i) * r.step / r.lead
	return x*math.Cos(angle) - y*math.Sin(angle) + r.originShift, x*math.Sin(angle) + y*math.Cos(angle)
}

// build draws the sections with the real sketch package and returns them with
// each section's fit points and boundary lines in plane coordinates.
func (r twistedRibbon) build(t *testing.T) ([]decad.LoftSection, [][][2]float64) {
	t.Helper()
	world := sketch.NewWorld()
	sections := make([]decad.LoftSection, r.sections)
	loops := make([][][2]float64, r.sections)
	for i := range sections {
		plane := world.XY()
		if i != 0 {
			var err error
			plane, err = world.CreateOffsetPlane(world.XY(), float64(i)*r.step)
			require.NoError(t, err)
		}
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		station := float64(i) * r.step
		point := func(x, y float64) *sketch.Point {
			x, y = r.turn(i, x, y)
			loops[i] = append(loops[i], [2]float64{x, y})
			p := s.CreatePoint(x, y)
			s.Fix(p)
			return p
		}
		count := r.fitPoints
		if i == r.sections-1 {
			count -= r.skipLast
		}
		b0 := point(-r.width/2, -r.thick/2)
		fit := make([]*sketch.Point, count)
		for j := range fit {
			v := -r.thick/2 + r.thick*float64(j)/float64(count-1)
			fit[j] = point(r.toothU(v, station), v)
		}
		b1 := point(-r.width/2, r.thick/2)
		s.CreateLine(b0, fit[0])
		_, err = s.CreateFitSpline(fit...)
		require.NoError(t, err)
		s.CreateLine(fit[count-1], b1)
		s.CreateLine(b1, b0)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		for _, profile := range s.Profiles() {
			if profile.Valid && len(profile.Outer) == 4 && len(profile.Holes) == 0 {
				sections[i] = decad.LoftSection{Sketch: s, Profile: profile}
			}
		}
		require.NotNil(t, sections[i].Profile, "section %d has no four-edge region", i)
	}
	return sections, loops
}

// twistedReferenceVolume integrates the section areas of the denoted solid
// without Decad: each boundary sample moves along the C1 Hermite curve through
// its counterparts in every section, and the shoelace area of the sampled loop
// is integrated over height by Gauss-Legendre quadrature.
func twistedReference(t *testing.T, sections []decad.LoftSection, step float64) (float64, float64) {
	t.Helper()
	const lineSamples, splineSamples = 24, 96
	samples := make([][][2]float64, len(sections))
	for i, section := range sections {
		var perEdge [][][2]float64
		fitAt := -1
		for e, edge := range section.Profile.Outer {
			var pts [][2]float64
			switch curve := edge.Entity.(type) {
			case *sketch.Line:
				x0, y0, x1, y1 := curve.Start.X(), curve.Start.Y(), curve.End.X(), curve.End.Y()
				for j := range lineSamples {
					f := float64(j) / lineSamples
					pts = append(pts, [2]float64{x0 + f*(x1-x0), y0 + f*(y1-y0)})
				}
			case *sketch.FitSpline:
				fitAt = e
				// Spans are the chord-length intervals between fit points.
				chord := make([]float64, len(curve.Fit))
				for j := 1; j < len(chord); j++ {
					chord[j] = chord[j-1] + math.Hypot(curve.Fit[j].X()-curve.Fit[j-1].X(),
						curve.Fit[j].Y()-curve.Fit[j-1].Y())
				}
				for j := range len(chord) - 1 {
					for k := range splineSamples {
						f := float64(k) / splineSamples
						x, y := curve.Eval((chord[j] + f*(chord[j+1]-chord[j])) / chord[len(chord)-1])
						pts = append(pts, [2]float64{x, y})
					}
				}
			default:
				t.Fatalf("unexpected entity %T", edge.Entity)
			}
			require.False(t, edge.Reversed, "the reference samples assume a forward walk")
			perEdge = append(perEdge, pts)
		}
		require.GreaterOrEqual(t, fitAt, 0)
		for e := range perEdge {
			samples[i] = append(samples[i], perEdge[(fitAt+e)%len(perEdge)]...)
		}
	}
	n := len(sections)
	count := len(samples[0])
	for i := range samples {
		require.Len(t, samples[i], count)
	}
	tangent := func(i, k, axis int) float64 {
		switch i {
		case 0:
			return samples[1][k][axis] - samples[0][k][axis]
		case n - 1:
			return samples[n-1][k][axis] - samples[n-2][k][axis]
		}
		return (samples[i+1][k][axis] - samples[i-1][k][axis]) / 2
	}
	// point is boundary sample k at height span i and parameter s.
	point := func(i, k int, s float64) [2]float64 {
		h00, h10 := 2*s*s*s-3*s*s+1, s*s*s-2*s*s+s
		h01, h11 := -2*s*s*s+3*s*s, s*s*s-s*s
		var p [2]float64
		for axis := range 2 {
			p[axis] = h00*samples[i][k][axis] + h10*tangent(i, k, axis) +
				h01*samples[i+1][k][axis] + h11*tangent(i+1, k, axis)
		}
		return p
	}
	shoelace := func(pts [][2]float64) float64 {
		twice := 0.0
		for k, p := range pts {
			q := pts[(k+1)%len(pts)]
			twice += p[0]*q[1] - p[1]*q[0]
		}
		return twice / 2
	}
	nodes := [5]float64{0, -0.5384693101056831, 0.5384693101056831, -0.9061798459386640, 0.9061798459386640}
	weights := [5]float64{0.5688888888888889, 0.4786286704993665, 0.4786286704993665, 0.2369268850561891, 0.2369268850561891}
	volume := 0.0
	for i := range n - 1 {
		for g := range nodes {
			ring := make([][2]float64, count)
			for k := range ring {
				ring[k] = point(i, k, (nodes[g]+1)/2)
			}
			volume += weights[g] / 2 * step * shoelace(ring)
		}
	}
	// The wall area is the sum of fine triangles over the same samples; the
	// caps are the first and last section's own polygons.
	const bands = 8
	area := 0.0
	for i := range n - 1 {
		for b := range bands {
			s0, s1 := float64(b)/bands, float64(b+1)/bands
			for k := range count {
				next := (k + 1) % count
				p00, p10 := point(i, k, s0), point(i, next, s0)
				p01, p11 := point(i, k, s1), point(i, next, s1)
				v := func(p [2]float64, s float64) [3]float64 { return [3]float64{p[0], p[1], (float64(i) + s) * step} }
				a, b2, c, d := v(p00, s0), v(p10, s0), v(p11, s1), v(p01, s1)
				area += triangleArea(a, b2, c) + triangleArea(a, c, d)
			}
		}
	}
	first, last := make([][2]float64, count), make([][2]float64, count)
	for k := range count {
		first[k], last[k] = samples[0][k], samples[n-1][k]
	}
	return volume, area + math.Abs(shoelace(first)) + math.Abs(shoelace(last))
}

func triangleArea(a, b, c [3]float64) float64 {
	u := [3]float64{b[0] - a[0], b[1] - a[1], b[2] - a[2]}
	v := [3]float64{c[0] - a[0], c[1] - a[1], c[2] - a[2]}
	x, y, z := u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]
	return math.Sqrt(x*x+y*y+z*z) / 2
}

// meshIsWatertight reports whether every undirected edge has exactly one
// triangle in each direction, and returns the signed volume.
func meshIsWatertight(mesh *decad.Mesh) (bool, float64) {
	type edge struct{ a, b int }
	directed := make(map[edge]int)
	vertices := mesh.Vertices()
	volume := 0.0
	for _, tri := range mesh.Triangles() {
		for k := range 3 {
			directed[edge{tri[k], tri[(k+1)%3]}]++
		}
		a, b, c := vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]
		volume += a.Dot(b.Cross(c)) / 6
	}
	for e, count := range directed {
		if count != 1 || directed[edge{e.b, e.a}] != 1 {
			return false, volume
		}
	}
	return true, volume
}

func TestLoftSectionsTwistedRibbonKeepsVolumeAndMeshProof(t *testing.T) {
	ribbon := twistedRibbon{sections: 41}.defaults()
	sections, loops := ribbon.build(t)
	doc := decad.New()
	body, err := decad.LoftSections(t.Context(), doc, sections...)
	require.NoError(t, err)
	require.Equal(t, decad.BodySolid, body.Kind())
	require.Len(t, doc.Bodies(), 1)
	require.Len(t, body.Faces(), 6)

	reference, referenceArea := twistedReference(t, sections, ribbon.step)
	volume, err := body.Volume()
	require.NoError(t, err)
	t.Logf("volume %.4f reference %.4f bound %.4f", volume.Value.Base(), reference, volume.Bound.Base())
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-reference), volume.Bound.Base())
	require.Less(t, volume.Bound.Base(), 0.02*reference)
	area, err := body.Area()
	require.NoError(t, err)
	t.Logf("area %.4f bound %.4f", area.Value.Base(), area.Bound.Base())
	t.Logf("area reference %.4f", referenceArea)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-referenceArea), area.Bound.Base())
	require.Less(t, area.Bound.Base(), area.Value.Base())

	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.LessOrEqual(t, mesh.Bound().Base(), 0.05)
	closed, signed := meshIsWatertight(mesh)
	require.True(t, closed, "the mesh must be a closed oriented surface")
	require.Greater(t, signed, 0.0)
	require.LessOrEqual(t, math.Abs(signed-reference), volume.Bound.Base())
	t.Logf("mesh: %d vertices, %d triangles, bound %.5f", len(mesh.Vertices()), len(mesh.Triangles()), mesh.Bound().Base())

	// Every recorded section point of the first and last section is held.
	for _, which := range []int{0, ribbon.sections - 1} {
		z := float64(which) * ribbon.step
		for _, p := range loops[which] {
			found := false
			for _, v := range mesh.Vertices() {
				found = found || v.X == p[0] && v.Y == p[1] && v.Z == z
			}
			require.Truef(t, found, "section %d point %v is not held", which, p)
		}
	}

	// A rigid motion keeps the body usable and its readings.
	motion, err := r3.RotationAround(r3.NewVec(1, 2, 3), r3.NewVec(1, 1, 0), units.Radians(0.7))
	require.NoError(t, err)
	moved, err := decad.Copy(t.Context(), body, decad.WithCopyPlacement(motion))
	require.NoError(t, err)
	movedVolume, err := moved.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(movedVolume.Value.Base()-volume.Value.Base()),
		movedVolume.Bound.Base()+volume.Bound.Base())
	movedMesh, err := moved.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, movedMesh.BoundaryVerified())
	require.True(t, movedMesh.VolumeVerified())
}

func TestLoftSectionsTwistedThreeSectionsFallBackToTwistedRoute(t *testing.T) {
	ribbon := twistedRibbon{sections: 3}.defaults()
	ribbon.step = 2
	sections, _ := ribbon.build(t)
	body, err := decad.LoftSections(t.Context(), decad.New(), sections...)
	require.NoError(t, err)
	reference, referenceArea := twistedReference(t, sections, ribbon.step)
	volume, err := body.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-reference), volume.Bound.Base())
	area, err := body.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-referenceArea), area.Bound.Base())
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())

	move, err := r3.Translation(r3.NewVec(20, 0, 0))
	require.NoError(t, err)
	placed, err := decad.Copy(t.Context(), body, decad.WithCopyPlacement(move))
	require.NoError(t, err)
	placedMesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, placedMesh.BoundaryVerified())
	require.True(t, placedMesh.VolumeVerified())
}

func TestLoftSectionsTwistedRefusesUnprovedInputs(t *testing.T) {
	for _, tc := range []struct {
		name, reason string
		ribbon       twistedRibbon
	}{
		{"fit point count", "fit-point counts", twistedRibbon{sections: 5, skipLast: 1}},
		{"origin outside", "polar angle", twistedRibbon{sections: 5, originShift: -9}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			ribbon := tc.ribbon.defaults()
			ribbon.sections, ribbon.originShift, ribbon.skipLast = tc.ribbon.sections, tc.ribbon.originShift, tc.ribbon.skipLast
			sections, _ := ribbon.build(t)
			doc := decad.New()
			body, err := decad.LoftSections(t.Context(), doc, sections...)
			require.Nil(t, body)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.ErrorContains(t, err, tc.reason)
			require.Empty(t, doc.Bodies())
		})
	}
}

func TestLoftSectionsTwistedRefusesExcessAndUnevenSections(t *testing.T) {
	ribbon := twistedRibbon{sections: 5}.defaults()
	sections, _ := ribbon.build(t)
	tooMany := make([]decad.LoftSection, 1025)
	for i := range tooMany {
		tooMany[i] = sections[i%len(sections)]
	}
	doc := decad.New()
	body, err := decad.LoftSections(t.Context(), doc, tooMany...)
	require.Nil(t, body)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "at most 1024 sections")
	require.Empty(t, doc.Bodies())

	// Reordering two sections breaks the ascending equal spacing.
	uneven := append([]decad.LoftSection(nil), sections...)
	uneven[1], uneven[2] = uneven[2], uneven[1]
	body, err = decad.LoftSections(t.Context(), doc, uneven...)
	require.Nil(t, body)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "strictly ascending")
	require.Empty(t, doc.Bodies())
}
