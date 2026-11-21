package apitest_test

import (
	"context"
	"fmt"
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/geom"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func circularFitSections(t *testing.T, offsets [2]float64, firstQuarter, circleLevels int) []decad.LoftSection {
	return circularFitSectionsAtCenter(t, offsets, firstQuarter, circleLevels, [2]float64{})
}

func circularFitSectionsAtCenter(t *testing.T, offsets [2]float64, firstQuarter, circleLevels int,
	circleCenter [2]float64) []decad.LoftSection {
	t.Helper()
	w := sketch.NewWorld()
	sections := make([]decad.LoftSection, circleLevels+2)
	for i := range sections {
		plane := w.XY()
		if i > 0 {
			var err error
			plane, err = w.CreateOffsetPlane(w.XY(), float64(i))
			require.NoError(t, err)
		}
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		point := func(x, y float64) *sketch.Point {
			p := s.CreatePoint(x, y)
			s.Fix(p)
			return p
		}
		coords := [][2]float64{{1, 0}, {0, 1}, {-1, 0}, {0, -1}}
		if i < circleLevels {
			for j := range coords {
				coords[j][0] += circleCenter[0]
				coords[j][1] += circleCenter[1]
			}
		}
		vertices := make([]*sketch.Point, len(coords))
		for j, p := range coords {
			vertices[j] = point(p[0], p[1])
		}
		if i < circleLevels {
			origin := point(circleCenter[0], circleCenter[1])
			for j := range vertices {
				index := (j + firstQuarter) % len(vertices)
				s.CreateArc(origin, vertices[index], vertices[(index+1)%len(vertices)])
			}
		} else {
			offset := offsets[i-circleLevels]
			if offsets == [2]float64{} {
				_, err = s.CreateFitSpline(vertices[0], point(0.5, 0.5), vertices[1])
			} else {
				_, err = s.CreateSpline(vertices[0], point(0.75+offset, 0.25+offset),
					point(0.5+offset, 0.5+offset), point(0.25+offset, 0.75+offset), vertices[1])
			}
			require.NoError(t, err)
			for j := 1; j < len(vertices); j++ {
				s.CreateLine(vertices[j], vertices[(j+1)%len(vertices)])
			}
		}
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		for _, profile := range s.Profiles() {
			if profile.Valid && len(profile.Holes) == 0 && len(profile.Outer) == 4 {
				sections[i] = decad.LoftSection{Sketch: s, Profile: profile}
			}
		}
		require.NotNil(t, sections[i].Profile)
	}
	return sections
}

func TestLoftSectionsCircularFitOffCenter(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name   string
		center [2]float64
		want   bool
	}{
		{"origin strictly inside", [2]float64{0.25, 0.125}, true},
		{"origin on circle", [2]float64{1, 0}, false},
		{"outside", [2]float64{1.25, 0}, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			sections := circularFitSectionsAtCenter(t, [2]float64{}, 0, 1, tc.center)
			doc := decad.New()
			body, err := decad.LoftSections(t.Context(), doc, sections...)
			if !tc.want {
				require.ErrorIs(t, err, decad.ErrUnsupported)
				require.Empty(t, doc.Bodies())
				return
			}
			require.NoError(t, err)
			mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
			for _, p := range mesh.Vertices() {
				if p.Z == 0 {
					x, y := p.X-tc.center[0], p.Y-tc.center[1]
					require.InDelta(t, 1, x*x+y*y, 1e-12)
				}
			}
		})
	}
}

func TestLoftSectionsCircularFitFlatQuarter(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	sections := make([]decad.LoftSection, 3)
	for i := range sections {
		plane := w.XY()
		if i > 0 {
			var err error
			plane, err = w.CreateOffsetPlane(w.XY(), float64(i))
			require.NoError(t, err)
		}
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		point := func(x, y float64) *sketch.Point {
			p := s.CreatePoint(x, y)
			s.Fix(p)
			return p
		}
		if i == 0 {
			center := point(0, 0)
			p := []*sketch.Point{point(1, 0), point(0, 1), point(-1, 0), point(0, -1)}
			for j := range p {
				index := (j + 1) % len(p)
				s.CreateArc(center, p[index], p[(index+1)%len(p)])
			}
		} else {
			low, high := point(1, -1), point(1, 1)
			_, err = s.CreateSpline(high, point(-0.5, 1), point(-1, 0), point(-0.5, -1), low)
			require.NoError(t, err)
			lowerJoin, upperJoin := point(1, -0.5), point(1, 0.5)
			s.CreateLine(low, lowerJoin)
			s.CreateLine(lowerJoin, upperJoin)
			s.CreateLine(upperJoin, high)
		}
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		for _, profile := range s.Profiles() {
			if profile.Valid && len(profile.Holes) == 0 && len(profile.Outer) == 4 {
				sections[i] = decad.LoftSection{Sketch: s, Profile: profile}
			}
		}
		require.NotNil(t, sections[i].Profile)
	}
	body, err := decad.LoftSections(t.Context(), decad.New(), sections...)
	require.NoError(t, err)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.LessOrEqual(t, mesh.Bound().Base(), 0.05)
	vertices, triangles := mesh.Vertices(), mesh.Triangles()
	for _, z := range []float64{1, 2} {
		for _, y := range []float64{-0.5, 0.5} {
			found := false
			for _, p := range vertices {
				found = found || p == (r3.Vec{X: 1, Y: y, Z: z})
			}
			require.True(t, found, "flat join (%g,%g) was not retained", y, z)
		}
	}
	// The source seam starts at +Y, so its fourth side owns the +X/+Y arc
	// that corresponds to the flat, irrespective of target edge order.
	flatRef := featureRefWithRole(t, body, "side(0,3)")
	flatFace, err := decad.Faces(decad.FaceCreatedBy(flatRef)).Exactly(1).SelectFaces(body)
	require.NoError(t, err)
	sources := mesh.SourceFaces()
	found := false
	var meshArea float64
	// At the lower flat join, the source quarter parameter is 1/4. At the
	// unrecorded height t=1/2, smoothstep is 1/2, so this is the exact wall point.
	wantMid := r3.Vec{X: 16.0 / 17, Y: -1.0 / 68, Z: 0.5}
	nearest := math.Inf(1)
	for i, tri := range triangles {
		a, b, c := vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]
		meshArea += b.Sub(a).Cross(c.Sub(a)).Len() / 2
		if sources[i] == flatFace[0] {
			nearest = math.Min(nearest, pointTriangleDistance(wantMid, a, b, c))
		}
		if sources[i] == flatFace[0] && a.Z == 0 && b.Z == 0 && a.X >= 0 && b.X >= 0 && a.Y >= 0 && b.Y >= 0 {
			found = true
		}
	}
	require.True(t, found, "the rotated source arc must retain its side role")
	require.LessOrEqual(t, nearest, mesh.Bound().Base()+1e-12, "unrecorded-height wall point")
	area, err := body.Area()
	require.NoError(t, err)
	require.Positive(t, area.Bound.Base())
	require.InDelta(t, meshArea, area.Value.Base(), area.Bound.Base())
	volume, err := body.Volume()
	require.NoError(t, err)
	require.Positive(t, volume.Bound.Base())
	closed, signed := meshIsWatertight(mesh)
	require.True(t, closed)
	require.Positive(t, signed)
	require.InDelta(t, signed, volume.Value.Base(), volume.Bound.Base())

	var correspondence decad.CircularLoftCorrespondence
	for i := range correspondence.Numerators {
		correspondence.Numerators[i] = []int64{0, 1, 3}
		correspondence.Denominators[i] = 3
	}
	explicit, err := decad.LoftSectionsWithCircularCorrespondence(t.Context(), decad.New(), correspondence, sections...)
	require.NoError(t, err)
	explicitMesh, err := explicit.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, explicitMesh.BoundaryVerified())
	require.True(t, explicitMesh.VolumeVerified())
	explicitRef := featureRefWithRole(t, explicit, "side(0,3)")
	explicitFace, err := decad.Faces(decad.FaceCreatedBy(explicitRef)).Exactly(1).SelectFaces(explicit)
	require.NoError(t, err)
	// The map sends 1/4 to 3/16. The target flat join is unchanged.
	wantExplicit := r3.Vec{X: 256.0 / 265, Y: -73.0 / 1060, Z: 0.5}
	explicitVertices, explicitSources := explicitMesh.Vertices(), explicitMesh.SourceFaces()
	explicitNearest := math.Inf(1)
	for i, tri := range explicitMesh.Triangles() {
		if explicitSources[i] == explicitFace[0] {
			explicitNearest = math.Min(explicitNearest, pointTriangleDistance(wantExplicit,
				explicitVertices[tri[0]], explicitVertices[tri[1]], explicitVertices[tri[2]]))
		}
	}
	require.LessOrEqual(t, explicitNearest, explicitMesh.Bound().Base()+1e-12, "explicit correspondence wall point")
}

func TestCircularLoftCorrespondenceRejectsInvalidMaps(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name        string
		controls    []int64
		denominator int64
	}{
		{"zero denominator", []int64{0, 1}, 0},
		{"wrong start", []int64{1, 3}, 3},
		{"wrong end", []int64{0, 2}, 3},
		{"repeated", []int64{0, 1, 1, 3}, 3},
		{"descending", []int64{0, 2, 1, 3}, 3},
		{"too many", []int64{0, 1, 2, 3, 4, 5, 6}, 6},
	} {
		t.Run(tc.name, func(t *testing.T) {
			var correspondence decad.CircularLoftCorrespondence
			for i := range correspondence.Numerators {
				correspondence.Numerators[i], correspondence.Denominators[i] = []int64{0, 1}, 1
			}
			correspondence.Numerators[0], correspondence.Denominators[0] = tc.controls, tc.denominator
			doc := decad.New()
			_, err := decad.LoftSectionsWithCircularCorrespondence(t.Context(), doc, correspondence, make([]decad.LoftSection, 3)...)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Empty(t, doc.Bodies())
		})
	}
}

func TestCircularLoftCorrespondenceSmoothCircularBands(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	sections := make([]decad.LoftSection, 4)
	for i := range sections {
		plane := w.XY()
		if i > 0 {
			var err error
			plane, err = w.CreateOffsetPlane(w.XY(), float64(i))
			require.NoError(t, err)
		}
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		point := func(x, y float64) *sketch.Point {
			p := s.CreatePoint(x, y)
			s.Fix(p)
			return p
		}
		if i < 3 {
			radius := [...]float64{0.25, 0.3125, 0.25}[i]
			p := []*sketch.Point{point(radius, 0), point(0, radius), point(-radius, 0), point(0, -radius)}
			for j := range p {
				s.CreateArc(s.Origin(), p[j], p[(j+1)%len(p)])
			}
		} else {
			coords := [][2]float64{{0.75, 0.75}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {0.75, -0.75}}
			controls := make([]*sketch.Point, len(coords))
			for j, p := range coords {
				controls[j] = point(p[0], p[1])
			}
			_, err = s.CreateSpline(controls...)
			require.NoError(t, err)
			lower, upper := point(0.75, -0.25), point(0.75, 0.25)
			s.CreateLine(controls[len(controls)-1], lower)
			s.CreateLine(lower, upper)
			s.CreateLine(upper, controls[0])
		}
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		for _, profile := range s.Profiles() {
			if profile.Valid && len(profile.Holes) == 0 && len(profile.Outer) == 4 {
				sections[i] = decad.LoftSection{Sketch: s, Profile: profile}
			}
		}
		require.NotNil(t, sections[i].Profile)
	}
	var correspondence decad.CircularLoftCorrespondence
	for i := range correspondence.Numerators {
		correspondence.Numerators[i], correspondence.Denominators[i] = []int64{0, 1, 3}, 3
	}
	linear, err := decad.LoftSectionsWithCircularCorrespondence(t.Context(), decad.New(), correspondence, sections...)
	require.NoError(t, err)
	linearMesh, err := linear.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	correspondence.SmoothCircularBands = true
	smooth, err := decad.LoftSectionsWithCircularCorrespondence(t.Context(), decad.New(), correspondence, sections...)
	require.NoError(t, err)
	smoothMesh, err := smooth.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, smoothMesh.BoundaryVerified())
	require.True(t, smoothMesh.VolumeVerified())
	// The linear mesh shares the +X seam's endpoints as a single mesh edge.
	// Therefore its held seam interpolates the radii linearly between them.
	indices := [2]int{-1, -1}
	for i, p := range linearMesh.Vertices() {
		if p == (r3.Vec{X: 0.3125, Z: 1}) {
			indices[0] = i
		}
		if p == (r3.Vec{X: 0.25, Z: 2}) {
			indices[1] = i
		}
	}
	require.NotEqual(t, -1, indices[0])
	require.NotEqual(t, -1, indices[1])
	sharedEdge := false
	for _, tri := range linearMesh.Triangles() {
		a, b := false, false
		for _, vertex := range tri {
			a = a || vertex == indices[0]
			b = b || vertex == indices[1]
		}
		sharedEdge = sharedEdge || a && b
	}
	require.True(t, sharedEdge)
	var samples int
	var maxDifference float64
	for _, p := range smoothMesh.Vertices() {
		if p.Z <= 1 || p.Z >= 2 {
			continue
		}
		tau := p.Z - 1
		radius := math.Hypot(p.X, p.Y)
		want := 0.3125 - 0.0625*tau*tau*(3-2*tau)
		require.InDelta(t, want, radius, 1e-12)
		maxDifference = math.Max(maxDifference, math.Abs(radius-(0.3125-0.0625*tau)))
		samples++
	}
	require.Positive(t, samples)
	require.Greater(t, maxDifference, 1e-4)
}

func TestLoftSectionsCircularFitFlatQuarterRecordedKnots(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	sections := make([]decad.LoftSection, 3)
	for i := range sections {
		plane := w.XY()
		if i > 0 {
			var err error
			plane, err = w.CreateOffsetPlane(w.XY(), float64(i))
			require.NoError(t, err)
		}
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		point := func(x, y float64) *sketch.Point {
			p := s.CreatePoint(x, y)
			s.Fix(p)
			return p
		}
		if i == 0 {
			center := point(0, 0)
			p := []*sketch.Point{point(1, 0), point(0, 1), point(-1, 0), point(0, -1)}
			for j := range p {
				s.CreateArc(center, p[j], p[(j+1)%len(p)])
			}
		} else {
			coords := [][2]float64{{1, 1}, {0.25, 1}, {-0.5, 1}, {-1, 0.5}, {-1, 0},
				{-1, -0.5}, {-0.5, -1}, {0.25, -1}, {1, -1}}
			controls := make([]*sketch.Point, len(coords))
			for j, p := range coords {
				controls[j] = point(p[0], p[1])
			}
			low, high := controls[len(controls)-1], controls[0]
			slices.Reverse(controls)
			_, err = s.CreateSpline(controls...)
			require.NoError(t, err)
			a, b := point(1, -0.5), point(1, 0.5)
			s.CreateLine(low, a)
			s.CreateLine(a, b)
			s.CreateLine(b, high)
		}
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		for _, profile := range s.Profiles() {
			if profile.Valid && len(profile.Holes) == 0 && len(profile.Outer) == 4 {
				sections[i] = decad.LoftSection{Sketch: s, Profile: profile}
			}
		}
		require.NotNil(t, sections[i].Profile)
	}
	body, err := decad.LoftSections(t.Context(), decad.New(), sections...)
	require.NoError(t, err)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	closed, volume := meshIsWatertight(mesh)
	require.True(t, closed)
	require.Positive(t, volume)
}

func TestLoftSectionsCircularFitRefusesDifferentRecordDirections(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	sections := make([]decad.LoftSection, 3)
	for i := range sections {
		plane := w.XY()
		if i > 0 {
			var err error
			plane, err = w.CreateOffsetPlane(w.XY(), float64(i))
			require.NoError(t, err)
		}
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		point := func(x, y float64) *sketch.Point {
			p := s.CreatePoint(x, y)
			s.Fix(p)
			return p
		}
		if i == 0 {
			center := point(0, 0)
			p := []*sketch.Point{point(1, 0), point(0, 1), point(-1, 0), point(0, -1)}
			for j := range p {
				s.CreateArc(center, p[j], p[(j+1)%len(p)])
			}
		} else {
			coords := [][2]float64{{1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}}
			controls := make([]*sketch.Point, len(coords))
			for j, p := range coords {
				controls[j] = point(p[0], p[1])
			}
			low, high := controls[len(controls)-1], controls[0]
			if i == 2 {
				slices.Reverse(controls)
			}
			_, err = s.CreateSpline(controls...)
			require.NoError(t, err)
			a, b := point(1, -0.5), point(1, 0.5)
			s.CreateLine(low, a)
			s.CreateLine(a, b)
			s.CreateLine(b, high)
		}
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		for _, profile := range s.Profiles() {
			if profile.Valid && len(profile.Holes) == 0 && len(profile.Outer) == 4 {
				sections[i] = decad.LoftSection{Sketch: s, Profile: profile}
			}
		}
		require.NotNil(t, sections[i].Profile)
	}
	doc := decad.New()
	_, err := decad.LoftSections(t.Context(), doc, sections...)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "spline record directions differ")
	require.Empty(t, doc.Bodies())
}

// circularFitReferenceVolume independently integrates dense section polygons.
// The first curved edge is the Sketch control-point spline.
func circularFitReferenceVolume(t *testing.T, offsets [2]float64) float64 {
	t.Helper()
	const samples = 256
	fit := func(u, offset float64) (float64, float64) {
		points := [][2]float64{{1, 0}, {0.75 + offset, 0.25 + offset},
			{0.5 + offset, 0.5 + offset}, {0.25 + offset, 0.75 + offset}, {0, 1}}
		x, y, err := geom.EvalCubicBSpline(points, u)
		require.NoError(t, err)
		return x, y
	}
	sectionArea := func(z float64) float64 {
		band, t := 0, z
		if z > 1 {
			band, t = 1, z-1
		}
		s := t * t * (3 - 2*t)
		var first, previous [2]float64
		var twiceArea float64
		for edge := range 4 {
			for j := range samples {
				u := float64(j) / samples
				x0, y0 := (1-u*u)/(1+u*u), 2*u/(1+u*u)
				if edge > 0 {
					rotations := [][2]float64{{x0, y0}, {-y0, x0}, {-x0, -y0}, {y0, -x0}}
					x0, y0 = rotations[edge][0], rotations[edge][1]
				}
				endpoint := func(offset float64) (float64, float64) {
					if edge == 0 {
						return fit(u, offset)
					}
					v := [][2]float64{{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 0}}
					return (1-u)*v[edge][0] + u*v[edge+1][0], (1-u)*v[edge][1] + u*v[edge+1][1]
				}
				if band == 1 {
					x0, y0 = endpoint(offsets[0])
				}
				x1, y1 := endpoint(offsets[band])
				p := [2]float64{(1-s)*x0 + s*x1, (1-s)*y0 + s*y1}
				if edge == 0 && j == 0 {
					first = p
				} else {
					twiceArea += previous[0]*p[1] - previous[1]*p[0]
				}
				previous = p
			}
		}
		return (twiceArea + previous[0]*first[1] - previous[1]*first[0]) / 2
	}
	// Simpson integration samples each band's smooth degree-six area law.
	var total float64
	for band := range 2 {
		var sum float64
		for j := 0; j <= samples; j++ {
			weight := 2.0
			if j == 0 || j == samples {
				weight = 1
			} else if j%2 != 0 {
				weight = 4
			}
			sum += weight * sectionArea(float64(band)+float64(j)/samples)
		}
		total += sum / (3 * samples)
	}
	return total
}

func TestLoftSectionsCircularFitDiamond(t *testing.T) {
	t.Parallel()
	sections := circularFitSections(t, [2]float64{}, 0, 1)
	doc := decad.New()
	body, err := decad.LoftSections(t.Context(), doc, sections...)
	require.NoError(t, err)
	require.Equal(t, decad.BodySolid, body.Kind())
	require.Len(t, body.Faces(), 6)
	volume, err := body.Volume()
	require.NoError(t, err)
	// The first band is the rational circle-to-diamond blend; the second is
	// a unit-height diamond prism. The integral is independent of the mesh.
	want := (22*math.Pi+8+18*math.Log(2))/35 + 2
	require.InDelta(t, want, volume.Value.Base(), volume.Bound.Base())
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.LessOrEqual(t, mesh.Bound().Base(), 0.05)
	closed, signed := meshIsWatertight(mesh)
	require.True(t, closed)
	require.Greater(t, signed, 0.0)
	require.InDelta(t, want, signed, volume.Bound.Base())
	for _, vertex := range mesh.Vertices() {
		if vertex.Z == 0 {
			require.InDelta(t, 1, vertex.X*vertex.X+vertex.Y*vertex.Y, 1e-12)
		}
		if vertex.Z == 2 {
			require.InDelta(t, 1, math.Abs(vertex.X)+math.Abs(vertex.Y), 1e-12)
		}
	}
}

func TestLoftSectionsCircularFitVariesAndRefusesAtomically(t *testing.T) {
	t.Parallel()
	sections := circularFitSections(t, [2]float64{0.08, 0.15}, 0, 1)
	doc := decad.New()
	body, err := decad.LoftSections(t.Context(), doc, sections...)
	require.NoError(t, err)
	require.Len(t, body.Faces(), 6)
	for i := range 4 {
		ref := featureRefWithRole(t, body, fmt.Sprintf("side(0,%d)", i))
		_, err := decad.Faces(decad.FaceCreatedBy(ref)).Exactly(1).SelectFaces(body)
		require.NoError(t, err)
	}
	volume, err := body.Volume()
	require.NoError(t, err)
	require.Greater(t, volume.Value.Base(), 0.0)
	require.Positive(t, volume.Bound.Base())
	reference := circularFitReferenceVolume(t, [2]float64{0.08, 0.15})
	require.InDelta(t, reference, volume.Value.Base(), volume.Bound.Base()+1e-5)
	area, err := body.Area()
	require.NoError(t, err)
	require.Greater(t, area.Value.Base(), 0.0)
	require.Positive(t, area.Bound.Base())
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	closed, signed := meshIsWatertight(mesh)
	require.True(t, closed)
	require.Greater(t, signed, 0.0)
	for _, z := range []float64{1, 2} {
		want := 0.5 + 0.08
		if z == 2 {
			want = 0.5 + 0.15
		}
		found := false
		for _, v := range mesh.Vertices() {
			found = found || v.Z == z && math.Abs(v.X-want) < 1e-12 && math.Abs(v.Y-want) < 1e-12
		}
		require.True(t, found, "the fitted section midpoint at z=%g must be held", z)
	}

	cancelled, cancel := context.WithCancel(t.Context())
	cancel()
	before := len(doc.Bodies())
	_, err = decad.LoftSections(cancelled, doc, sections...)
	require.ErrorIs(t, err, context.Canceled)
	require.Len(t, doc.Bodies(), before)
	// A zero-area polar crossing is refused before a body is committed.
	bad := circularFitSections(t, [2]float64{-0.6, -0.6}, 0, 1)
	_, err = decad.LoftSections(t.Context(), doc, bad...)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Len(t, doc.Bodies(), before)
}

func TestLoftSectionsCircularFitRotatedSourceSeam(t *testing.T) {
	t.Parallel()
	sections := circularFitSections(t, [2]float64{}, 1, 1)
	body, err := decad.LoftSections(t.Context(), decad.New(), sections...)
	require.NoError(t, err)
	require.Len(t, body.Faces(), 6)
	ref := featureRefWithRole(t, body, "side(0,0)")
	_, err = decad.Faces(decad.FaceCreatedBy(ref)).Exactly(1).SelectFaces(body)
	require.NoError(t, err)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestLoftSectionsCircularFitTransverseHole(t *testing.T) {
	t.Parallel()
	sections := circularFitSections(t, [2]float64{0.08, 0.15}, 0, 2)
	doc := decad.New()
	body, err := decad.LoftSections(t.Context(), doc, sections...)
	require.NoError(t, err)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	before, err := body.Volume()
	require.NoError(t, err)
	require.True(t, meshContains(t, mesh, r3.Vec{Z: 0.5}))
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.YZ(), -2)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(0, 0.5)
	s.Fix(center)
	s.CreateCircle(center, 0.1)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	tool, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(4), Dir: decad.Along})
	require.NoError(t, err)
	bored, err := decad.Cut(t.Context(), body, tool)
	require.NoError(t, err)
	require.Equal(t, decad.BodySolid, bored.Kind())
	cutMesh, err := bored.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, cutMesh.BoundaryVerified())
	require.True(t, cutMesh.VolumeVerified())
	after, err := bored.Volume()
	require.NoError(t, err)
	require.Greater(t, before.Value.Base()-after.Value.Base(), 0.02)
	require.Less(t, before.Value.Base()-after.Value.Base(), 0.1)
	// The bore has a 0.1 mm radius. Probe both its interior and material
	// beyond the wall, separated from the held boundary by the mesh bound.
	require.Less(t, cutMesh.Bound().Base(), 0.05)
	require.False(t, meshContains(t, cutMesh, r3.Vec{Y: 0.04, Z: 0.5}))
	require.True(t, meshContains(t, cutMesh, r3.Vec{Y: 0.2, Z: 0.5}))

	// A separate second hole exercises a later Boolean on the derived body.
	second, err := w.CreateSketch(plane)
	require.NoError(t, err)
	secondCenter := second.CreatePoint(0, 0.8)
	second.Fix(secondCenter)
	second.CreateCircle(secondCenter, 0.1)
	_, err = second.Solve(t.Context())
	require.NoError(t, err)
	secondTool, err := decad.Extrude(t.Context(), doc, second, second.Profiles()[0],
		decad.Distance{D: units.Millimeters(4), Dir: decad.Along})
	require.NoError(t, err)
	chained, err := decad.Cut(t.Context(), bored, secondTool)
	require.NoError(t, err)
	require.Equal(t, decad.BodySolid, chained.Kind())
	chainedMesh, err := chained.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, chainedMesh.BoundaryVerified())
	require.True(t, chainedMesh.VolumeVerified())
	require.False(t, meshContains(t, chainedMesh, r3.Vec{Z: 0.5}))
	require.False(t, meshContains(t, chainedMesh, r3.Vec{Z: 0.8}))
	require.True(t, meshContains(t, chainedMesh, r3.Vec{Z: 0.65}))
}
