package decad

import (
	"math"
	"math/big"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestBevelFitSeatPointEnclosesRecordedMeridianCrossing(t *testing.T) {
	world := sketch.NewWorld()
	plane, err := world.CreateOffsetPlane(world.YZ(), 8)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	fit, err := s.CreateFitSpline(s.CreatePoint(7.5, 0), s.CreatePoint(8.5, 0))
	require.NoError(t, err)
	frame, err := plane.Frame()
	require.NoError(t, err)
	blank := &bevelBlankSource{
		toeRoot: Point2{U: 7, V: 7.001},
		ded:     Point2{U: 9, V: 9.001},
		rootCone: Cone{
			Origin: r3.Vec{}, Axis: r3.Vec{X: 1},
			HalfAngle: units.Radians(math.Pi / 4),
		},
	}
	cone := blank.coneSection()
	cert, err := s.CertifyConeFitCrossing(fit, cone)
	require.NoError(t, err)
	src := &pointSectionSource{sketch: s, revision: s.Revision(), frame: frame}
	point, bound, interval, ok := bevelFitSeatPoint(src, cert, blank)
	require.True(t, ok)
	require.Greater(t, interval[1], interval[0])
	require.Less(t, bound, 0.03)
	m, b := blank.meridianEquation()
	exactRadius := new(big.Rat).Add(new(big.Rat).Mul(m, proofarith.FloatRat(8)), b)
	radius := proofbound.RatFloatUp(exactRadius)
	require.LessOrEqual(t, math.Abs(point.X-8), bound)
	require.LessOrEqual(t, math.Abs(point.Y-radius), bound)
	require.LessOrEqual(t, math.Abs(point.Z), bound)
}

func TestBevelFitStationEdgeRequiresOneStrictBracket(t *testing.T) {
	world := sketch.NewWorld()
	plane, err := world.CreateOffsetPlane(world.YZ(), 8)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	fit, err := s.CreateFitSpline(s.CreatePoint(7.5, 0), s.CreatePoint(8.5, 0))
	require.NoError(t, err)
	cert, err := s.CertifyConeFitCrossing(fit, sketch.ConeSection{
		Apex: r3.Vec{}, Axis: r3.Vec{X: 1}, Slope: 1,
	})
	require.NoError(t, err)
	source := &pointSectionSource{sketch: s, revision: s.Revision(), entities: []sketch.Entity{fit}}
	base := facetedPayload{pointSection: &pointSectionRecord{
		source: source, sourceStationRound: 1e-12,
		profile: profileRecord{Outer: loopRecord{Segments: []curveSegment{
			fitSplineSeg{TStart: 0, TEnd: 1},
		}}},
	}}
	root := [2]float64{0.49, 0.51}
	base.pointSection.sourceStation = []Point2{{U: 7.5}, {U: 8.5}}
	edge, ok := bevelFitStationEdge(base, []int{0, 0}, source, 0, cert, root)
	require.True(t, ok)
	require.Equal(t, 0, edge)

	base.pointSection.sourceStation = []Point2{{U: 7.5}, {U: 8}, {U: 8.5}}
	_, ok = bevelFitStationEdge(base, []int{0, 0, 0}, source, 0, cert, root)
	require.False(t, ok, "a station inside the crossing interval cannot select either chord")

	base.pointSection.sourceStation = []Point2{{U: 7.5}, {U: 8.5}, {U: 7.5}, {U: 8.5}}
	_, ok = bevelFitStationEdge(base, []int{0, 0, 0, 0}, source, 0, cert, root)
	require.False(t, ok, "two matching chords must not be chosen by distance")

	base.pointSection.profile.Outer.Segments[0] = fitSplineSeg{TStart: 1, TEnd: 0}
	base.pointSection.sourceStation = []Point2{{U: 8.5}, {U: 7.5}}
	edge, ok = bevelFitStationEdge(base, []int{0, 0}, source, 0, cert, root)
	require.True(t, ok)
	require.Equal(t, 0, edge)
}

func TestBevelFitExteriorPathChargesSeatOnce(t *testing.T) {
	world := sketch.NewWorld()
	plane, err := world.CreateOffsetPlane(world.YZ(), 8)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	frame, err := plane.Frame()
	require.NoError(t, err)
	points := [6]*sketch.Point{
		s.CreatePoint(7.5, -0.1), s.CreatePoint(7.5, 0.1),
		s.CreatePoint(7.6, 0.1), s.CreatePoint(8.5, 0.1),
		s.CreatePoint(8.5, -0.1), s.CreatePoint(7.6, -0.1),
	}
	center := s.CreatePoint(0, 0)
	fitUpper, err := s.CreateFitSpline(points[2], points[3])
	require.NoError(t, err)
	fitLower, err := s.CreateFitSpline(points[4], points[5])
	require.NoError(t, err)
	blank := &bevelBlankSource{
		toeRoot: Point2{U: 7, V: 7.01},
		ded:     Point2{U: 9, V: 9.01},
		rootCone: Cone{
			Origin: r3.Vec{}, Axis: r3.Vec{X: 1},
			HalfAngle: units.Radians(math.Pi / 4),
		},
	}
	cone := blank.coneSection()
	source := &pointSectionSource{
		sketch: s, frame: frame,
		entities: []sketch.Entity{
			s.CreateArc(center, points[0], points[1]),
			s.CreateLine(points[1], points[2]),
			fitUpper,
			s.CreateArc(center, points[4], points[3]),
			fitLower,
			s.CreateLine(points[5], points[0]),
		},
	}
	source.revision = s.Revision()
	upper, err := s.CertifyConeFitCrossing(fitUpper, cone)
	require.NoError(t, err)
	lower, err := s.CertifyConeFitCrossing(fitLower, cone)
	require.NoError(t, err)
	boundary := &bevelToothBoundary{
		insideArc: 0, fitRoute: true,
		fitIndex: [2]int{2, 4}, fitCross: [2]sketch.ConeFitCrossing{upper, lower},
	}
	base := facetedPayload{meshBound: 0.02}
	base.pointSection = &pointSectionRecord{
		source: source, sourceStationRound: 1e-12,
		profile: profileRecord{Outer: loopRecord{Segments: make([]curveSegment, len(points))}},
	}
	base.pointSection.profile.Outer.Segments[2] = fitSplineSeg{TStart: 0, TEnd: 1}
	base.pointSection.profile.Outer.Segments[4] = fitSplineSeg{TStart: 0, TEnd: 1}
	base.verts = append(base.verts, r3.Vec{})
	for _, point := range points {
		local := point.Geometry()
		base.pointSection.sourceStation = append(base.pointSection.sourceStation,
			Point2{U: local.X, V: local.Y})
		base.verts = append(base.verts, frame.Origin().Add(
			frame.U().Scale(local.X)).Add(frame.V().Scale(local.Y)))
	}
	for i := range points {
		base.tris = append(base.tris, [3]int{0, i + 1, (i+1)%len(points) + 1})
		base.src = append(base.src, i)
	}
	_, upperBound, _, ok := bevelFitSeatPoint(source, upper, blank)
	require.True(t, ok)
	_, lowerBound, _, ok := bevelFitSeatPoint(source, lower, blank)
	require.True(t, ok)
	path, charged, err := bevelExteriorPath(base, boundary, source, blank)
	require.NoError(t, err)
	require.Greater(t, len(path), 2)
	require.Equal(t, max(upperBound, lowerBound), charged)
}
