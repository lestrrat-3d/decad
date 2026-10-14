package decad

import (
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/sectioncert"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func internalEmbeddedFlankSketch(t *testing.T, world *sketch.World,
	plane *sketch.Plane) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	// Two short curved lobes keep each retained root-circle arc below a
	// half-turn, the range that the exact departure proof can certify.
	const root = 5.375
	flank := [][2]float64{
		{5.159280153566717, 0.6179225655473234},
		{5.491823558586549, 0.5775989367209748},
		{5.829613089125064, 0.46433458292302054},
		{6.166922954050798, 0.29632382480791636},
		{6.499520490645434, 0.07895183139186746},
	}
	for lobe := range 2 {
		turnSin, turnCos := math.Sincos(math.Pi * float64(lobe))
		left, right := make([]*sketch.Point, len(flank)), make([]*sketch.Point, len(flank))
		for i, point := range flank {
			flankX, flankY := point[0], point[1]
			left[i] = s.CreatePoint(flankX*turnCos-flankY*turnSin, flankX*turnSin+flankY*turnCos)
			right[i] = s.CreatePoint(flankX*turnCos+flankY*turnSin, flankX*turnSin-flankY*turnCos)
			s.Fix(left[i])
			s.Fix(right[i])
		}
		_, err = s.CreateFitSpline(right...)
		require.NoError(t, err)
		s.CreateArc(center, right[len(right)-1], left[len(left)-1])
		_, err = s.CreateFitSpline(left...)
		require.NoError(t, err)
	}
	s.CreateCircle(center, root)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	regions := s.Profiles()
	indices := make([]int, len(regions))
	for i := range indices {
		indices[i] = i
	}
	profile, err := s.UnionProfiles(indices...)
	require.NoError(t, err)
	require.Len(t, profile.Outer, 8)
	return s, profile
}

func requireEmbeddedFlanks(t *testing.T, body *Body, source []fitSplineSeg) {
	t.Helper()
	payload, ok := body.n.Payload.(loftPayload)
	require.True(t, ok, "the body must retain its loft payload")
	for _, section := range []profileRecord{payload.profile0, payload.profile1} {
		var retained []fitSplineSeg
		for _, segment := range section.Outer.Segments {
			if fit, ok := segment.(fitSplineSeg); ok {
				retained = append(retained, fit)
			}
		}
		require.Len(t, retained, len(source))
		for i, got := range retained {
			want := source[i]
			require.Equal(t, want.Fit, got.Fit, "flank %d retains every original fit point", i)
			if want.TStart < want.TEnd {
				require.Less(t, got.TStart, got.TEnd)
				require.GreaterOrEqual(t, got.TStart, want.TStart)
				require.LessOrEqual(t, got.TEnd, want.TEnd)
			} else {
				require.Greater(t, got.TStart, got.TEnd)
				require.LessOrEqual(t, got.TStart, want.TStart)
				require.GreaterOrEqual(t, got.TEnd, want.TEnd)
			}
		}
	}
}

func TestLoftEmbeddedFlankRecordsSurviveFilletAndBore(t *testing.T) {
	world := sketch.NewWorld()
	upperFrame, err := r3.NewFrame(r3.NewVec(0, 0, 2), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	upper, err := world.CreatePlaneFromFrame(upperFrame)
	require.NoError(t, err)
	s0, p0 := internalEmbeddedFlankSketch(t, world, world.XY())
	s1, p1 := internalEmbeddedFlankSketch(t, world, upper)
	sourceRecord, _, _, err := sectioncert.RecordProfile(s0, p0)
	require.NoError(t, err)
	var source []fitSplineSeg
	var rootCorners []r3.Vec
	for _, segment := range sourceRecord.Outer.Segments {
		if fit, ok := segment.(fitSplineSeg); ok {
			require.Greater(t, math.Min(fit.TStart, fit.TEnd), 0.0,
				"the original flank must be trimmed by the root circle")
			fit.Fit = slices.Clone(fit.Fit)
			source = append(source, fit)
		}
	}
	for _, edge := range p0.Outer {
		if fit, ok := edge.Entity.(*sketch.FitSpline); ok {
			x, y := fit.Eval(edge.TStart)
			rootCorners = append(rootCorners, r3.NewVec(x, y, 0))
		}
	}
	require.Len(t, source, 4)
	require.Len(t, rootCorners, 4)
	doc := New()
	lofted, err := doc.Loft(t.Context(), s0, p0, s1, p1)
	require.NoError(t, err)
	requireEmbeddedFlanks(t, lofted, source)
	axis := r3.NewVec(0, 0, 1)
	var corners []r3.Vec
	axial, err := Edges(ParallelTo(axis)).SelectEdges(lofted)
	require.NoError(t, err)
	for _, edge := range axial {
		point := edge.Start().Position().Value
		for _, want := range rootCorners {
			if math.Hypot(point.X-want.X, point.Y-want.Y) < 1e-8 {
				corners = append(corners, point)
				break
			}
		}
	}
	require.Len(t, corners, 4)
	query := Edges(ParallelTo(axis), EndpointAt(corners[0]))
	for _, corner := range corners[1:] {
		query.Or(ParallelTo(axis), EndpointAt(corner))
	}
	filleted, err := Fillet(t.Context(), lofted, query.Exactly(4), units.Millimeters(0.01))
	require.NoError(t, err)
	requireEmbeddedFlanks(t, filleted, source)
	toolWorld := sketch.NewWorld()
	toolFrame, err := r3.NewFrame(r3.NewVec(0, 0, -1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	toolPlane, err := toolWorld.CreatePlaneFromFrame(toolFrame)
	require.NoError(t, err)
	toolSketch, err := toolWorld.CreateSketch(toolPlane)
	require.NoError(t, err)
	center := toolSketch.CreatePoint(0, 0)
	toolSketch.Fix(center)
	toolSketch.CreateCircle(center, 0.25)
	_, err = toolSketch.Solve(t.Context())
	require.NoError(t, err)
	tool, err := Extrude(t.Context(), doc, toolSketch, toolSketch.Profiles()[0], Distance{
		D: units.Millimeters(4), Dir: Along,
	})
	require.NoError(t, err)
	bored, err := Cut(t.Context(), filleted, tool)
	require.NoError(t, err)
	requireEmbeddedFlanks(t, bored, source)
}
