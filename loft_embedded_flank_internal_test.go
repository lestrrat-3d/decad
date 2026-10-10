package decad

import (
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func internalEmbeddedGearSketch(t *testing.T, world *sketch.World, plane *sketch.Plane,
	teeth, samples int) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	const module = 0.5
	pressure := 30 * math.Pi / 180
	pitch := module * float64(teeth) / 2
	base := pitch * math.Cos(pressure)
	root := (module*float64(teeth) - 2.5*module) / 2
	tip := (module*float64(teeth) + 2*module) / 2
	involute := func(radius float64) (float64, float64) {
		angle := math.Tan(math.Acos(base / radius))
		return base * (math.Cos(angle) + angle*math.Sin(angle)),
			base * (math.Sin(angle) - angle*math.Cos(angle))
	}
	rotate := func(x, y, angle float64) (float64, float64) {
		sine, cosine := math.Sincos(angle)
		return x*cosine - y*sine, x*sine + y*cosine
	}
	px, py := involute(pitch)
	rotation := math.Pi/(2*float64(teeth)) - math.Atan2(-py, px)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	for tooth := range teeth {
		angle := 2 * math.Pi * float64(tooth) / float64(teeth)
		left, right := make([]*sketch.Point, samples), make([]*sketch.Point, samples)
		for i := range samples {
			radius := base + (tip-base)*float64(i)/float64(samples-1)
			x, y := involute(radius)
			flankX, flankY := rotate(x, -y, rotation)
			leftX, leftY := rotate(flankX, flankY, angle)
			rightX, rightY := rotate(flankX, -flankY, angle)
			left[i], right[i] = s.CreatePoint(leftX, leftY), s.CreatePoint(rightX, rightY)
			s.Fix(left[i])
			s.Fix(right[i])
		}
		_, err = s.CreateFitSpline(right...)
		require.NoError(t, err)
		s.CreateArc(center, right[samples-1], left[samples-1])
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
	require.Len(t, profile.Outer, 4*teeth)
	return s, profile
}

func requireEmbeddedFlanks(t *testing.T, body *Body, source []fitSplineSeg) {
	t.Helper()
	payload, ok := body.payload.(loftPayload)
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
	const teeth, samples = 24, 5
	world := sketch.NewWorld()
	upperFrame, err := r3.NewFrame(r3.NewVec(0, 0, 2), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	upper, err := world.CreatePlaneFromFrame(upperFrame)
	require.NoError(t, err)
	s0, p0 := internalEmbeddedGearSketch(t, world, world.XY(), teeth, samples)
	s1, p1 := internalEmbeddedGearSketch(t, world, upper, teeth, samples)
	sourceRecord, _, _, err := recordProfile(s0, p0)
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
	require.Len(t, source, 2*teeth)
	require.Len(t, rootCorners, 2*teeth)
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
	require.Len(t, corners, 2*teeth)
	query := Edges(ParallelTo(axis), EndpointAt(corners[0]))
	for _, corner := range corners[1:] {
		query.Or(ParallelTo(axis), EndpointAt(corner))
	}
	filleted, err := lofted.Fillet(t.Context(), query.Exactly(2*teeth), units.Millimeters(0.01))
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
	tool, err := doc.Extrude(toolSketch, toolSketch.Profiles()[0], Distance{
		D: units.Millimeters(4), Dir: Along,
	})
	require.NoError(t, err)
	bored, err := Cut(t.Context(), filleted, tool)
	require.NoError(t, err)
	requireEmbeddedFlanks(t, bored, source)
}
