package apitest_test

import (
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func threePrismUnionInputs(t *testing.T, g r3.Frame, thickness float64) (*decad.Document, [3]*decad.Body) {
	t.Helper()
	ctx := t.Context()
	doc := decad.New()
	type hole struct{ x, y, r float64 }
	panel := func(x0, y0, x1, y1, length float64, origin, u, v r3.Vec, holes ...hole) *decad.Body {
		world := sketch.NewWorld()
		s, err := world.CreateSketch(world.XY())
		require.NoError(t, err)
		rect := s.CreateRectangle(x0, y0, x1, y1)
		s.AddConstraint(sketch.NewHorizontalDistance(s.Origin(), rect.A, x0), sketch.NewVerticalDistance(s.Origin(), rect.A, y0),
			sketch.NewHorizontalDistance(rect.A, rect.B, x1-x0), sketch.NewVerticalDistance(rect.B, rect.C, y1-y0))
		for _, h := range holes {
			centre := s.Origin()
			if h.x != 0 || h.y != 0 {
				centre = s.CreatePoint(h.x, h.y)
				s.AddConstraint(sketch.NewHorizontalDistance(s.Origin(), centre, h.x), sketch.NewVerticalDistance(s.Origin(), centre, h.y))
			}
			s.AddConstraint(sketch.NewRadius(s.CreateCircle(centre, h.r), h.r))
		}
		_, err = s.Solve(ctx)
		require.NoError(t, err)
		var profile *sketch.Profile
		for _, candidate := range s.Profiles() {
			if candidate.Valid && len(candidate.Holes) == len(holes) {
				require.True(t, profile == nil)
				profile = candidate
			}
		}
		require.True(t, profile != nil)
		body, err := decad.Extrude(ctx, doc, s, profile, decad.Distance{D: units.Millimeters(length), Dir: decad.Along})
		require.NoError(t, err)
		f, err := r3.NewFrame(origin, u, v)
		require.NoError(t, err)
		xf, err := r3.FromFrame(f)
		require.NoError(t, err)
		body, err = decad.Transform(ctx, body, xf)
		require.NoError(t, err)
		return body
	}
	bridge := panel(-2, -3, 2, 3, 1, g.ToWorld(r3.Vec{Z: -.5}), g.U(), g.V(), hole{0, 0, .75})
	left := panel(.8, -1, 4, 1, thickness, g.ToWorld(r3.Vec{Y: -2}), g.U(), g.N(), hole{3, 0, .25})
	right := panel(.8, -1, 4, 1, thickness, g.ToWorld(r3.Vec{Y: 2 + thickness}), g.U(), g.N(), hole{3, 0, .25})
	return doc, [3]*decad.Body{bridge, left, right}
}

func TestUnionThreeRegroupsCoarseHeldContact(t *testing.T) {
	ctx := t.Context()
	g, err := r3.NewFrame(r3.Vec{X: .25, Y: .5, Z: .75}, r3.Vec{X: -.6, Z: -.8}, r3.Vec{X: .8, Z: -.6})
	require.NoError(t, err)
	doc, inputs := threePrismUnionInputs(t, g, 2)
	before := len(doc.OperationHistory().Operations())
	result, err := decad.Union(ctx, inputs[0], inputs[1], inputs[2])
	require.NoError(t, err)
	require.True(t, tookMeshPath(result))
	require.Empty(t, result.OperationDiagnostics())
	volume, err := result.Volume()
	require.NoError(t, err)
	require.InDelta(t, 47.2-.8125*math.Pi, volume.Value.Base(), volume.Bound.Base()+1e-8)
	mesh, err := result.Tessellate(ctx, units.Millimeters(.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	ops := doc.OperationHistory().Operations()
	require.Len(t, ops, before+1)
	op := ops[before]
	require.Equal(t, decad.OperationUnion, op.Kind())
	require.Len(t, op.Inputs(), 3)
	for i, input := range inputs {
		require.True(t, input == op.Inputs()[i].Body)
		require.Equal(t, decad.InputConsumed, op.Inputs()[i].Use)
	}
	allowed := make(map[decad.FeatureRef]struct{})
	for _, input := range inputs {
		for _, face := range input.Faces() {
			for _, origin := range face.Origins() {
				allowed[origin] = struct{}{}
			}
		}
	}
	for i, face := range result.Faces() {
		require.NotEmpty(t, face.Origins(), "result face %d", i)
		for _, origin := range face.Origins() {
			_, found := allowed[origin]
			require.True(t, found, "result face %d retains a source origin", i)
		}
	}
}

func TestUnionThreeRejectsAdjustedRegroup(t *testing.T) {
	g, err := r3.NewFrame(r3.Vec{X: .25, Y: .5, Z: .75},
		r3.Vec{X: 1, Y: 2, Z: 3}, r3.Vec{X: 4, Y: -2})
	require.NoError(t, err)
	doc, inputs := threePrismUnionInputs(t, g, 1)
	before := doc.OperationHistory().Operations()
	_, err = decad.Union(t.Context(), inputs[0], inputs[1], inputs[2])
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "first operand holds its mesh")
	after := doc.OperationHistory().Operations()
	require.Equal(t, len(before), len(after))
	for i := range before {
		require.True(t, before[i] == after[i], "history operation %d", i)
	}
	live := doc.Bodies()
	require.Equal(t, len(inputs), len(live))
	for _, input := range inputs {
		require.True(t, slices.Contains(live, input))
	}
}
