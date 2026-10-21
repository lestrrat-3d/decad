package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/planarpose"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestPlanarPoseAdmissionGates asks planarpose.SolidAtPose for the exact
// solid of real bodies and breaks one admission fact at a time. A box reads as
// a solid; a sheet, a non-solid, a reflected or invalid pose, and a source
// sphere or cylinder (which keep their exact contact families) are refused,
// as is a curved body at a zero chord.
func TestPlanarPoseAdmissionGates(t *testing.T) {
	doc := New()
	box := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
	ball := internalBallBody(t, doc, 4)
	disc := internalDiscBody(t, doc, 3, 8)

	solidAt := func(b *Body, pose r3.Transform, chord float64) bool {
		t.Helper()
		_, _, ok, err := planarpose.SolidAtPose(t.Context(), proofbound.NewWorkBudget(t.Context()),
			verifyAllMesher{}, bodyNode(b), pose, chord)
		require.NoError(t, err)
		return ok
	}

	require.True(t, solidAt(box, r3.Identity(), 0))
	move, err := r3.Translation(r3.Vec{X: 5})
	require.NoError(t, err)
	require.True(t, solidAt(box, move, 0))

	t.Run("pose", func(t *testing.T) {
		mirror, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
		require.NoError(t, err)
		flip, err := r3.Reflection(mirror)
		require.NoError(t, err)
		require.False(t, solidAt(box, flip, 0))
		require.False(t, solidAt(box, r3.Transform{}, 0))
	})
	t.Run("not a solid", func(t *testing.T) {
		box.n.Solid = false
		defer func() { box.n.Solid = true }()
		require.False(t, solidAt(box, r3.Identity(), 0))
	})
	t.Run("sheet kind", func(t *testing.T) {
		kind := box.n.Kind
		box.n.Kind = topo.BodySheet
		defer func() { box.n.Kind = kind }()
		require.False(t, solidAt(box, r3.Identity(), 0))
	})
	t.Run("nil body", func(t *testing.T) {
		_, _, ok, err := planarpose.SolidAtPose(t.Context(), proofbound.NewWorkBudget(t.Context()),
			verifyAllMesher{}, nil, r3.Identity(), 0)
		require.NoError(t, err)
		require.False(t, ok)
	})
	t.Run("source sphere keeps its exact family", func(t *testing.T) {
		require.False(t, solidAt(ball, r3.Identity(), 1))
	})
	t.Run("source cylinder keeps its exact family", func(t *testing.T) {
		require.False(t, solidAt(disc, r3.Identity(), 1))
	})
	t.Run("curved body at a zero chord", func(t *testing.T) {
		curved := roundedBoxBody(t, doc)
		require.False(t, solidAt(curved, r3.Identity(), 0))
		require.True(t, solidAt(curved, r3.Identity(), 0.5), "a positive chord admits the held mesh")
	})
	t.Run("convexity reads the same snapshot", func(t *testing.T) {
		convex, err := planarpose.Convexity{Mesher: verifyAllMesher{}}.Convex(t.Context(),
			proofbound.NewWorkBudget(t.Context()), bodyNode(box))
		require.NoError(t, err)
		require.True(t, convex)
	})
}

func roundedBoxBody(t *testing.T, doc *Document) *Body {
	t.Helper()
	body := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
	rounded, err := Fillet(t.Context(), body, Edges(), units.Millimeters(1))
	require.NoError(t, err)
	return rounded
}
