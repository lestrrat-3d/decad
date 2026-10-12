package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestLoftMirrorCylinderBoreChamfersExteriorCaps(t *testing.T) {
	doc := decad.New()
	world := sketch.NewWorld()
	middleFrame, err := r3.NewFrame(r3.Vec{Z: 1}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	middlePlane, err := world.CreatePlaneFromFrame(middleFrame)
	require.NoError(t, err)
	bottomSketch, bottomProfile := loftMirrorTabSection(t, world, world.XY(), 0)
	middleSketch, middleProfile := loftMirrorTabSection(t, world, middlePlane, 0.1)
	lower, err := doc.Loft(t.Context(), bottomSketch, bottomProfile, middleSketch, middleProfile)
	require.NoError(t, err)
	upper, err := lower.MirroredCopy(t.Context(), decad.MirrorFrame{Frame: middleFrame})
	require.NoError(t, err)
	pair, err := decad.Union(t.Context(), lower, upper)
	require.NoError(t, err)
	quarterTurn, err := r3.Rotation(r3.Vec{Z: 1}, units.Degrees(90))
	require.NoError(t, err)
	turnedPair, err := pair.PlacedCopy(t.Context(), quarterTurn)
	require.NoError(t, err)
	blank := circleBodyAtZ(t, doc, 0, 3, 0, 2)
	joined, err := decad.CombineJoin(t.Context(), blank, pair, turnedPair)
	require.NoError(t, err)
	bore := circleBodyAtZ(t, doc, 0, 0.5, -1, 4)
	cut, err := decad.Cut(t.Context(), joined, bore)
	require.NoError(t, err)
	outerCaps := decad.Edges(decad.OuterLoopOf(decad.CapStart(cut))).
		Or(decad.OuterLoopOf(decad.CapEnd(cut)))
	chamfered, err := cut.Chamfer(t.Context(), outerCaps, units.Millimeters(0.06))
	require.NoError(t, err)
	mesh, err := chamfered.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Len(t, doc.Bodies(), 1)
}
