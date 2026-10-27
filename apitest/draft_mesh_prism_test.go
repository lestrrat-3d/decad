package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/orthogonal-boolean-design.md B21's public-surface tests: Draft of every wall of a mesh solid
// that is a straight prism.

func meshWalls() *decad.FaceQuery {
	return decad.Faces(decad.Facing(r3.NewVec(1, 0, 0))).Or(decad.Facing(r3.NewVec(-1, 0, 0))).
		Or(decad.Facing(r3.NewVec(0, 1, 0))).Or(decad.Facing(r3.NewVec(0, -1, 0)))
}

func TestDraftOfAMeshBoxLeansEveryWallAboutTheBottom(t *testing.T) {
	t.Parallel()
	doc, block := blockSolid(t, r3.NewVec(0, 0, 0), r3.NewVec(20, 10, 8))
	bottom := decad.NeutralFace{Body: block, Face: decad.Faces(decad.Facing(r3.NewVec(0, 0, -1)))}
	out, err := decad.Draft(t.Context(), block, meshWalls(), bottom, units.Degrees(5))
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies())
	k := math.Tan(5 * math.Pi / 180)
	// Each wall moves in t*k at the height t, so the section is (20 - 2tk)(10 - 2tk): 200 - 60tk + 4t^2k^2 integrated to 8.
	want := 200*8 - 30*k*64 + 4*k*k*512/3
	volume, err := out.Volume()
	require.NoError(t, err)
	require.InDelta(t, want, volume.Value.Base(), volume.Bound.Base()+1e-6)
	box, err := out.Bounds()
	require.NoError(t, err)
	require.InDelta(t, 8, box.Max.Z, 1e-9)
	require.InDelta(t, 0, box.Min.Z, 1e-9)
	// At the top the section has lost 8k from each side: 20 - 16k by 10 - 16k, centred on the bottom's centre.
	top := 8 * k
	for point, want := range map[r3.Vec]decad.PointLocation{
		r3.NewVec(top+0.1, 5, 7.99):   decad.PointInside,
		r3.NewVec(top-0.1, 5, 7.99):   decad.PointOutside,
		r3.NewVec(0.1, 5, 0.01):       decad.PointInside,
		r3.NewVec(20-top-0.1, 5, 7.9): decad.PointInside,
		r3.NewVec(20-top+0.1, 5, 7.9): decad.PointOutside,
	} {
		got, err := out.LocatePoint(t.Context(), point, units.Millimeters(0.01))
		require.NoError(t, err)
		require.Equal(t, want, got, "%v", point)
	}
}

func TestDraftOfAMeshBoxAboutTheTopWidensTheOtherWay(t *testing.T) {
	t.Parallel()
	_, block := blockSolid(t, r3.NewVec(0, 0, 0), r3.NewVec(20, 10, 8))
	top := decad.NeutralFace{Body: block, Face: decad.Faces(decad.Facing(r3.NewVec(0, 0, 1)))}
	out, err := decad.Draft(t.Context(), block, meshWalls(), top, units.Degrees(5))
	require.NoError(t, err)
	k := math.Tan(5 * math.Pi / 180)
	// About the top the body narrows toward the bottom: the same volume as narrowing upward from the bottom.
	want := 200*8 - 30*k*64 + 4*k*k*512/3
	volume, err := out.Volume()
	require.NoError(t, err)
	require.InDelta(t, want, volume.Value.Base(), volume.Bound.Base()+1e-6)
	box, err := out.Bounds()
	require.NoError(t, err)
	require.InDelta(t, 0, box.Min.Z, 1e-9, "the body stays where the block was")
	require.InDelta(t, 8, box.Max.Z, 1e-9)
	got, err := out.LocatePoint(t.Context(), r3.NewVec(8*k-0.1, 5, 0.01), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointOutside, got, "the bottom is the narrow end")
}

func TestDraftOfAMeshSolidRefusesWhatIsNotAllWallsOfAPrism(t *testing.T) {
	t.Parallel()
	_, block := blockSolid(t, r3.NewVec(0, 0, 0), r3.NewVec(20, 10, 8))
	bottom := decad.NeutralFace{Body: block, Face: decad.Faces(decad.Facing(r3.NewVec(0, 0, -1)))}
	// Only two of the four walls.
	two := decad.Faces(decad.Facing(r3.NewVec(1, 0, 0))).Or(decad.Facing(r3.NewVec(-1, 0, 0)))
	_, err := decad.Draft(t.Context(), block, two, bottom, units.Degrees(5))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	// A cap among the walls.
	withCap := meshWalls().Or(decad.Facing(r3.NewVec(0, 0, 1)))
	_, err = decad.Draft(t.Context(), block, withCap, bottom, units.Degrees(5))
	require.Error(t, err)
	// An L: a section of eight corners is still a prism, but a body of three levels is not.
	doc := decad.New()
	a := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(20, 10, 8))
	c := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(10, 10, 4))
	stepped, err := decad.Union(t.Context(), a, c)
	require.NoError(t, err)
	_, err = decad.Draft(t.Context(), stepped, meshWalls(),
		decad.NeutralFace{Body: stepped, Face: decad.Faces(decad.Facing(r3.NewVec(0, 0, -1)))}, units.Degrees(5))
	require.Error(t, err)
	require.Equal(t, []*decad.Body{stepped}, doc.Bodies(), "a refusal leaves the body live")
}
