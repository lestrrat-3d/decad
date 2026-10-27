package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/orthogonal-shell-design.md O12's public-surface tests for WithShellFaceThickness.

func TestShellGivesEachDirectionItsOwnThickness(t *testing.T) {
	t.Parallel()
	top := decad.Faces(decad.Facing(r3.NewVec(0, 0, 1)))
	down, east := r3.NewVec(0, 0, -1), r3.NewVec(1, 0, 0)
	shell := func(opts ...decad.ShellOption) *decad.Body {
		_, block := blockSolid(t, r3.NewVec(0, 0, 0), r3.NewVec(40, 30, 20))
		out, err := decad.Shell(t.Context(), block, top, units.Millimeters(1), opts...)
		require.NoError(t, err)
		return out
	}
	// The block is 24000 mm³. A 3 mm floor leaves the cavity 38 x 28 x 17 open at the top.
	floor := shell(decad.WithShellFaceThickness(down, units.Millimeters(3)))
	requireVolume(t, floor, 24000-38*28*17)
	// A 4 mm east wall as well: the cavity is 35 x 28 x 17.
	both := shell(decad.WithShellFaceThickness(down, units.Millimeters(3)), decad.WithShellFaceThickness(east, units.Millimeters(4)))
	requireVolume(t, both, 24000-35*28*17)
	for point, want := range map[r3.Vec]decad.PointLocation{
		r3.NewVec(38, 15, 10):  decad.PointInside,  // in the 4 mm east wall, which a 1 mm wall would not reach
		r3.NewVec(37, 15, 10):  decad.PointInside,  // still inside it, 3 mm in
		r3.NewVec(35, 15, 10):  decad.PointOutside, // the cavity
		r3.NewVec(1.5, 15, 10): decad.PointOutside, // west wall stays 1 mm
		r3.NewVec(20, 15, 2.5): decad.PointInside,  // the 3 mm floor
		r3.NewVec(20, 15, 3.5): decad.PointOutside,
	} {
		got, err := both.LocatePoint(t.Context(), point, units.Millimeters(0.01))
		require.NoError(t, err)
		require.Equal(t, want, got, "%v", point)
	}
	// Naming nothing different from the base thickness changes nothing.
	same := shell(decad.WithShellFaceThickness(east, units.Millimeters(1)))
	requireVolume(t, same, 24000-38*28*19)
	box, err := both.Bounds()
	require.NoError(t, err)
	require.True(t, box.Max.Equal(r3.NewVec(40, 30, 20), 1e-12), "the outer skin does not move")
	require.False(t, math.IsNaN(box.Min.X))
}

func TestShellFaceThicknessRefusals(t *testing.T) {
	t.Parallel()
	top := decad.Faces(decad.Facing(r3.NewVec(0, 0, 1)))
	_, block := blockSolid(t, r3.NewVec(0, 0, 0), r3.NewVec(40, 30, 20))
	down := r3.NewVec(0, 0, -1)
	shell := func(opts ...decad.ShellOption) error {
		_, err := decad.Shell(t.Context(), block, top, units.Millimeters(1), opts...)
		return err
	}
	require.ErrorIs(t, shell(decad.WithShellFaceThickness(r3.NewVec(1, 1, 0), units.Millimeters(2))), decad.ErrDegenerate, "a skew direction")
	require.ErrorIs(t, shell(decad.WithShellFaceThickness(down, units.Millimeters(0))), decad.ErrDegenerate, "zero thickness")
	require.ErrorIs(t, shell(decad.WithShellFaceThickness(down, units.Degrees(2))), decad.ErrUnitKind)
	require.ErrorIs(t, shell(decad.WithShellFaceThickness(down, units.Millimeters(-2))), decad.ErrNegativeMagnitude)
	require.ErrorIs(t, shell(decad.WithShellFaceThickness(down, units.Millimeters(2)), decad.WithShellFaceThickness(down, units.Millimeters(3))),
		decad.ErrDegenerate, "one direction twice")
	require.ErrorIs(t, shell(decad.WithShellFaceThickness(down, units.Millimeters(2)), decad.WithShellSense(decad.Outward)),
		decad.ErrDegenerate, "only an Inward shell")
	// A walls so thick the cavity vanishes.
	require.ErrorIs(t, shell(decad.WithShellFaceThickness(down, units.Millimeters(25))), decad.ErrDegenerate)
	// A solid that is not faceted takes no thickness per direction.
	doc := decad.New()
	prism := boxBody(t, doc, 0, 0, 40, 30, 20)
	_, err := decad.Shell(t.Context(), prism, decad.Faces(decad.Facing(r3.NewVec(0, 0, 1))), units.Millimeters(1),
		decad.WithShellFaceThickness(down, units.Millimeters(3)))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}
