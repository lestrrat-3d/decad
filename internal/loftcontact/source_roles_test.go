package loftcontact_test

import (
	"context"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

// wallLine is the source segment from the origin to (u, v).
func wallLine(u, v float64) sectionrecord.LineSeg {
	return sectionrecord.LineSeg{End: sectionrecord.Point2{U: u, V: v}, TStart: 0, TEnd: 1}
}

func wallVertex(u, v, z int64) loftcontact.OutputVertex {
	return loftcontact.OutputVertex{Point: loftcontact.OutputPoint{
		U: big.NewRat(u, 1), V: big.NewRat(v, 1), Z: big.NewRat(z, 1)}}
}

// TestSourceRoles reads each face class and each wall triangle position off a
// planar wall cell. The source corner validity gate has no input of its own: a
// float64 corner is never larger than the rational size cap.
func TestSourceRoles(t *testing.T) {
	t.Parallel()

	// One planar wall cell on v = 0, u in [0, 10], z in [0, 10]; its straight
	// source diagonal runs from (0, 0, 0) to (10, 0, 10).
	lower, upper := []sectionrecord.LineSeg{wallLine(10, 0)}, []sectionrecord.LineSeg{wallLine(10, 0)}
	shell := loftcontact.OutputShell{Vertices: []loftcontact.OutputVertex{
		wallVertex(0, 0, 0), wallVertex(10, 0, 0), wallVertex(10, 0, 10), wallVertex(0, 0, 10)}}
	z0, z1 := big.NewRat(0, 1), big.NewRat(10, 1)
	budget := func() *proofbound.WorkBudget { return proofbound.NewWorkBudget(t.Context()) }
	call := func(face loftcontact.OutputFace, triangle [3]int) ([2]string, int, bool, error) {
		return loftcontact.SourceRoles(face, triangle, shell, lower, upper, z0, z1, budget())
	}

	t.Run("caps", func(t *testing.T) {
		t.Parallel()
		tests := []struct {
			owner, level int
			want         string
			found        bool
		}{
			{0, 0, provenance.RoleCapStart, true}, {1, 0, provenance.RoleCapStart, true},
			{1, 1, provenance.RoleCapEnd, true}, {0, 2, provenance.RoleCapEnd, true},
			{0, 1, "", false}, {1, 2, "", false}, {2, 0, "", false},
		}
		for _, tc := range tests {
			roles, owner, found, err := call(loftcontact.OutputFace{Kind: 0, Owner: tc.owner, Level: tc.level}, [3]int{})
			require.NoError(t, err)
			require.Equal(t, tc.found, found, "owner %d level %d", tc.owner, tc.level)
			if tc.found {
				require.Equal(t, [2]string{tc.want}, roles)
				require.Equal(t, tc.owner, owner)
			}
		}
	})
	t.Run("first operand wall", func(t *testing.T) {
		t.Parallel()
		roles, owner, found, err := call(loftcontact.OutputFace{Kind: 1, Owner: 0, Segment: 3}, [3]int{})
		require.NoError(t, err)
		require.True(t, found)
		require.Equal(t, [2]string{"side(0,3)"}, roles)
		require.Zero(t, owner)
	})
	t.Run("second operand wall halves", func(t *testing.T) {
		t.Parallel()
		face := loftcontact.OutputFace{Kind: 1, Owner: 1, Segment: 0}
		tests := []struct {
			name     string
			triangle [3]int
			want     [2]string
		}{
			{"below the diagonal", [3]int{0, 1, 2}, [2]string{"side(0,0,0)"}},
			{"above the diagonal", [3]int{0, 2, 3}, [2]string{"side(0,0,1)"}},
			{"across the diagonal", [3]int{0, 1, 3}, [2]string{"side(0,0,0)", "side(0,0,1)"}},
		}
		for _, tc := range tests {
			roles, owner, found, err := call(face, tc.triangle)
			require.NoError(t, err, tc.name)
			require.True(t, found, tc.name)
			require.Equal(t, 1, owner, tc.name)
			require.Equal(t, tc.want, roles, tc.name)
		}
	})
	t.Run("faces with no source role", func(t *testing.T) {
		t.Parallel()
		for name, face := range map[string]loftcontact.OutputFace{
			"unknown kind":         {Kind: 2},
			"negative segment":     {Kind: 1, Segment: -1},
			"segment out of range": {Kind: 1, Owner: 1, Segment: 1},
			"third operand":        {Kind: 1, Owner: 2},
		} {
			_, _, found, err := call(face, [3]int{0, 1, 2})
			require.NoError(t, err, name)
			require.False(t, found, name)
		}
	})
	t.Run("wall triangles the cell does not hold", func(t *testing.T) {
		t.Parallel()
		face := loftcontact.OutputFace{Kind: 1, Owner: 1, Segment: 0}
		_, _, found, err := call(face, [3]int{0, 1, 9})
		require.NoError(t, err)
		require.False(t, found, "a vertex id outside the shell")
		off := shell
		off.Vertices = append([]loftcontact.OutputVertex(nil), shell.Vertices...)
		off.Vertices[2] = wallVertex(10, 5, 10)
		_, _, found, err = loftcontact.SourceRoles(face, [3]int{0, 1, 2}, off, lower, upper, z0, z1, budget())
		require.NoError(t, err)
		require.False(t, found, "a vertex off the wall")
		flat := loftcontact.OutputShell{Vertices: []loftcontact.OutputVertex{
			wallVertex(0, 0, 0), wallVertex(0, 0, 0), wallVertex(0, 0, 0)}}
		_, _, found, err = loftcontact.SourceRoles(face, [3]int{0, 1, 2}, flat, lower, upper, z0, z1, budget())
		require.NoError(t, err)
		require.False(t, found, "a degenerate triangle sits on the diagonal's end only")
		_, _, found, err = loftcontact.SourceRoles(face, [3]int{0, 1, 2}, shell, lower, upper, z0, z0, budget())
		require.NoError(t, err)
		require.False(t, found, "a wall of no height")
	})
	t.Run("segment tables shorter than the face", func(t *testing.T) {
		t.Parallel()
		face := loftcontact.OutputFace{Kind: 1, Owner: 1, Segment: 0}
		_, _, found, err := loftcontact.SourceRoles(face, [3]int{0, 1, 2}, shell, lower, nil, z0, z1, budget())
		require.NoError(t, err)
		require.False(t, found, "no upper segment")
		_, _, found, err = loftcontact.SourceRoles(face, [3]int{0, 1, 2}, shell, nil, upper, z0, z1, budget())
		require.NoError(t, err)
		require.False(t, found, "no lower segment")
	})
	t.Run("vertex ids outside the shell", func(t *testing.T) {
		t.Parallel()
		face := loftcontact.OutputFace{Kind: 1, Owner: 1, Segment: 0}
		_, _, found, err := call(face, [3]int{-1, 0, 1})
		require.NoError(t, err)
		require.False(t, found)
	})
	t.Run("a wall of negative height", func(t *testing.T) {
		t.Parallel()
		face := loftcontact.OutputFace{Kind: 1, Owner: 1, Segment: 0}
		_, _, found, err := loftcontact.SourceRoles(face, [3]int{0, 1, 1}, shell, lower, upper, z0, big.NewRat(-10, 1), budget())
		require.NoError(t, err)
		require.False(t, found)
	})
	t.Run("a wall running along v", func(t *testing.T) {
		t.Parallel()
		face := loftcontact.OutputFace{Kind: 1, Owner: 1, Segment: 0}
		vLower, vUpper := []sectionrecord.LineSeg{wallLine(0, 10)}, []sectionrecord.LineSeg{wallLine(0, 10)}
		vShell := loftcontact.OutputShell{Vertices: []loftcontact.OutputVertex{
			wallVertex(0, 0, 0), wallVertex(0, 10, 0), wallVertex(0, 10, 10)}}
		roles, _, found, err := loftcontact.SourceRoles(face, [3]int{0, 1, 2}, vShell, vLower, vUpper, z0, z1, budget())
		require.NoError(t, err)
		require.True(t, found)
		require.Equal(t, [2]string{"side(0,0,0)"}, roles)
	})
	t.Run("vertices outside the cell", func(t *testing.T) {
		t.Parallel()
		face := loftcontact.OutputFace{Kind: 1, Owner: 1, Segment: 0}
		for name, outside := range map[string]loftcontact.OutputVertex{
			"above the top level": wallVertex(5, 0, 20),
			"below the bottom":    wallVertex(5, 0, -5),
			"past the wall end":   wallVertex(20, 0, 5),
			"before the wall":     wallVertex(-5, 0, 5),
		} {
			out := shell
			out.Vertices = append([]loftcontact.OutputVertex(nil), shell.Vertices...)
			out.Vertices[2] = outside
			_, _, found, err := loftcontact.SourceRoles(face, [3]int{0, 1, 2}, out, lower, upper, z0, z1, budget())
			require.NoError(t, err, name)
			require.False(t, found, name)
		}
	})
	t.Run("reserves its work before allocating", func(t *testing.T) {
		t.Parallel()
		steps := 0
		b := &proofbound.WorkBudget{StepFn: func() error { steps++; return nil }}
		_, _, found, err := loftcontact.SourceRoles(loftcontact.OutputFace{Kind: 1, Owner: 1}, [3]int{0, 1, 2},
			shell, lower, upper, z0, z1, b)
		require.NoError(t, err)
		require.True(t, found)
		require.Equal(t, 9+3*46, steps, "nine setup operations and 46 per vertex")
	})
	t.Run("budget", func(t *testing.T) {
		t.Parallel()
		boom := context.Canceled
		b := &proofbound.WorkBudget{StepFn: func() error { return boom }, ErrFn: func() error { return boom }}
		_, _, found, err := loftcontact.SourceRoles(loftcontact.OutputFace{Kind: 1, Owner: 1}, [3]int{0, 1, 2},
			shell, lower, upper, z0, z1, b)
		require.ErrorIs(t, err, boom)
		require.False(t, found)
	})
}
