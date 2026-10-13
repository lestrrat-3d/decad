package shellseparation_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/shellseparation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func shellAt(origin r3.Vec) shellseparation.Shell {
	return shellseparation.Shell{
		Verts: []r3.Vec{origin, origin.Add(r3.Vec{X: 1}),
			origin.Add(r3.Vec{Y: 1}), origin.Add(r3.Vec{Z: 1})},
		Tris:        [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}},
		VertexBound: make([]float64, 4),
	}
}

func placedShell(source shellseparation.Shell, move r3.Transform) shellseparation.Shell {
	image := source
	image.Verts = make([]r3.Vec, len(source.Verts))
	for i, vertex := range source.Verts {
		image.Verts[i] = move.Apply(vertex)
	}
	return image
}

func TestCartesianAxesSeparateRecordedCopies(t *testing.T) {
	for _, tc := range []struct {
		name      string
		axis      shellseparation.Axis
		worldAxis r3.Vec
		offset    r3.Vec
	}{
		{"x", shellseparation.AxisX, r3.Vec{X: 1}, r3.Vec{Y: 10}},
		{"y", shellseparation.AxisY, r3.Vec{Y: 1}, r3.Vec{Z: 10}},
		{"z", shellseparation.AxisZ, r3.Vec{Z: 1}, r3.Vec{X: 10}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			turn, err := r3.Rotation(tc.worldAxis, units.Degrees(180))
			require.NoError(t, err)
			source := shellAt(tc.offset)
			image := placedShell(source, turn)
			require.True(t, shellseparation.Separate(t.Context(),
				[]shellseparation.Shell{source, image},
				[]r3.Transform{r3.Identity(), turn}, shellseparation.SourceShell, tc.axis, true))
			require.False(t, shellseparation.Separate(t.Context(),
				[]shellseparation.Shell{source, source},
				[]r3.Transform{r3.Identity(), r3.Identity()}, shellseparation.SourceShell, tc.axis, false))
		})
	}
}

func TestSeparationHasNoCopyCountCap(t *testing.T) {
	const copies = 32
	for _, tc := range []struct {
		name              string
		axis              shellseparation.Axis
		worldAxis, offset r3.Vec
	}{
		{"x", shellseparation.AxisX, r3.Vec{X: 1}, r3.Vec{Y: 100}},
		{"y", shellseparation.AxisY, r3.Vec{Y: 1}, r3.Vec{Z: 100}},
		{"z", shellseparation.AxisZ, r3.Vec{Z: 1}, r3.Vec{X: 100}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			source := shellAt(tc.offset)
			shells := make([]shellseparation.Shell, copies)
			moves := make([]r3.Transform, copies)
			shells[0], moves[0] = source, r3.Identity()
			for i := 1; i < copies; i++ {
				turn, err := r3.Rotation(tc.worldAxis, units.Degrees(360*float64(i)/copies))
				require.NoError(t, err)
				moves[i] = turn
				shells[i] = placedShell(source, turn)
			}
			require.True(t, shellseparation.Separate(t.Context(), shells, moves,
				shellseparation.SourceShell, tc.axis, true))
		})
	}
}

func TestLongitudinallyTranslatedCopiesSeparateWithoutAngularGap(t *testing.T) {
	move, err := r3.Translation(r3.Vec{X: 10})
	require.NoError(t, err)
	source := shellAt(r3.Vec{})
	image := placedShell(source, move)
	require.True(t, shellseparation.Separate(t.Context(),
		[]shellseparation.Shell{source, image},
		[]r3.Transform{r3.Identity(), move}, shellseparation.SourceShell,
		shellseparation.AxisX, false))
}

func TestSeparationRefusesMalformedProofEvenWhenProjectedApart(t *testing.T) {
	turn, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(180))
	require.NoError(t, err)
	source := shellAt(r3.Vec{Y: 10})
	image := placedShell(source, turn)
	moves := []r3.Transform{r3.Identity(), turn}

	for name, change := range map[string]func(*shellseparation.Shell){
		"missing vertex bounds": func(p *shellseparation.Shell) { p.VertexBound = nil },
		"negative beta":         func(p *shellseparation.Shell) { p.VertexBound[0] = -1 },
		"nonfinite beta":        func(p *shellseparation.Shell) { p.VertexBound[0] = math.NaN() },
		"bound below beta":      func(p *shellseparation.Shell) { p.VertexBound[0] = 0.1 },
		"nonfinite vertex":      func(p *shellseparation.Shell) { p.Verts[0].Y = math.Inf(1) },
		"invalid facet index":   func(p *shellseparation.Shell) { p.Tris[0][0] = len(p.Verts) },
	} {
		t.Run(name, func(t *testing.T) {
			bad := image
			bad.VertexBound = append([]float64(nil), image.VertexBound...)
			bad.Verts = append([]r3.Vec(nil), image.Verts...)
			bad.Tris = append([][3]int(nil), image.Tris...)
			change(&bad)
			require.False(t, shellseparation.Separate(t.Context(),
				[]shellseparation.Shell{source, bad}, moves, shellseparation.SourceShell,
				shellseparation.AxisX, true))
		})
	}
}

func TestPlacedHeldCopiesRefuseMalformedMesh(t *testing.T) {
	turn, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(180))
	require.NoError(t, err)
	source := shellAt(r3.Vec{Y: 10})
	image := placedShell(source, turn)
	moves := []r3.Transform{r3.Identity(), turn}
	copyOf := func(shell shellseparation.Shell) shellseparation.HeldCopy {
		return shellseparation.HeldCopy{
			Verts: append([]r3.Vec(nil), shell.Verts...),
			Tris:  append([][3]int(nil), shell.Tris...), Linked: true,
		}
	}
	for _, tc := range []struct {
		name   string
		change func([]shellseparation.HeldCopy)
	}{
		{"invalid source facet", func(c []shellseparation.HeldCopy) { c[0].Tris[0][0] = 99 }},
		{"nonfinite source vertex", func(c []shellseparation.HeldCopy) { c[0].Verts[0].X = math.NaN() }},
		{"nonfinite placed vertex", func(c []shellseparation.HeldCopy) { c[1].Verts[0].Y = math.Inf(1) }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			copies := []shellseparation.HeldCopy{copyOf(source), copyOf(image)}
			tc.change(copies)
			_, err := shellseparation.PlacedHeldShells(t.Context(), copies, moves)
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
		})
	}
}

func TestPlacedHeldCopiesChargeVertexDisplacement(t *testing.T) {
	move, err := r3.Translation(r3.Vec{X: 1})
	require.NoError(t, err)
	source := shellAt(r3.Vec{})
	image := placedShell(source, move)
	image.Verts[3].Z += 0.25
	parts, err := shellseparation.PlacedHeldShells(t.Context(), []shellseparation.HeldCopy{
		{Verts: source.Verts, Tris: source.Tris, Linked: true},
		{Verts: image.Verts, Tris: image.Tris, Linked: true},
	}, []r3.Transform{r3.Identity(), move})
	require.NoError(t, err)
	require.Len(t, parts, 2)
	require.Equal(t, make([]float64, 4), parts[0].VertexBound)
	require.GreaterOrEqual(t, parts[1].VertexBound[3], 0.25)
	require.GreaterOrEqual(t, parts[1].MeshBound, 0.25)
	for _, bound := range parts[1].VertexBound[:3] {
		require.Less(t, bound, 1e-12)
	}
}

func TestPlacedHeldCopiesRefuseBrokenCorrespondence(t *testing.T) {
	move, err := r3.Translation(r3.Vec{X: 1})
	require.NoError(t, err)
	source := shellAt(r3.Vec{})
	image := placedShell(source, move)
	first := shellseparation.HeldCopy{Verts: source.Verts, Tris: source.Tris, Linked: true}
	second := shellseparation.HeldCopy{Verts: image.Verts, Tris: image.Tris, Linked: true}
	moves := []r3.Transform{r3.Identity(), move}

	for name, copies := range map[string][]shellseparation.HeldCopy{
		"unlinked source": {{Verts: first.Verts, Tris: first.Tris}, second},
		"unlinked copy":   {first, {Verts: second.Verts, Tris: second.Tris}},
		"reordered facets": {first, {Verts: second.Verts,
			Tris: [][3]int{second.Tris[1], second.Tris[0], second.Tris[2], second.Tris[3]}, Linked: true}},
	} {
		t.Run(name, func(t *testing.T) {
			_, err := shellseparation.PlacedHeldShells(t.Context(), copies, moves)
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
		})
	}
}
