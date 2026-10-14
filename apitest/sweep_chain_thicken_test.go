package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func cardinalChainThickenSource(t *testing.T, width, radius, height float64) (*sketch.Sketch, *sketch.Chain, *decad.Path) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(0, 0)
	b := s.CreatePoint(width, 0)
	s.Fix(a)
	s.Fix(b)
	s.CreateLine(a, b)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Chains(), 1)
	path, err := decad.NewPath(r3.Vec{},
		decad.LineTo{End: r3.NewVec(0, 0, height)},
		decad.ArcThrough{Through: r3.NewVec(radius, 0, height+radius),
			End: r3.NewVec(2*radius, 0, height)})
	require.NoError(t, err)
	return s, s.Chains()[0], path
}

func TestSweepChainCardinalThickenSolid(t *testing.T) {
	for _, tc := range []struct {
		name                  string
		width, radius, height float64
		side                  decad.ThickenSide
		minY, maxY            float64
	}{
		{"right wall", 4, 5, 20, decad.ThickenPositive, -1, 0},
		{"negative", 4, 5, 20, decad.ThickenNegative, 0, 1},
		{"two-sided", 2, 3, 10, decad.ThickenCentered, -0.5, 0.5},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			s, chain, path := cardinalChainThickenSource(t, tc.width, tc.radius, tc.height)
			doc := decad.New()
			sheet, err := decad.Sweep(t.Context(), doc, s, chain, path)
			require.NoError(t, err)
			solid, err := decad.Thicken(t.Context(), sheet, units.Millimeters(1), decad.WithThickenSide(tc.side))
			require.NoError(t, err)
			require.Equal(t, decad.BodySolid, solid.Kind())
			require.True(t, solid.IsSolid())
			require.Len(t, solid.Faces(), 10)
			_, err = decad.Edges(decad.Free()).SelectEdges(solid)
			require.ErrorIs(t, err, decad.ErrNoMatch)
			require.Len(t, doc.Bodies(), 1)
			require.Same(t, solid, doc.Bodies()[0])
			_, err = decad.Thicken(t.Context(), sheet, units.Millimeters(1))
			require.ErrorIs(t, err, decad.ErrRetiredBody)

			wantVolume := tc.width * (tc.height + math.Pi*(tc.radius-tc.width/2))
			volume, err := solid.Volume()
			require.NoError(t, err)
			require.LessOrEqual(t, math.Abs(volume.Value.Base()-wantVolume), volume.Bound.Base()+1e-12)
			wantArea := 2*tc.width + 2*tc.height*(tc.width+1) +
				math.Pi*((2*tc.radius-tc.width)+(2*tc.radius*tc.width-tc.width*tc.width))
			area, err := solid.Area()
			require.NoError(t, err)
			require.LessOrEqual(t, math.Abs(area.Value.Base()-wantArea), area.Bound.Base()+1e-12)
			box, err := solid.Bounds()
			require.NoError(t, err)
			require.Equal(t, r3.NewVec(0, tc.minY, 0), box.Min)
			require.Equal(t, r3.NewVec(2*tc.radius, tc.maxY, tc.height+tc.radius), box.Max)
			mesh, err := solid.Tessellate(t.Context(), units.Millimeters(0.1),
				decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
			require.LessOrEqual(t, mesh.Bound().Base(), 0.1)
			mass, err := solid.MassProperties(t.Context(), units.KilogramsPerCubicMillimeter(0.001))
			require.NoError(t, err)
			require.LessOrEqual(t, math.Abs(mass.Mass.Value.Base()-wantVolume*0.001),
				mass.Mass.Bound.Base()+1e-12)
		})
	}
}

func TestSweepChainThickenConservativeRefusals(t *testing.T) {
	t.Parallel()
	s, chain, path := cardinalChainThickenSource(t, 4, 5, 20)
	doc := decad.New()
	sheet, err := decad.Sweep(t.Context(), doc, s, chain, path)
	require.NoError(t, err)
	_, err = decad.Thicken(t.Context(), sheet, units.Millimeters(0))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.Thicken(t.Context(), sheet, units.Inches(0.1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Len(t, doc.Bodies(), 1)
	require.Same(t, sheet, doc.Bodies()[0])

	oneSpan, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, 20)})
	require.NoError(t, err)
	otherDoc := decad.New()
	oneSheet, err := decad.Sweep(t.Context(), otherDoc, s, chain, oneSpan)
	require.NoError(t, err)
	_, err = decad.Thicken(t.Context(), oneSheet, units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Len(t, otherDoc.Bodies(), 1)
}
