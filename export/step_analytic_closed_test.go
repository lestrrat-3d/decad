package export_test

import (
	"bytes"
	"context"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/decad/internal/closedstep"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/step"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestNewSTEPFileWholeSphereAnalytic(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	s.Fix(o)
	e := s.CreatePoint(10, 0)
	c := s.CreatePoint(5, 0)
	s.CreateLine(o, e)
	s.CreateArc(c, e, o)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	body, err := decad.Revolve(t.Context(), decad.New(), s, s.Profiles()[0], decad.ConstructionAxis{Origin: r3.Vec{}, Dir: r3.NewVec(1, 0, 0)}, decad.FullRevolution{})
	require.NoError(t, err)
	require.Len(t, body.Faces(), 1)
	require.Empty(t, body.Edges())
	require.Empty(t, body.Faces()[0].Loops())
	f, err := export.NewSTEPFile(t.Context(), body, units.Millimeters(0.1), header())
	require.NoError(t, err)
	counts := map[string]int{}
	entities := map[step.Reference]step.Entity{}
	for _, e := range f.Entities {
		counts[e.Name]++
		entities[step.Reference(e.ID)] = e
	}
	require.Equal(t, 1, counts["SPHERICAL_SURFACE"])
	require.Equal(t, 1, counts["VERTEX_LOOP"])
	require.Equal(t, 0, counts["EDGE_CURVE"])
	require.Equal(t, 1, counts["FACE_OUTER_BOUND"])
	for _, entity := range f.Entities {
		if entity.Name != "VERTEX_LOOP" {
			continue
		}
		vertex := entities[entity.Parameters[1].(step.Reference)]
		require.Equal(t, "VERTEX_POINT", vertex.Name)
		point := entities[vertex.Parameters[1].(step.Reference)]
		require.Equal(t, "CARTESIAN_POINT", point.Name)
		require.Equal(t, step.List{step.Real(5), step.Real(0), step.Real(5)}, point.Parameters[1])
	}

	b, err := f.Marshal()
	require.NoError(t, err)
	_, err = decad.ReadSTEP(t.Context(), bytes.NewReader(b))
	require.Error(t, err)
	f2, err := export.NewSTEPFile(t.Context(), body, units.Millimeters(0.1), header())
	require.NoError(t, err)
	b2, err := f2.Marshal()
	require.NoError(t, err)
	require.True(t, bytes.Equal(b, b2))
	require.Len(t, body.Faces(), 1)
	require.Empty(t, body.Edges())
	require.Empty(t, body.Faces()[0].Loops())
	moved, err := r3.Translation(r3.NewVec(8, -4, 2))
	require.NoError(t, err)
	placed, err := decad.Copy(t.Context(), body, decad.WithCopyPlacement(moved))
	require.NoError(t, err)
	coverage, ok, err := closedstep.Read(t.Context(), placed, placed.Faces()[0])
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, r3.NewVec(13, -4, 2), coverage.Center)
	require.Equal(t, 5.0, coverage.Radius)
	translated, err := export.NewSTEPFile(t.Context(), placed, units.Millimeters(0.1), header())
	require.NoError(t, err)
	translatedBytes, err := translated.Marshal()
	require.NoError(t, err)
	require.NotEmpty(t, translatedBytes)
	found := false
	translatedEntities := map[step.Reference]step.Entity{}
	for _, entity := range translated.Entities {
		translatedEntities[step.Reference(entity.ID)] = entity
	}
	for _, entity := range translated.Entities {
		if entity.Name == "VERTEX_LOOP" {
			vertex := translatedEntities[entity.Parameters[1].(step.Reference)]
			point := translatedEntities[vertex.Parameters[1].(step.Reference)]
			require.Equal(t, step.List{step.Real(13), step.Real(-4), step.Real(7)}, point.Parameters[1])
		}
		if entity.Name == "SPHERICAL_SURFACE" {
			found = true
		}
	}
	require.True(t, found)

	for _, tc := range []struct {
		translation r3.Vec
		coverage    bool
		analytic    bool
	}{
		{r3.NewVec(0.1, 0, 0), false, false},
		{r3.NewVec(0, 0, 0.1), true, false},
		{r3.NewVec(256, -128, 64), true, true},
	} {
		move, err := r3.Translation(tc.translation)
		require.NoError(t, err)
		copyBody, err := decad.Copy(t.Context(), body, decad.WithCopyPlacement(move))
		require.NoError(t, err)
		_, ok, err := closedstep.Read(t.Context(), copyBody, copyBody.Faces()[0])
		require.NoError(t, err)
		require.Equal(t, tc.coverage, ok)
		file, err := export.NewSTEPFile(t.Context(), copyBody, units.Millimeters(0.1), header())
		require.NoError(t, err)
		sphere, planes := 0, 0
		for _, entity := range file.Entities {
			if entity.Name == "SPHERICAL_SURFACE" {
				sphere++
			}
			if entity.Name == "PLANE" {
				planes++
			}
		}
		if tc.analytic {
			require.Equal(t, 1, sphere)
		} else {
			require.Zero(t, sphere)
			require.Positive(t, planes)
		}
	}
	transformed, err := decad.Transform(t.Context(), placed, moved)
	require.NoError(t, err)
	again, ok, err := closedstep.Read(t.Context(), transformed, transformed.Faces()[0])
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, r3.NewVec(21, -8, 4), again.Center)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = export.NewSTEPFile(ctx, body, units.Millimeters(0.1), header())
	require.ErrorIs(t, err, context.Canceled)
}

func TestNewSTEPFileWholeRingAnalytic(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 10)
	s.Fix(center)
	s.CreateCircle(center, 3)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Revolve(t.Context(), decad.New(), s, s.Profiles()[0], decad.ConstructionAxis{Origin: r3.Vec{}, Dir: r3.NewVec(1, 0, 0)}, decad.FullRevolution{})
	require.NoError(t, err)
	f, err := export.NewSTEPFile(t.Context(), body, units.Millimeters(0.1), header())
	require.NoError(t, err)
	counts := map[string]int{}
	entities := map[step.Reference]step.Entity{}
	uses := map[step.Reference][]step.Enumeration{}
	for _, e := range f.Entities {
		counts[e.Name]++
		entities[step.Reference(e.ID)] = e
		if e.Name == orientedEdgeEntity {
			id := e.Parameters[3].(step.Reference)
			uses[id] = append(uses[id], e.Parameters[4].(step.Enumeration))
		}
	}
	require.Equal(t, 1, counts["TOROIDAL_SURFACE"])
	require.Equal(t, 1, counts["VERTEX_POINT"])
	require.Equal(t, 2, counts["EDGE_CURVE"])
	require.Len(t, uses, 2)
	for _, entity := range f.Entities {
		if entity.Name != "VERTEX_POINT" {
			continue
		}
		point := entities[entity.Parameters[1].(step.Reference)]
		require.Equal(t, "CARTESIAN_POINT", point.Name)
		require.Equal(t, step.List{step.Real(0), step.Real(13), step.Real(0)}, point.Parameters[1])
	}
	for _, use := range uses {
		require.ElementsMatch(t, []step.Enumeration{"T", "F"}, use)
	}
	b, err := f.Marshal()
	require.NoError(t, err)
	f2, err := export.NewSTEPFile(t.Context(), body, units.Millimeters(0.1), header())
	require.NoError(t, err)
	b2, err := f2.Marshal()
	require.NoError(t, err)
	require.True(t, bytes.Equal(b, b2))
	require.Len(t, body.Faces(), 1)
	require.Empty(t, body.Edges())
	require.Empty(t, body.Faces()[0].Loops())
	move, err := r3.Translation(r3.NewVec(8, -4, 2))
	require.NoError(t, err)
	placed, err := decad.Copy(t.Context(), body, decad.WithCopyPlacement(move))
	require.NoError(t, err)
	coverage, ok, err := closedstep.Read(t.Context(), placed, placed.Faces()[0])
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, r3.NewVec(8, -4, 2), coverage.Center)
	translated, err := export.NewSTEPFile(t.Context(), placed, units.Millimeters(0.1), header())
	require.NoError(t, err)
	output, err := translated.Marshal()
	require.NoError(t, err)
	require.NotEmpty(t, output)
	found := false
	translatedEntities := map[step.Reference]step.Entity{}
	for _, entity := range translated.Entities {
		translatedEntities[step.Reference(entity.ID)] = entity
		if entity.Name == "TOROIDAL_SURFACE" {
			found = true
		}
	}
	require.True(t, found)
	for _, entity := range translated.Entities {
		if entity.Name == "VERTEX_POINT" {
			point := translatedEntities[entity.Parameters[1].(step.Reference)]
			require.Equal(t, step.List{step.Real(8), step.Real(9), step.Real(2)}, point.Parameters[1])
		}
	}
	moveInexact, err := r3.Translation(r3.NewVec(0, 0.1, 0))
	require.NoError(t, err)
	inexact, err := decad.Copy(t.Context(), body, decad.WithCopyPlacement(moveInexact))
	require.NoError(t, err)
	_, ok, err = closedstep.Read(t.Context(), inexact, inexact.Faces()[0])
	require.NoError(t, err)
	require.True(t, ok)
	fallback, err := export.NewSTEPFile(t.Context(), inexact, units.Millimeters(0.1), header())
	require.NoError(t, err)
	planes := 0
	for _, entity := range fallback.Entities {
		require.NotEqual(t, "TOROIDAL_SURFACE", entity.Name)
		if entity.Name == "PLANE" {
			planes++
		}
	}
	require.Positive(t, planes)
}

func TestClosedSTEPInexactSyntheticPointFallsBack(t *testing.T) {
	t.Run("sphere", func(t *testing.T) {
		w := sketch.NewWorld()
		plane, err := w.CreateOffsetPlane(w.XY(), 0.1)
		require.NoError(t, err)
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		start := s.CreatePoint(0, 0)
		s.Fix(start)
		end := s.CreatePoint(0.4, 0)
		center := s.CreatePoint(0.2, 0)
		s.CreateLine(start, end)
		s.CreateArc(center, end, start)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		body, err := decad.Revolve(t.Context(), decad.New(), s, s.Profiles()[0], decad.ConstructionAxis{Origin: r3.NewVec(0, 0, 0.1), Dir: r3.NewVec(1, 0, 0)}, decad.FullRevolution{})
		require.NoError(t, err)
		coverage, ok, err := closedstep.Read(t.Context(), body, body.Faces()[0])
		require.NoError(t, err)
		require.True(t, ok)
		require.False(t, coverage.Ring)
		exact := proof.DyAdd(proof.MustDyOf(coverage.Center.Z), proof.MustDyOf(coverage.Radius))
		held := coverage.Center.Z + coverage.Radius
		require.NotZero(t, proof.DyCmp(exact, proof.MustDyOf(held)))
		f, err := export.NewSTEPFile(t.Context(), body, units.Millimeters(0.02), header())
		require.NoError(t, err)
		planes := 0
		for _, e := range f.Entities {
			require.NotEqual(t, "SPHERICAL_SURFACE", e.Name)
			if e.Name == "PLANE" {
				planes++
			}
		}
		require.Positive(t, planes)
	})
	t.Run("ring", func(t *testing.T) {
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		center := s.CreatePoint(0, 0.4)
		s.Fix(center)
		s.CreateCircle(center, 0.1)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		body, err := decad.Revolve(t.Context(), decad.New(), s, s.Profiles()[0], decad.ConstructionAxis{Origin: r3.Vec{}, Dir: r3.NewVec(1, 0, 0)}, decad.FullRevolution{})
		require.NoError(t, err)
		coverage, ok, err := closedstep.Read(t.Context(), body, body.Faces()[0])
		require.NoError(t, err)
		require.True(t, ok)
		require.True(t, coverage.Ring)
		exact := proof.DyAdd(proof.MustDyOf(coverage.Major), proof.MustDyOf(coverage.Minor))
		held := coverage.Major + coverage.Minor
		require.NotZero(t, proof.DyCmp(exact, proof.MustDyOf(held)))
		f, err := export.NewSTEPFile(t.Context(), body, units.Millimeters(0.02), header())
		require.NoError(t, err)
		planes := 0
		for _, e := range f.Entities {
			require.NotEqual(t, "TOROIDAL_SURFACE", e.Name)
			if e.Name == "PLANE" {
				planes++
			}
		}
		require.Positive(t, planes)
	})
}
