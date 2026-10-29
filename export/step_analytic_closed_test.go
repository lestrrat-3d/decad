package export_test

import (
	"bytes"
	"context"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
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
	moved, err := r3.Translation(r3.NewVec(1, 0, 0))
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), body, moved)
	require.NoError(t, err)
	fallback, err := export.NewSTEPFile(t.Context(), placed, units.Millimeters(0.1), header())
	require.NoError(t, err)
	for _, entity := range fallback.Entities {
		require.NotEqual(t, "SPHERICAL_SURFACE", entity.Name)
	}
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
}
