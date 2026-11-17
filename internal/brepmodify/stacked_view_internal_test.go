package brepmodify

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// stackedViewBody is a body of producer 7 with one face per role, in order: a
// planar record face, a swept wall, a planar face of another producer, and a
// face with no origin.
func stackedViewBody() *topo.Body {
	face := func(producer provenance.Producer, role string) *topo.Face {
		f := &topo.Face{}
		if role != "" {
			f.Origins = []provenance.Ref{{Producer: producer, Role: role}}
		}
		return f
	}
	b := &topo.Body{Origin: provenance.Ref{Producer: 7, Role: provenance.RoleBody}}
	b.Lumps = []*topo.Lump{{Shells: []*topo.FaceShell{{Faces: []*topo.Face{
		face(7, "face(0)"), face(7, "wall(1)"), face(8, "face(2)"), face(7, ""),
	}}}}}
	return b
}

// TestStackedViewUntag keeps every tag of an exact record and drops all but
// the planar record faces' once the record carries a section displacement or a
// carrier charge.
func TestStackedViewUntag(t *testing.T) {
	b := stackedViewBody()
	require.Nil(t, stackedViewUntag(b, breprecord.Payload{Faces: []breprecord.Face{{}, {}}}))
	all := map[int]struct{}{1: {}, 2: {}, 3: {}}
	require.Equal(t, all, stackedViewUntag(b, breprecord.Payload{Faces: []breprecord.Face{{Delta: 1e-15}, {}}}))
	require.Equal(t, all, stackedViewUntag(b, breprecord.Payload{CarrierCharge: 1e-15}))
}

// TestViewMissed reads only a view-build refusal on a live context as a miss.
func TestViewMissed(t *testing.T) {
	built := viewBuildError{decaderr.ErrUnsupported}
	require.True(t, viewMissed(t.Context(), built))
	require.True(t, errors.Is(built, decaderr.ErrUnsupported))
	require.False(t, viewMissed(t.Context(), decaderr.ErrUnsupported))
	require.False(t, viewMissed(t.Context(), nil))
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	require.False(t, viewMissed(ctx, built))
}

// TestStackedViewCharge states the record's displacement on each swept circular
// wall of the producer and untags every other face that is not a planar record
// face.
func TestStackedViewCharge(t *testing.T) {
	face := func(producer provenance.Producer, role string, surface surfacegeom.Surface) *topo.Face {
		f := &topo.Face{Surface: surface}
		if role != "" {
			f.Origins = []provenance.Ref{{Producer: producer, Role: role}}
		}
		return f
	}
	cylinder := surfacegeom.Cylinder{Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(3)}
	plane := surfacegeom.Plane{}
	b := &topo.Body{Origin: provenance.Ref{Producer: 7, Role: provenance.RoleBody}}
	b.Lumps = []*topo.Lump{{Shells: []*topo.FaceShell{{Faces: []*topo.Face{
		face(7, "face(0)", plane), face(7, "wall(1)", cylinder), face(7, "wall(2)", plane),
		face(8, "wall(3)", cylinder), face(7, "", cylinder),
	}}}}}

	t.Run("section displacement under the carrier charge", func(t *testing.T) {
		bp := breprecord.Payload{Faces: []breprecord.Face{{Delta: 2e-14}}, CarrierCharge: 1e-14}
		require.Equal(t, map[int]float64{1: 2e-14}, stackedViewCharge(b, bp))
		require.Equal(t, map[int]struct{}{2: {}, 3: {}, 4: {}}, stackedViewUntag(b, bp))
	})
	t.Run("carrier charge over the section displacement", func(t *testing.T) {
		bp := breprecord.Payload{Faces: []breprecord.Face{{Delta: 2e-14}}, CarrierCharge: 3e-14}
		require.Equal(t, map[int]float64{1: 3e-14}, stackedViewCharge(b, bp))
	})
	t.Run("exact record", func(t *testing.T) {
		bp := breprecord.Payload{Faces: []breprecord.Face{{}}}
		require.Nil(t, stackedViewCharge(b, bp))
		require.Nil(t, stackedViewUntag(b, bp))
	})
	t.Run("NaN displacement", func(t *testing.T) {
		bp := breprecord.Payload{Faces: []breprecord.Face{{Delta: math.NaN()}}}
		require.Nil(t, stackedViewCharge(b, bp))
		require.Equal(t, map[int]struct{}{1: {}, 2: {}, 3: {}, 4: {}}, stackedViewUntag(b, bp))
	})
}
