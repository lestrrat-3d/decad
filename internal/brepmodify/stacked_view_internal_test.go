package brepmodify

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
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
