package draftrecord_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

const producer = provenance.Producer(7)

func face(roles ...string) *topo.Face {
	f := &topo.Face{}
	for _, r := range roles {
		f.Origins = append(f.Origins, provenance.Ref{Producer: producer, Role: r})
	}
	return f
}

func bodyOf(faces ...*topo.Face) *topo.Body {
	b := &topo.Body{Lumps: []*topo.Lump{{Shells: []*topo.FaceShell{{Faces: faces}}}}}
	b.Origin.Producer = producer
	for _, f := range faces {
		f.Body = b
	}
	return b
}

func TestReceiverGates(t *testing.T) {
	t.Parallel()
	_, err := draftrecord.Receiver(&topo.Body{Payload: draftrecord.Payload{}})
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "already a draft body")
	_, err = draftrecord.Receiver(&topo.Body{})
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "straight prism only")
	pp, err := draftrecord.Receiver(&topo.Body{Payload: prismrecord.Payload{Z1: 3}})
	require.NoError(t, err)
	require.Equal(t, 3.0, pp.Z1)
}

func TestNeutralCapGates(t *testing.T) {
	t.Parallel()
	start, end, wall := face("capStart"), face("capEnd"), face("side(0,1)")
	b := bodyOf(start, end, wall)

	near, err := draftrecord.NeutralCap(start, b)
	require.NoError(t, err)
	require.True(t, near)
	near, err = draftrecord.NeutralCap(end, b)
	require.NoError(t, err)
	require.False(t, near)

	_, err = draftrecord.NeutralCap(wall, b)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "capStart or capEnd")
	_, err = draftrecord.NeutralCap(start, bodyOf())
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "belongs to another body")
}

func TestKeptWalls(t *testing.T) {
	t.Parallel()
	start, end := face("capStart"), face("capEnd")
	w01, w02 := face("side(0,1)"), face("side(0,2)")
	// A wall of another producer and a malformed role never count as kept.
	other := &topo.Face{Origins: []provenance.Ref{{Producer: producer + 1, Role: "side(3,4)"}}}
	padded := face("side(5, 6)")
	mixed := &topo.Face{Origins: []provenance.Ref{
		{Producer: producer + 1, Role: "side(8,9)"}, {Producer: producer, Role: "side(1,2)"}}}
	b := bodyOf(start, end, w01, w02, other, padded, mixed)

	kept, err := draftrecord.KeptWalls(b, producer, []*topo.Face{w01})
	require.NoError(t, err)
	require.Equal(t, map[offset2d.DraftWall]struct{}{{Loop: 0, Seg: 2}: {}, {Loop: 1, Seg: 2}: {}}, kept)

	for _, tc := range []struct {
		name     string
		selected []*topo.Face
		err      error
		msg      string
	}{
		{"start cap", []*topo.Face{start}, decaderr.ErrDegenerate, "contains a cap"},
		{"end cap", []*topo.Face{w01, end}, decaderr.ErrDegenerate, "contains a cap"},
		{"not a wall", []*topo.Face{face()}, decaderr.ErrUnsupported, "no wall of the receiver"},
	} {
		_, err := draftrecord.KeptWalls(b, producer, tc.selected)
		require.ErrorIs(t, err, tc.err, tc.name)
		require.ErrorContains(t, err, tc.msg, tc.name)
	}
}
