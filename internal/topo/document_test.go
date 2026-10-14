package topo_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestDocumentNodeMintsAndReadsLiveness drives the document node's own
// methods: MintLevel and MintCurve advance their counter by one per call and
// never repeat an identity; IsLive and Live read the live set by pointer;
// Live returns a copy, and nil for an empty set, including one emptied after
// use.
func TestDocumentNodeMintsAndReadsLiveness(t *testing.T) {
	t.Parallel()
	var d topo.Document
	require.Nil(t, d.Live())

	origin, normal := r3.Vec{X: 1, Y: 2, Z: 3}, r3.Vec{X: 0, Y: 0, Z: 1}
	l1 := d.MintLevel(origin, normal)
	l2 := d.MintLevel(origin, normal)
	require.Equal(t, denotation.LevelID(1), l1.ID)
	require.Equal(t, denotation.LevelID(2), l2.ID)
	require.Equal(t, denotation.LevelID(2), d.NextLevel)
	require.Equal(t, origin, l1.Origin)
	require.Equal(t, normal, l1.Normal)
	require.False(t, denotation.SameLevel(l1, l2), "two mints never denote one level")

	c1 := d.MintCurve()
	c2 := d.MintCurve()
	require.Equal(t, denotation.CurveID(1), c1.ID)
	require.Equal(t, denotation.CurveID(2), c2.ID)
	require.Equal(t, denotation.CurveID(2), d.NextCurve)
	require.False(t, denotation.SameCurve(c1, c2), "two mints never denote one curve")

	a, b := new(topo.Body), new(topo.Body)
	require.False(t, d.IsLive(a))
	d.LiveSet = []*topo.Body{a, b}
	require.True(t, d.IsLive(a))
	require.True(t, d.IsLive(b))
	require.False(t, d.IsLive(new(topo.Body)), "liveness is by pointer, not by value")
	live := d.Live()
	require.Len(t, live, 2)
	require.Same(t, a, live[0])
	require.Same(t, b, live[1])
	live[0] = nil
	require.Same(t, a, d.LiveSet[0], "Live returns a copy")
	d.LiveSet = d.LiveSet[:0]
	require.Nil(t, d.Live(), "an emptied live set reads as nil")
}
