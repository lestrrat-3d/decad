package prismpattern_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/prismpattern"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// The joined-prism fillet reads only authenticated axial roots. Every gate
// here fires before any root is matched; the matching itself is exercised by
// the root package's fillet tests on a real joined prism.
func TestFilletContactsGates(t *testing.T) {
	t.Parallel()

	joined := facetedrecord.Payload{PrismPatternJoin: &facetedrecord.PrismPatternJoinRecord{}}
	axial := func(dir r3.Vec, proven bool) *topo.Edge {
		return &topo.Edge{SelectorDirection: dir, SelectorDirectionProven: proven,
			Start: &topo.Vertex{}, End: &topo.Vertex{Position: dir}}
	}
	tests := []struct {
		name   string
		fp     facetedrecord.Payload
		edges  []*topo.Edge
		radius float64
		msg    string
	}{
		{"no joined source", facetedrecord.Payload{}, nil, 0.1, "no certified joined-prism source or radius"},
		{"zero radius", joined, nil, 0, "no certified joined-prism source or radius"},
		{"negative radius", joined, nil, -1, "no certified joined-prism source or radius"},
		{"not finite radius", joined, nil, math.Inf(1), "no certified joined-prism source or radius"},
		{"nil edge", joined, []*topo.Edge{nil}, 0.1, "unproved axial edge"},
		{"unproven direction", joined, []*topo.Edge{axial(r3.Vec{Z: 1}, false)}, 0.1, "unproved axial edge"},
		{"not axial", joined, []*topo.Edge{axial(r3.Vec{X: 1}, true)}, 0.1, "unproved axial edge"},
		{"axial up matches no root", joined, []*topo.Edge{axial(r3.Vec{Z: 1}, true)}, 0.1, "selects no authenticated root"},
		{"axial down matches no root", joined, []*topo.Edge{axial(r3.Vec{Z: -1}, true)}, 0.1, "selects no authenticated root"},
	}
	for _, tc := range tests {
		_, err := prismpattern.FilletContacts(t.Context(), tc.fp, tc.edges, tc.radius)
		require.ErrorIs(t, err, decaderr.ErrUnsupported, tc.name)
		require.ErrorContains(t, err, tc.msg, tc.name)
	}

	cancelled, cancel := context.WithCancel(t.Context())
	cancel()
	_, err := prismpattern.FilletContacts(cancelled, joined, []*topo.Edge{axial(r3.Vec{Z: 1}, true)}, 0.1)
	require.ErrorIs(t, err, cancelled.Err())
}
