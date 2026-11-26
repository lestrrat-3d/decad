package facetedrecord_test

// CI: isolated-tests

import (
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/occupancy"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestCapSideRecordsReadOnlyOnTheirOwnMesh(t *testing.T) {
	verts := []r3.Vec{{}, {X: 1}, {Y: 1}, {Z: 1}}
	tris := [][3]int{{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {0, 3, 2}}
	zero := big.NewRat(0, 1)
	records := []facetedrecord.CapSideRecord{{Group: 0, O: [3]*big.Rat{zero, zero, zero},
		N: [3]*big.Rat{zero, zero, big.NewRat(-1, 1)}}}
	p := facetedrecord.Payload{Verts: verts, Tris: tris, CapSides: records, CapSideMesh: occupancy.Bind(verts, tris)}
	require.Len(t, p.CapSideRecords(), 1)

	// A copy that keeps the record but holds new arrays, as a placement or a rewrite does, reads none.
	moved := p
	moved.Verts = slices.Clone(verts)
	require.Nil(t, moved.CapSideRecords())
	rewritten := p
	rewritten.Tris = slices.Clone(tris)
	require.Nil(t, rewritten.CapSideRecords())
	unbound := p
	unbound.CapSideMesh = occupancy.Binding{}
	require.Nil(t, unbound.CapSideRecords())
}
