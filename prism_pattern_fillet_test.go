package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestPrismPatternFilletOfCopiedExtrudes(t *testing.T) {
	doc, blank, section, rotated := prismPatternTabBodies(t, math.Nextafter(3, math.Inf(1)))
	joined, err := Union(t.Context(), blank, section, rotated)
	require.NoError(t, err)
	before, err := joined.Volume()
	require.NoError(t, err)
	selection := prismPatternRootSelection(t, joined)
	rounded, err := joined.Fillet(t.Context(), selection, units.Millimeters(0.05))
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1)
	after, err := rounded.Volume()
	require.NoError(t, err)
	require.Greater(t, after.Value.Base(), before.Value.Base())
	mesh, err := rounded.Tessellate(t.Context(), units.Millimeters(0.1), WithVerification(VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Less(t, mesh.Bound().Base(), 0.1)
}

func TestPrismPatternFilletConnectorDiskClearance(t *testing.T) {
	point := func(x, y *big.Rat) prismPatternIVPoint { return prismPatternIVPointOf(x, y) }
	root := point(big.NewRat(3, 1), big.NewRat(0, 1))
	otherRoot := point(big.NewRat(149, 50), big.NewRat(33, 100))
	far := point(big.NewRat(61, 20), big.NewRat(0, 1))
	// The two roots clear their 0.15 mm disks, and the other connector
	// moves outward across the 3 mm blank, but it enters the first disk.
	require.Greater(t, math.Hypot(0.02, 0.33), 0.3)
	require.Less(t, math.Hypot(2.98, 0.33), 3.0)
	require.Greater(t, math.Hypot(3.05, 0), 3.0)
	require.False(t, prismPatternSegmentClearsDisk(root, otherRoot, far, 0.15))
	require.True(t, prismPatternSegmentClearsDisk(root, otherRoot,
		point(big.NewRat(16, 5), big.NewRat(1, 100)), 0.15))
	// A tangency is not enough clearance for a later proof.
	require.False(t, prismPatternSegmentClearsDisk(
		point(big.NewRat(0, 1), big.NewRat(0, 1)),
		point(big.NewRat(-1, 1), big.NewRat(1, 1)),
		point(big.NewRat(1, 1), big.NewRat(1, 1)), 1))
}

func TestPrismPatternFilletRefusesUnprovedCornerAndLargeRadius(t *testing.T) {
	doc, blank, section, rotated := prismPatternTabBodies(t, math.Nextafter(3, math.Inf(1)))
	joined, err := Union(t.Context(), blank, section, rotated)
	require.NoError(t, err)
	axial, err := Edges(ParallelTo(r3.Vec{Z: 1})).SelectEdges(joined)
	require.NoError(t, err)
	var exterior *Edge
	for _, edge := range axial {
		p := edge.Start().Position().Value
		if math.Hypot(p.X, p.Y) > 3.3 {
			exterior = edge
			break
		}
	}
	require.NotNil(t, exterior)
	_, err = joined.Fillet(t.Context(), Edges(ParallelTo(r3.Vec{Z: 1}),
		EndpointAt(exterior.Start().Position().Value)), units.Millimeters(0.05))
	require.ErrorIs(t, err, ErrUnsupported)
	require.Len(t, doc.Bodies(), 1)
	_, err = joined.Fillet(t.Context(), prismPatternRootSelection(t, joined), units.Millimeters(0.2))
	require.ErrorIs(t, err, ErrUnsupported)
	require.Len(t, doc.Bodies(), 1)
}

func prismPatternRootSelection(t *testing.T, joined *Body) *EdgeQuery {
	t.Helper()
	axial, err := Edges(ParallelTo(r3.Vec{Z: 1})).SelectEdges(joined)
	require.NoError(t, err)
	var roots []*Edge
	for _, edge := range axial {
		p := edge.Start().Position().Value
		if math.Abs(math.Hypot(p.X, p.Y)-3) < 1e-8 {
			roots = append(roots, edge)
		}
	}
	require.Len(t, roots, 4)
	selection := Edges(ParallelTo(r3.Vec{Z: 1}), EndpointAt(roots[0].Start().Position().Value))
	for _, edge := range roots[1:] {
		selection.Or(ParallelTo(r3.Vec{Z: 1}), EndpointAt(edge.Start().Position().Value))
	}
	return selection.Exactly(len(roots))
}
