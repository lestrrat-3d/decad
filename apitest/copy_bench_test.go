package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// BenchmarkCopyRadialLug measures one rotated Copy of the trimmed radial lug
// that TestUnionHubAndManyRadialLugs patterns around its hub: the faceted
// payload's placement (facetproof.PlaceMesh) and the rebuild of the placed
// body (facetedrecord.Build). The angles cycle through the test's 39 turns.
func BenchmarkCopyRadialLug(b *testing.B) {
	doc, hub, lug, _, _ := taperedHubAndRadialLugWidth(b, 9, 7.5, 4.3, 0.1, false)
	require.Contains(b, doc.Bodies(), hub)
	const count = 40
	turns := make([]r3.Transform, 0, count-1)
	for i := 1; i < count; i++ {
		turn, err := r3.Rotation(r3.Vec{X: 1}, units.Radians(2*math.Pi*float64(i)/count))
		require.NoError(b, err)
		turns = append(turns, turn)
	}
	i := 0
	for b.Loop() {
		copied, err := decad.Copy(b.Context(), lug, decad.WithCopyPlacement(turns[i%len(turns)]))
		if err != nil {
			b.Fatal(err)
		}
		if copied == nil {
			b.Fatal("Copy returned no body")
		}
		i++
	}
}
