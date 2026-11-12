package parts_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/parts"
	"github.com/stretchr/testify/require"
)

// BenchmarkDerivedExplodeBoltRing derives the removal of twelve identical bolts
// from a plate: one large mesh with many holes and many small repeated meshes.
func BenchmarkDerivedExplodeBoltRing(b *testing.B) {
	s := boltRingSet(b, 12)
	b.ReportAllocs()
	for b.Loop() {
		_, err := s.Explode(b.Context(), parts.WithDerivedExplode(), parts.WithExplodeGround(namePlate))
		require.NoError(b, err)
	}
}
