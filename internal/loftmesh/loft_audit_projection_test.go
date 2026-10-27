package loftmesh

import (
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestLoftFloatProjectionProofsMatchExactReference(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(103, 107))
	fixtures := []vertexFixture{
		prism(7, 2, 3), uvSphere(4, 7),
		flatFan("fan", uniformAngles(12), []float64{1, 2}, 0, true),
		randomSoup(rng, "soup", 12, 24, 1, 0),
		randomSoup(rng, "nearly-flat", 12, 24, 1, 1e-15),
		randomSoup(rng, "tiny", 12, 24, 1e-9, 0),
	}
	var counts [4]int
	for _, f := range fixtures {
		data := NewLoftAuditData(f.verts, f.tris)
		for i := range f.tris {
			for j := i + 1; j < len(f.tris); j++ {
				verdict := loftFloatPairProof(f.verts, f.tris, i, j, loftGenericShortcuts)
				counts[verdict]++
				if verdict == loftFloatUnknown {
					continue
				}
				_, err := AuditLoftPairData(data, f.tris, i, j, LoftAuditShortcuts{})
				require.NoError(t, err, "%s pair %d,%d proof %d", f.name, i, j, verdict)
			}
		}
	}
	for verdict, count := range counts {
		require.Positive(t, count, "proof %d was never exercised", verdict)
	}
}

func TestLoftFloatProjectionProofsKeepUnexpectedContacts(t *testing.T) {
	t.Parallel()
	for name, f := range map[string]vertexFixture{
		"coplanar area across shared edge": {
			verts: []r3.Vec{{}, {X: 2}, {Y: 2}, {X: 0.5, Y: 0.5}},
			tris:  [][3]int{{0, 1, 2}, {0, 1, 3}},
		},
		"crossing beyond shared vertex": {
			verts: []r3.Vec{{}, {X: 2}, {Y: 2}, {X: 2, Y: 2, Z: 1}, {X: 2, Y: 2, Z: -1}},
			tris:  [][3]int{{0, 1, 2}, {0, 3, 4}},
		},
		"unshared point": {
			verts: []r3.Vec{{}, {X: 2}, {Y: 2}, {}, {X: -1}, {Y: -1}},
			tris:  [][3]int{{0, 1, 2}, {3, 4, 5}},
		},
		"unshared segment": {
			verts: []r3.Vec{{}, {X: 2}, {Y: 2}, {X: 0.5, Y: -0.5, Z: -1},
				{X: 0.5, Y: 1.5, Z: 1}, {X: 0.5, Y: 1.5, Z: -1}},
			tris: [][3]int{{0, 1, 2}, {3, 4, 5}},
		},
		"duplicate triangle": {
			verts: []r3.Vec{{}, {X: 2}, {Y: 2}},
			tris:  [][3]int{{0, 1, 2}, {0, 1, 2}},
		},
	} {
		t.Run(name, func(t *testing.T) {
			require.Equal(t, loftFloatUnknown, loftFloatPairProof(f.verts, f.tris, 0, 1, loftGenericShortcuts))
			_, err := AuditLoftPairData(NewLoftAuditData(f.verts, f.tris), f.tris, 0, 1, LoftAuditShortcuts{})
			require.Error(t, err)
		})
	}
}

func TestLoftFloatProjectionProofsRespectExactInput(t *testing.T) {
	t.Parallel()
	f := prism(5, 1, 2)
	exact := loftGenericShortcuts
	exact.ExactInput = true
	for i := range f.tris {
		for j := i + 1; j < len(f.tris); j++ {
			require.Equal(t, loftFloatUnknown, loftFloatPairProof(f.verts, f.tris, i, j, exact))
			require.Equal(t, loftFloatUnknown, loftFloatPairProof(f.verts, f.tris, i, j, LoftAuditShortcuts{}))
		}
	}
}
