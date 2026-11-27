package meshbool

// CI: isolated-tests

import (
	"context"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/stretchr/testify/require"
)

// weldAwayTetra returns the four faces of a tetrahedron on vertices base..base+3.
func weldAwayTetra(base int) [][3]int {
	a, b, c, d := base, base+1, base+2, base+3
	return [][3]int{{a, c, b}, {a, b, d}, {b, c, d}, {c, a, d}}
}

func TestRefuseWeldedAwayComponentMatchesReferenceOnRandomMeshes(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(7, 11))
	ctx := context.Background()
	for range 400 {
		var tris [][3]int
		for k := range rng.IntN(4) {
			tris = append(tris, weldAwayTetra(4*k)...)
		}
		// Add loose and non-manifold facets, including repeated edges.
		for range rng.IntN(6) {
			tris = append(tris, [3]int{rng.IntN(16), rng.IntN(16), rng.IntN(16)})
		}
		dropped := make([]bool, len(tris))
		p := []float64{0, 0, 0.1, 0.5, 0.9, 1}[rng.IntN(6)]
		for i := range dropped {
			dropped[i] = rng.Float64() < p
		}
		want := refuseWeldedAwayComponentReference(ctx, tris, dropped)
		got := RefuseWeldedAwayComponent(ctx, tris, dropped)
		require.Equal(t, want == nil, got == nil, "tris=%v dropped=%v", tris, dropped)
		if got != nil {
			require.ErrorIs(t, got, decaderr.ErrUnsupported)
		}
	}
}

func TestRefuseWeldedAwayComponentRefusesWholeDroppedComponent(t *testing.T) {
	t.Parallel()
	tris := append(weldAwayTetra(0), weldAwayTetra(4)...)
	dropped := []bool{false, false, false, false, true, true, true, true}
	err := RefuseWeldedAwayComponent(context.Background(), tris, dropped)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "welds away")
}

func TestRefuseWeldedAwayComponentKeepsPartlyDroppedComponent(t *testing.T) {
	t.Parallel()
	tris := append(weldAwayTetra(0), weldAwayTetra(4)...)
	dropped := []bool{true, true, true, false, false, false, true, false}
	require.NoError(t, RefuseWeldedAwayComponent(context.Background(), tris, dropped))
}

func TestRefuseWeldedAwayComponentAcceptsNothingDroppedOnNonManifoldMesh(t *testing.T) {
	t.Parallel()
	// Three facets share the directed edge 0->1; the mesh is not a manifold.
	tris := [][3]int{{0, 1, 2}, {0, 1, 3}, {0, 1, 4}, {1, 0, 5}}
	require.NoError(t, RefuseWeldedAwayComponent(context.Background(), tris, make([]bool, len(tris))))
	require.NoError(t, RefuseWeldedAwayComponent(context.Background(), nil, nil))
}

func TestRefuseWeldedAwayComponentReportsCancelledContext(t *testing.T) {
	t.Parallel()
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	tris := weldAwayTetra(0)
	require.ErrorIs(t, RefuseWeldedAwayComponent(ctx, tris, make([]bool, len(tris))), context.Canceled)
}
