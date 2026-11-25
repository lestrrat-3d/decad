package facetproof_test

// CI: isolated-tests

import (
	"context"
	"math"
	"sync"
	"testing"

	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/gaptol"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func certMotion(t *testing.T, axis r3.Vec, degrees float64, tr r3.Vec, mirror bool) r3.Transform {
	t.Helper()
	rot, err := r3.Rotation(axis, units.Degrees(degrees))
	require.NoError(t, err)
	move, err := r3.Translation(tr)
	require.NoError(t, err)
	out, err := rot.Then(move)
	require.NoError(t, err)
	if !mirror {
		return out
	}
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	flip, err := r3.Reflection(frame)
	require.NoError(t, err)
	out, err = flip.Then(out)
	require.NoError(t, err)
	return out
}

func placeCertified(ctx context.Context, verts []r3.Vec, tris [][3]int, d r3.Transform) (facetproof.PlacedMesh, error) {
	return facetproof.PlaceMesh(ctx, proofbound.NewWorkBudget(ctx), verts, tris, make([]float64, len(verts)), 0, 0, 0, 0, d)
}

// With a certificate cache, the third and later placements of a source skip the
// held scan and return exactly what a placement without the cache returns, a
// reflected copy included. The shadow switch runs today's scan on each certified
// copy and refuses a copy it would move.
func TestPlacementCertificateKeepsPlacementsIdentical(t *testing.T) {
	facetproof.PlacementCertificateShadow.Store(true)
	t.Cleanup(func() { facetproof.PlacementCertificateShadow.Store(false) })
	verts, tris := certSphere(4, 5, r3.Vec{X: 1, Y: 2, Z: 3})
	require.GreaterOrEqual(t, len(tris), meshbool.CertificateMinFacets)
	ctx := meshbool.WithPlacementCertificates(t.Context())
	cache := meshbool.PlacementCertificatesFrom(ctx)
	for k, tc := range []struct {
		axis    r3.Vec
		degrees float64
		tr      r3.Vec
		mirror  bool
	}{
		{r3.Vec{Z: 1}, 11.6129, r3.Vec{}, false},
		{r3.Vec{Z: 1}, 23.2258, r3.Vec{}, false},
		{r3.Vec{X: 1, Y: 1, Z: 1}, 35, r3.Vec{X: 100, Y: -40, Z: 7}, false},
		{r3.Vec{X: 1, Y: 2}, 71, r3.Vec{X: -3e4, Y: 2e3}, true},
		{r3.Vec{Y: 1}, 127.5, r3.Vec{Z: 1e5}, false},
	} {
		d := certMotion(t, tc.axis, tc.degrees, tc.tr, tc.mirror)
		want, err := placeCertified(t.Context(), verts, tris, d)
		require.NoError(t, err)
		got, err := placeCertified(ctx, verts, tris, d)
		require.NoError(t, err)
		requireSamePlacement(t, want, got, "placement")
		if k == 0 {
			require.Zero(t, cache.Built(), "one request pays no source pass")
		}
	}
	require.Equal(t, 1, cache.Built())
	cert, err := cache.Certificate(ctx, verts, tris)
	require.NoError(t, err)
	require.NotNil(t, cert)
	require.Equal(t, 4, cert.Hits(), "every placement after the first is certified")
}

// A source below the facet floor never asks for a certificate, and a copy whose
// rounding exceeds the covered radius still places exactly as before.
func TestPlacementCertificateFallsBackWithoutChangingTheResult(t *testing.T) {
	t.Parallel()
	small, smallTris := certSphere(2, 3, r3.Vec{})
	ctx := meshbool.WithPlacementCertificates(t.Context())
	d := certMotion(t, r3.Vec{Z: 1}, 20, r3.Vec{X: 1}, false)
	for range 4 {
		_, err := placeCertified(ctx, small, smallTris, d)
		require.NoError(t, err)
	}
	require.Zero(t, meshbool.PlacementCertificatesFrom(ctx).Built())

	verts, tris := certSphere(4, 5, r3.Vec{})
	far := certMotion(t, r3.Vec{X: 1, Y: 1}, 50, r3.Vec{X: 4e5, Y: -3e5}, false)
	want, err := placeCertified(t.Context(), verts, tris, far)
	require.NoError(t, err)
	for range 3 {
		got, err := placeCertified(ctx, verts, tris, far)
		require.NoError(t, err)
		requireSamePlacement(t, want, got, "far copy")
	}
	cert, err := meshbool.PlacementCertificatesFrom(ctx).Certificate(ctx, verts, tris)
	require.NoError(t, err)
	require.NotNil(t, cert)
	require.Zero(t, cert.Hits(), "a copy 500 m away rounds past the certified radius")
}

// A copy that folds after rounding is not certified: the placement repairs or
// refuses it exactly as it does without the cache.
func TestPlacementCertificateLeavesFoldedCopiesToTheRepair(t *testing.T) {
	t.Parallel()
	verts, tris := certSphere(4, 5, r3.Vec{})
	// Two parallel plates, a hair apart, join the sphere so that rounding at a
	// distance folds them.
	n := len(verts)
	verts = append(verts, r3.Vec{X: 20}, r3.Vec{X: 21}, r3.Vec{X: 21, Y: 1}, r3.Vec{X: 20, Y: 1},
		r3.Vec{X: 20, Z: 1e-13}, r3.Vec{X: 21, Z: 1e-13}, r3.Vec{X: 21, Y: 1, Z: 1e-13}, r3.Vec{X: 20, Y: 1, Z: 1e-13})
	for _, t := range [][3]int{{0, 1, 2}, {0, 2, 3}, {4, 6, 5}, {4, 7, 6}} {
		tris = append(tris, [3]int{t[0] + n, t[1] + n, t[2] + n})
	}
	ctx := gaptol.With(meshbool.WithPlacementCertificates(t.Context()), 0.01)
	plain := gaptol.With(t.Context(), 0.01)
	repaired := 0
	for k := range 4 {
		d := certMotion(t, r3.Vec{X: 1, Y: 3, Z: 2}, float64(13+17*k), r3.Vec{X: 1e4, Y: -1.3e4 - 2e3*float64(k), Z: 1.1e4}, false)
		want, wantErr := placeCertified(plain, verts, tris, d)
		got, gotErr := placeCertified(ctx, verts, tris, d)
		require.Equal(t, wantErr == nil, gotErr == nil, "copy %d error: %v vs %v", k, wantErr, gotErr)
		if wantErr == nil {
			requireSamePlacement(t, want, got, "folded copy")
			repaired += btoi(want.EmbeddingReach != nil)
		}
	}
	require.Positive(t, repaired, "no copy folded, so the test checked nothing")
}

// Placements through an exact rotation take the same certificate and return the
// same mesh.
func TestPlacementCertificateWithAnExactRotation(t *testing.T) {
	t.Parallel()
	verts, tris := certSphere(4, 5, r3.Vec{X: 2})
	rotation, d, err := exactrotation.New(r3.Vec{X: 1, Y: 2}, r3.Vec{Z: 1}, units.Radians(0.7))
	require.NoError(t, err)
	ctx := meshbool.WithPlacementCertificates(t.Context())
	beta := make([]float64, len(verts))
	want, err := facetproof.PlaceRotatedMesh(t.Context(), proofbound.NewWorkBudget(t.Context()), verts, tris, beta, 0, 0, d, rotation)
	require.NoError(t, err)
	for range 3 {
		got, err := facetproof.PlaceRotatedMesh(ctx, proofbound.NewWorkBudget(ctx), verts, tris, beta, 0, 0, d, rotation)
		require.NoError(t, err)
		requireSamePlacement(t, want, got, "rotated copy")
	}
	cert, err := meshbool.PlacementCertificatesFrom(ctx).Certificate(ctx, verts, tris)
	require.NoError(t, err)
	require.NotNil(t, cert)
	require.Positive(t, cert.Hits())
}

// Many goroutines placing the same source share one cache: every copy matches
// the cacheless placement, and the race detector sees no shared write.
func TestPlacementCertificateConcurrentPlacements(t *testing.T) {
	t.Parallel()
	verts, tris := certSphere(4, 7, r3.Vec{X: -1})
	ctx := meshbool.WithPlacementCertificates(t.Context())
	const workers = 8
	plain := t.Context()
	var group sync.WaitGroup
	wants := make([]facetproof.PlacedMesh, workers)
	gots := make([]facetproof.PlacedMesh, workers)
	errs := make([]error, 2*workers)
	for w := range workers {
		group.Go(func() {
			d := certMotion(t, r3.Vec{X: 1, Y: float64(w), Z: 1}, float64(9*w+5), r3.Vec{X: float64(w) * 10}, w%3 == 0)
			wants[w], errs[w] = placeCertified(plain, verts, tris, d)
			for range 2 {
				gots[w], errs[workers+w] = placeCertified(ctx, verts, tris, d)
			}
		})
	}
	group.Wait()
	for _, err := range errs {
		require.NoError(t, err)
	}
	for w := range workers {
		requireSamePlacement(t, wants[w], gots[w], "concurrent copy")
	}
	require.LessOrEqual(t, meshbool.PlacementCertificatesFrom(ctx).Built(), 1)
	require.False(t, math.IsNaN(gots[0].MeshBound))
}

func btoi(b bool) int {
	if b {
		return 1
	}
	return 0
}
