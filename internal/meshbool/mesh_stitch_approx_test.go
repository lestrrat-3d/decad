package meshbool

import (
	"math"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// approxMesh is the state the conforming passes and the needle flip share.
type approxMesh struct {
	verts  []proof.Xpt
	tris   [][3]int
	src    []int
	bounds []float64
	xbeta  []float64
}

func (m approxMesh) clone() approxMesh {
	return approxMesh{
		verts:  slices.Clone(m.verts),
		tris:   slices.Clone(m.tris),
		src:    slices.Clone(m.src),
		bounds: slices.Clone(m.bounds),
		xbeta:  slices.Clone(m.xbeta),
	}
}

// conformFlipOld is the stitch flow before the vertices were rounded once:
// the exported wrappers, each rounding the vertices itself.
func conformFlipOld(t *testing.T, m *approxMesh) (int, error) {
	t.Helper()
	for round := 0; ; round++ {
		require.LessOrEqual(t, round, 6)
		split, err := ConformOnce(t.Context(), m.verts, &m.tris, &m.src, &m.bounds, m.xbeta)
		if err != nil {
			return 0, err
		}
		if !split {
			break
		}
	}
	return FlipNeedles(t.Context(), m.verts, m.tris, m.src, m.bounds)
}

// conformFlipNew is the flow StitchFacetsContext runs: one rounding shared by
// every pass.
func conformFlipNew(t *testing.T, m *approxMesh) (int, error) {
	t.Helper()
	approx := make([]r3.Vec, len(m.verts))
	for i, p := range m.verts {
		approx[i] = p.Vec()
	}
	for round := 0; ; round++ {
		require.LessOrEqual(t, round, 6)
		split, err := conformOnceApprox(t.Context(), m.verts, approx, &m.tris, &m.src, &m.bounds, m.xbeta)
		if err != nil {
			return 0, err
		}
		if !split {
			break
		}
	}
	return flipNeedlesHeld(t.Context(), m.verts, approx, m.tris, m.src, m.bounds)
}

func requireSameMesh(t *testing.T, seed uint64, want, got approxMesh) {
	t.Helper()
	require.Equal(t, want.tris, got.tris, "seed %d", seed)
	require.Equal(t, want.src, got.src, "seed %d", seed)
	require.Equal(t, want.bounds, got.bounds, "seed %d", seed)
	require.Equal(t, want.xbeta, got.xbeta, "seed %d", seed)
	require.Len(t, got.verts, len(want.verts), "seed %d", seed)
	for i := range want.verts {
		require.Equal(t, want.verts[i].Key(), got.verts[i].Key(), "seed %d vertex %d", seed, i)
	}
}

// randomApproxMesh scatters lattice points, some nudged 2^-50 off the lattice,
// and joins random triples into coplanar facets of two sources. Lattice points
// land on other facets' edges, so the conforming pass splits, and the nudged
// points make needles for the flip.
func randomApproxMesh(rng *rand.Rand) approxMesh {
	tiny := math.Ldexp(1, -50)
	var m approxMesh
	for range 24 {
		p := r3.Vec{X: float64(rng.IntN(5)), Y: float64(rng.IntN(5))}
		if rng.IntN(4) == 0 {
			p.Y += tiny
		}
		m.verts = append(m.verts, proof.XptOf(p))
		m.xbeta = append(m.xbeta, float64(rng.IntN(3))/8)
	}
	for len(m.tris) < 14 {
		a, b, c := rng.IntN(len(m.verts)), rng.IntN(len(m.verts)), rng.IntN(len(m.verts))
		if a == b || b == c || c == a {
			continue
		}
		pa, pb, pc := m.verts[a].Vec(), m.verts[b].Vec(), m.verts[c].Vec()
		if pb.Sub(pa).Cross(pc.Sub(pa)).Len() == 0 {
			continue
		}
		m.tris = append(m.tris, [3]int{a, b, c})
		m.src = append(m.src, rng.IntN(2))
		m.bounds = append(m.bounds, float64(rng.IntN(4))/8)
	}
	return m
}

// TestConformFlipSharedApproxMatchesPerCallRounding holds the shared rounding
// to the per-call one: the conforming passes and the needle flip give the
// same triangles, sources and bounds. Some seed must split and some must flip.
//
// Leg shown to fail: leaving one entry of the shared slice stale (zeroed)
// changes the candidate set of the conforming scan and the needle test.
func TestConformFlipSharedApproxMatchesPerCallRounding(t *testing.T) {
	t.Parallel()
	splits, flips := 0, 0
	for seed := range uint64(300) {
		base := randomApproxMesh(rand.New(rand.NewPCG(seed, 7)))
		want, got := base.clone(), base.clone()
		wantFlips, wantErr := conformFlipOld(t, &want)
		gotFlips, gotErr := conformFlipNew(t, &got)
		// A soup the exact re-triangulation cannot split fails the same way.
		require.Equal(t, wantErr, gotErr, "seed %d", seed)
		require.Equal(t, wantFlips, gotFlips, "seed %d", seed)
		requireSameMesh(t, seed, want, got)
		if wantErr != nil {
			continue
		}
		if len(want.tris) > len(base.tris) {
			splits++
		}
		flips += wantFlips
	}
	require.Positive(t, splits, "no seed split a facet")
	require.Positive(t, flips, "no seed flipped a needle")
}

// stepCountBudget counts Step calls and never refuses.
func stepCountBudget(steps *uint64) *proofbound.WorkBudget {
	return &proofbound.WorkBudget{
		StepFn: func() error { *steps++; return nil },
		ErrFn:  func() error { return nil },
	}
}

// TestConformScanApproxMatchesWrapper holds the exported scan constructor, with
// no shared rounding, to the unexported one given an explicit rounding: the
// same scan fields and the same work charged.
func TestConformScanApproxMatchesWrapper(t *testing.T) {
	t.Parallel()
	for seed := range uint64(40) {
		m := randomApproxMesh(rand.New(rand.NewPCG(seed, 11)))
		approx := make([]r3.Vec, len(m.verts))
		for i, p := range m.verts {
			approx[i] = p.Vec()
		}
		var wantSteps, gotSteps uint64
		want, err := NewConformScan(stepCountBudget(&wantSteps), m.verts)
		require.NoError(t, err)
		got, err := newConformScanApprox(stepCountBudget(&gotSteps), m.verts, approx)
		require.NoError(t, err)
		require.Equal(t, wantSteps, gotSteps, "seed %d", seed)
		require.Equal(t, want.Approx, got.Approx)
		require.Equal(t, want.pointCells, got.pointCells)
		require.Equal(t, want.Lo, got.Lo)
		require.Equal(t, want.Cell, got.Cell)
		require.Equal(t, want.Slack, got.Slack)
		require.Equal(t, want.Tau2, got.Tau2)
		// The scan reads the shared slice and leaves it as it found it.
		for i, p := range m.verts {
			require.Equal(t, p.Vec(), approx[i])
		}
	}
}

// TestConformOnceNilApproxMatchesExplicit holds ConformOnce to the shared
// variant given the rounding it would compute for itself, one pass at a time.
func TestConformOnceNilApproxMatchesExplicit(t *testing.T) {
	t.Parallel()
	for seed := range uint64(100) {
		base := randomApproxMesh(rand.New(rand.NewPCG(seed, 13)))
		want, got := base.clone(), base.clone()
		approx := make([]r3.Vec, len(base.verts))
		for i, p := range base.verts {
			approx[i] = p.Vec()
		}
		wantSplit, wantErr := ConformOnce(t.Context(), want.verts, &want.tris, &want.src, &want.bounds, want.xbeta)
		gotSplit, gotErr := conformOnceApprox(t.Context(), got.verts, approx, &got.tris, &got.src, &got.bounds, got.xbeta)
		require.Equal(t, wantErr, gotErr, "seed %d", seed)
		require.Equal(t, wantSplit, gotSplit, "seed %d", seed)
		requireSameMesh(t, seed, want, got)
	}
}
