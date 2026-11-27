package meshbool

import (
	"context"
	"errors"
	"fmt"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// conformOnceTwoScan is the conforming pass as it stood before the twin-edge
// memo: every directed facet edge walks the point tree through
// ConformScan.EdgeInteriorHits. It is the reference the memoized pass is held to.
func conformOnceTwoScan(ctx context.Context, budget *proofbound.WorkBudget, verts []proof.Xpt, approx []r3.Vec,
	tris *[][3]int, src *[]int, bounds *[]float64, xbeta []float64) (bool, error) {
	if err := budget.Err(); err != nil {
		return false, err
	}
	scan, err := newConformScanApprox(budget, verts, approx)
	if err != nil {
		return false, err
	}
	var outTris [][3]int
	var outSrc []int
	var outBounds []float64
	splitAny := false
	for ti, tri := range *tris {
		if err := budget.Step(); err != nil {
			return false, err
		}
		inserted := [3][]int{}
		for k := range 3 {
			a, b := tri[k], tri[(k+1)%3]
			hits, err := scan.EdgeInteriorHits(budget, a, b, tri)
			if err != nil {
				return false, err
			}
			if len(hits) > 0 {
				if err := SortAlongEdge(budget, verts, a, b, hits); err != nil {
					return false, err
				}
				inserted[k] = hits
			}
		}
		if inserted[0] == nil && inserted[1] == nil && inserted[2] == nil {
			if splitAny {
				outTris = append(outTris, tri)
				outSrc = append(outSrc, (*src)[ti])
				outBounds = append(outBounds, (*bounds)[ti])
			}
			continue
		}
		if !splitAny {
			outTris = append(outTris, (*tris)[:ti]...)
			outSrc = append(outSrc, (*src)[:ti]...)
			outBounds = append(outBounds, (*bounds)[:ti]...)
		}
		splitAny = true
		for _, hits := range inserted {
			for _, h := range hits {
				xbeta[h] = max(xbeta[h], (*bounds)[ti])
			}
		}
		poly := []int{tri[0]}
		poly = append(poly, inserted[0]...)
		poly = append(poly, tri[1])
		poly = append(poly, inserted[1]...)
		poly = append(poly, tri[2])
		poly = append(poly, inserted[2]...)
		newTris, err := TriangulatePlanarPolygon(ctx, verts, poly)
		if err != nil {
			return false, err
		}
		for _, nt := range newTris {
			outTris = append(outTris, nt)
			outSrc = append(outSrc, (*src)[ti])
			outBounds = append(outBounds, (*bounds)[ti])
		}
	}
	if splitAny {
		*tris = outTris
		*src = outSrc
		*bounds = outBounds
	}
	return splitAny, nil
}

// twinEdgeMesh builds a mesh whose facets share edges in both directions, in
// one direction twice, or with extra users, and whose edges carry lattice
// points, isolated points, and degenerate facets whose third vertex lies on
// the edge. All coordinates are small lattice values, so exact collinearity is
// common.
func twinEdgeMesh(rng *rand.Rand, nTris int) approxMesh {
	var m approxMesh
	addVert := func(p r3.Vec) {
		m.verts = append(m.verts, proof.XptOf(p))
		m.xbeta = append(m.xbeta, float64(rng.IntN(3))/8)
	}
	for range 20 {
		addVert(r3.Vec{X: float64(rng.IntN(6)), Y: float64(rng.IntN(6)), Z: float64(rng.IntN(2))})
	}
	add := func(t [3]int) {
		m.tris = append(m.tris, t)
		m.src = append(m.src, rng.IntN(3))
		m.bounds = append(m.bounds, float64(rng.IntN(4))/8)
	}
	for len(m.tris) < nTris {
		n := len(m.verts)
		switch k := rng.IntN(8); {
		case k < 3 && len(m.tris) > 0:
			// A twin: the reverse of an earlier facet's edge with a new apex.
			t := m.tris[rng.IntN(len(m.tris))]
			e := rng.IntN(3)
			add([3]int{t[(e+1)%3], t[e], rng.IntN(n)})
		case k == 3 && len(m.tris) > 0:
			// The same facet again, reversed and not.
			t := m.tris[rng.IntN(len(m.tris))]
			if rng.IntN(2) == 0 {
				t[1], t[2] = t[2], t[1]
			}
			add(t)
		case k == 4 && len(m.tris) > 0:
			// A vertex dropped on an earlier facet's edge, sometimes at a
			// quarter of the way, so some edges gain interior points.
			t := m.tris[rng.IntN(len(m.tris))]
			e := rng.IntN(3)
			pa, pb := m.verts[t[e]].Vec(), m.verts[t[(e+1)%3]].Vec()
			f := []float64{0.5, 0.25, 0.75}[rng.IntN(3)]
			addVert(pa.Add(pb.Sub(pa).Scale(f)))
		case k == 5 && len(m.tris) > 0:
			// A facet whose third vertex lies on an earlier facet's edge line.
			t := m.tris[rng.IntN(len(m.tris))]
			e := rng.IntN(3)
			add([3]int{t[e], t[(e+1)%3], rng.IntN(n)})
		default:
			add([3]int{rng.IntN(n), rng.IntN(n), rng.IntN(n)})
		}
	}
	return m
}

func (m approxMesh) rounded() []r3.Vec {
	approx := make([]r3.Vec, len(m.verts))
	for i, p := range m.verts {
		approx[i] = p.Vec()
	}
	return approx
}

// errConformStep is what a refusing step budget returns.
var errConformStep = errors.New("conform step budget exhausted")

// conformRun is one pass over a mesh with a step-counting budget that
// refuses once limit steps were charged (limit 0 never refuses).
type conformRun struct {
	split bool
	err   error
	steps uint64
	mesh  approxMesh
}

type conformPass func(*proofbound.WorkBudget, *approxMesh, []r3.Vec) (bool, error)

func runConformPass(t *testing.T, base approxMesh, limit uint64, pass conformPass) conformRun {
	t.Helper()
	m := base.clone()
	var steps uint64
	budget := &proofbound.WorkBudget{
		StepFn: func() error {
			steps++
			if limit != 0 && steps >= limit {
				return errConformStep
			}
			return nil
		},
		ErrFn: func() error { return nil },
	}
	split, err := pass(budget, &m, base.rounded())
	return conformRun{split: split, err: err, steps: steps, mesh: m}
}

func twoScanPass(t *testing.T) conformPass {
	return func(b *proofbound.WorkBudget, m *approxMesh, a []r3.Vec) (bool, error) {
		return conformOnceTwoScan(t.Context(), b, m.verts, a, &m.tris, &m.src, &m.bounds, m.xbeta)
	}
}

func memoPass(t *testing.T) conformPass {
	return func(b *proofbound.WorkBudget, m *approxMesh, a []r3.Vec) (bool, error) {
		return conformOnceBudget(t.Context(), b, m.verts, a, &m.tris, &m.src, &m.bounds, m.xbeta)
	}
}

// requireSameRun requires the same refusal, steps charged, split flag and mesh
// state. A refusal leaves the triangle arrays alone but may already have
// raised xbeta; both must match either way.
func requireSameRunf(t *testing.T, want, got conformRun, format string, args ...any) {
	t.Helper()
	msg := fmt.Sprintf(format, args...)
	require.Equal(t, want.err, got.err, msg)
	require.Equal(t, want.steps, got.steps, msg)
	require.Equal(t, want.split, got.split, msg)
	requireSameMesh(t, 0, want.mesh, got.mesh)
}
