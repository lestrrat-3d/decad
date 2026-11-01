package meshbool

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func deepWitnessAtShared(ctx context.Context, p proof.Xpt, other *BoolMesh, b float64,
	hint *int, budget *proofbound.WorkBudget) (bool, error) {
	inside, err := deepWitnessAtPathProof(ctx, p, other, b, hint, budget)
	return inside, err
}

func deepWitnessAtPathProof(ctx context.Context, p proof.Xpt, other *BoolMesh, b float64,
	hint *int, budget *proofbound.WorkBudget) (bool, error) {
	deep, err := depthBeyondShared(ctx, p, other, b, hint, budget)
	if err != nil || !deep {
		return false, err
	}
	return witnessInsideShared(ctx, p, other, budget)
}

func witnessInsideShared(ctx context.Context, p proof.Xpt, other *BoolMesh,
	budget *proofbound.WorkBudget) (bool, error) {
	inside, boundary, err := meshParityIndexedShared(ctx, p, other.Parity, budget)
	if err != nil {
		if errors.Is(err, decaderr.ErrBooleanFailed) && ctx.Err() == nil {
			return false, nil
		}
		return false, err
	}
	return inside && !boundary, nil
}

func deepWitnessInsideCached(ctx context.Context, m *BoolMesh, closeFacets []int,
	other *BoolMesh, b float64, budget *proofbound.WorkBudget) (bool, error) {
	hint := -1
	for n, i := range closeFacets {
		if n >= MaxDepthWitnessFacets {
			break
		}
		if err := budget.Step(); err != nil {
			return false, err
		}
		tri := m.Tris[i]
		samples, err := facetSamplePointsShared(ctx, m.Xverts[tri[0]], m.Xverts[tri[1]], m.Xverts[tri[2]], budget)
		if err != nil {
			return false, err
		}
		for _, p := range samples {
			if err := budget.Step(); err != nil {
				return false, err
			}
			deep, err := deepWitnessAtPathProof(ctx, p, other, b, &hint, budget)
			if err != nil || deep {
				return deep, err
			}
		}
	}
	return false, nil
}

func walkFacetFromSpanShared(ctx context.Context, mid proof.Xpt, m *BoolMesh, fi int,
	other *BoolMesh, fj int, b float64, budget *proofbound.WorkBudget) (bool, error) {
	ot := other.Tris[fj]
	origin, normal := other.Xverts[ot[0]], other.Norms[fj]
	var midF r3.Vec
	var err error
	if budget.Replay {
		midF, err = proof.ReplayXptVec(ctx, mid, replayExactWork{budget})
		if err != nil {
			return false, err
		}
	} else {
		midF = mid.Vec()
	}
	one := replayExactOne
	hint := fj
	for _, vi := range m.Tris[fi] {
		if err := budget.Step(); err != nil {
			return false, err
		}
		c := m.Xverts[vi]
		side := 0
		if budget.Replay {
			side, err = proof.ReplayXdotDiffSign(ctx, normal, c, origin, replayExactWork{budget})
			if err != nil {
				return false, err
			}
		} else {
			side = proof.XdotSign(normal, proof.Xsub(c, origin))
		}
		if side >= 0 {
			continue
		}
		reach := m.Verts[vi].Sub(midF).Len()
		den := replayExactOne
		for step := range SpanWalkSteps {
			if err := budget.Step(); err != nil {
				return false, err
			}
			if budget.Replay {
				if err := budget.ReserveBytes(256); err != nil {
					return false, err
				}
				den = big.NewInt(int64(1) << (step + 1))
			} else {
				den = new(big.Int).Lsh(den, 1)
			}
			reach /= 2
			if reach*(1+1e-6) <= b {
				break
			}
			var p proof.Xpt
			if budget.Replay {
				p, err = proof.ReplayXlerp(ctx, mid, c, one, den, replayExactWork{budget})
				if err != nil {
					return false, err
				}
			} else {
				p = proof.Xlerp(mid, c, one, den)
			}
			deep, err := deepWitnessAtShared(ctx, p, other, b, &hint, budget)
			if err != nil || deep {
				return deep, err
			}
		}
	}
	return false, nil
}

func provenDepthExceedsShared(ctx context.Context, a *BoolMesh, closeA []int, b *BoolMesh,
	closeB []int, spans []ContactSpan, threshold float64, budget *proofbound.WorkBudget) (bool, error) {
	deep, err := deepWitnessInsideCached(ctx, a, closeA, b, threshold, budget)
	if err != nil || deep {
		return deep, err
	}
	deep, err = deepWitnessInsideCached(ctx, b, closeB, a, threshold, budget)
	if err != nil || deep {
		return deep, err
	}
	one, two := replayExactOne, replayDepthTwo
	for n, s := range spans {
		if n >= MaxDepthWitnessSpans {
			break
		}
		if err := budget.Step(); err != nil {
			return false, err
		}
		var mid proof.Xpt
		if budget.Replay {
			mid, err = proof.ReplayXlerp(ctx, s.P0, s.P1, one, two, replayExactWork{budget})
			if err != nil {
				return false, err
			}
		} else {
			mid = proof.Xlerp(s.P0, s.P1, one, two)
		}
		deep, err = walkFacetFromSpanShared(ctx, mid, a, s.I, b, s.J, threshold, budget)
		if err != nil || deep {
			return deep, err
		}
		deep, err = walkFacetFromSpanShared(ctx, mid, b, s.J, a, s.I, threshold, budget)
		if err != nil || deep {
			return deep, err
		}
	}
	return false, nil
}

// depthBeyondShared performs the existing depth test while charging each
// visited block and facet to the caller's work budget.
func depthBeyondShared(ctx context.Context, p proof.Xpt, other *BoolMesh, b float64, hint *int,
	budget *proofbound.WorkBudget) (bool, error) {
	var pf r3.Vec
	var err error
	if budget.Replay {
		pf, err = proof.ReplayXptVec(ctx, p, replayExactWork{budget})
		if err != nil {
			return false, err
		}
	} else {
		pf = p.Vec()
	}
	settled, err := depthHintsRejectShared(pf, other, b, hint, budget)
	if err != nil || settled {
		return false, err
	}
	return depthScanBeyondShared(ctx, p, other, b, hint, budget)
}

func depthHintsRejectShared(pf r3.Vec, other *BoolMesh, b float64, hint *int,
	budget *proofbound.WorkBudget) (bool, error) {
	_, proposed, err := depthHintProposalShared(pf, other, b, hint, budget)
	return proposed, err
}

// depthHintProposalShared identifies a facet that settles the rounded scan.
// Its result alone cannot reject the independent exact depth certificate.
func depthHintProposalShared(pf r3.Vec, other *BoolMesh, b float64, hint *int,
	budget *proofbound.WorkBudget) (int, bool, error) {
	if hint != nil && *hint >= 0 && *hint < len(other.Tris) {
		if err := budget.Step(); err != nil {
			return -1, false, err
		}
		h := *hint
		if !validDepthFacet(other, h) || h >= len(other.Boxes) {
			return -1, false, fmt.Errorf("%w: invalid depth hint facet %d", decaderr.ErrUnsupported, h)
		}
		up := math.Max(PointBoxDist(pf, other.Boxes[h]), PointTriDistance(pf, TriCorners(other, h)))
		if up*(1-1e-12) <= b {
			return h, true, nil
		}
	}

	return -1, false, nil
}

func validDepthFacet(mesh *BoolMesh, id int) bool {
	if id < 0 || id >= len(mesh.Tris) {
		return false
	}
	for _, v := range mesh.Tris[id] {
		if v < 0 || v >= len(mesh.Verts) {
			return false
		}
	}
	return true
}

func depthScanBeyondShared(ctx context.Context, p proof.Xpt, other *BoolMesh, b float64, hint *int,
	budget *proofbound.WorkBudget) (bool, error) {
	var pf r3.Vec
	var err error
	if budget.Replay {
		pf, err = proof.ReplayXptVec(ctx, p, replayExactWork{budget})
		if err != nil {
			return false, err
		}
	} else {
		pf = p.Vec()
	}
	s := depthScan{pf: pf, other: other, b: b, best: math.Inf(1)}
	settled, err := scanDepthShared(&s, 0, budget)
	if err != nil {
		return false, err
	}
	if settled >= 0 {
		if hint != nil {
			*hint = settled
		}
		return false, nil
	}
	depth := 0.0
	if s.best > 0 && !proofbound.IsNonFinite(s.best) {
		depth = s.best*(1-1e-12) - PointRoundBound(p, pf)
	}
	return !(depth <= b), nil
}

// scanDepthShared is the charged original source-order scan. A nonzero start
// resumes its facet loop after a spatial query declines.
func scanDepthShared(s *depthScan, start int, budget *proofbound.WorkBudget) (int, error) {
	facets := func(lo, hi int) (int, error) {
		for i := lo; i < hi; i++ {
			if err := budget.Step(); err != nil {
				return -1, err
			}
			if PointBoxDist(s.pf, s.other.Boxes[i]) >= s.best {
				continue
			}
			s.best = math.Min(s.best, PointTriDistance(s.pf, TriCorners(s.other, i)))
			if s.best*(1-1e-12) <= s.b {
				return i, nil
			}
		}
		return -1, nil
	}
	var block func(int, int) (int, error)
	block = func(level, j int) (int, error) {
		if err := budget.Step(); err != nil {
			return -1, err
		}
		if PointBoxDist(s.pf, s.other.depthLevels[level][j]) >= s.best {
			return -1, nil
		}
		lo := j * depthFanout
		if level == 0 {
			return facets(lo, min(lo+depthFanout, len(s.other.Tris)))
		}
		for k := lo; k < min(lo+depthFanout, len(s.other.depthLevels[level-1])); k++ {
			i, err := block(level-1, k)
			if err != nil || i >= 0 {
				return i, err
			}
		}
		return -1, nil
	}
	n := len(s.other.Tris)
	top := len(s.other.depthLevels) - 1
	if start > 0 {
		return facets(start, n)
	}
	if _, err := budget.StepN(uint64(len(s.other.depthLevels))); err != nil {
		return -1, err
	}
	if top < 0 || !s.other.depthBlocksFit(n) {
		return facets(0, n)
	}
	return block(top, 0)
}

func facetSamplePointsShared(ctx context.Context, a, b, c proof.Xpt, budget *proofbound.WorkBudget) ([]proof.Xpt, error) {
	if !budget.Replay {
		return facetSamplePoints(a, b, c), nil
	}
	if err := budget.Reserve(7); err != nil {
		return nil, err
	}
	if err := budget.ReserveBytesMul(7, uint64(unsafe.Sizeof(proof.Xpt{}))); err != nil {
		return nil, err
	}
	out := make([]proof.Xpt, 7)
	out[0], out[1], out[2] = a, b, c
	pairs := [3][2]proof.Xpt{{a, b}, {b, c}, {c, a}}
	for i, p := range pairs {
		v, err := proof.ReplayXlerp(ctx, p[0], p[1], replayExactOne, replayDepthTwo, replayExactWork{budget})
		if err != nil {
			return nil, err
		}
		out[3+i] = v
	}
	v, err := proof.ReplayCentroid(ctx, a, b, c, replayExactWork{budget})
	if err != nil {
		return nil, err
	}
	out[6] = v
	return out, nil
}

var replayDepthTwo = big.NewInt(2)
