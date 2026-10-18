package facetproof

import (
	"context"
	"errors"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

const (
	boundedFacetLimit     = 512
	boundedVertexLimit    = 512
	boundedVolumeRatBits  = 4096
	boundedFacetLiftWork  = 32
	boundedFacetDetWork   = 64
	boundedComponentWork  = 256
	boundedVolumeReadWork = 256
)

// boundedFacetCharge preserves exhaustion as an error until an entry point
// has checked its own cancellation source. The caller reserves capacity and
// exact-operation work before allocating or computing.
func boundedFacetCharge(budget *proofbound.WorkBudget, n uint64) (bool, error) {
	_, err := budget.StepN(n)
	if err == nil {
		return true, nil
	}
	if !errors.Is(err, proofbound.ErrWallWorkBudget) {
		return false, err
	}
	if stop := budget.Err(); stop != nil && !errors.Is(stop, proofbound.ErrWallWorkBudget) {
		return false, stop
	}
	return false, err
}

func boundedFacetPoll(budget *proofbound.WorkBudget) (bool, error) {
	if err := budget.Err(); err != nil {
		if errors.Is(err, proofbound.ErrWallWorkBudget) {
			return false, nil
		}
		return false, err
	}
	return true, nil
}

// boundedFacetCoordinate admits only float lifts whose complete determinant
// sum stays below the private 8192-bit exact-arithmetic ceiling. A nonzero
// float with Frexp exponent in [-459,129] has reduced dyadic exponent at
// least -512, so each lifted point has W <= 2^512 and coordinate numerator
// magnitude < 2^641 (up to 641 bits). A cross, dot, and 512-term aligned
// determinant sum then have unreduced intermediates below 4096 bits;
// multiplication by 1/6 adds at most three denominator bits.
func boundedFacetCoordinate(c float64) bool {
	if math.IsNaN(c) || math.IsInf(c, 0) {
		return false
	}
	if c == 0 {
		return true
	}
	_, exp := math.Frexp(c)
	return exp >= -459 && exp <= 129
}

// AuditFacetedMeshBounded certifies the private direct-shell class using the
// caller's shared work budget. It admits one closed, consistently oriented
// component with at most 512 held vertices and facets. Other shell classes,
// arithmetic outside the preflight range, and exhausted work safely refuse.
// Existing AuditFacetedMesh callers retain their separate contract.
// ctx and budget must be non-nil.
func AuditFacetedMeshBounded(ctx context.Context, budget *proofbound.WorkBudget,
	verts []r3.Vec, tris [][3]int) (*MeshAudit, bool, error) {
	audit, admitted, err := auditFacetedMeshBounded(ctx, budget, verts, tris)
	if ctx != nil {
		if cancellation := ctx.Err(); cancellation != nil {
			return nil, false, cancellation
		}
	}
	if errors.Is(err, proofbound.ErrWallWorkBudget) {
		return nil, false, nil
	}
	return audit, admitted, err
}

func auditFacetedMeshBounded(ctx context.Context, budget *proofbound.WorkBudget,
	verts []r3.Vec, tris [][3]int) (*MeshAudit, bool, error) {
	if ctx == nil || budget == nil {
		return nil, false, errors.New("bounded facet audit requires context and budget")
	}
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}
	if len(verts) == 0 || len(verts) > boundedVertexLimit ||
		len(tris) == 0 || len(tris) > boundedFacetLimit {
		return nil, false, nil
	}
	vertexCount, facetCount := len(verts), len(tris)
	incidenceCount := 3 * facetCount // facetCount <= 512, so this cannot overflow.
	for _, v := range verts {
		for _, c := range [3]float64{v.X, v.Y, v.Z} {
			ok, err := boundedFacetCharge(budget, 1)
			if err != nil || !ok {
				return nil, false, err
			}
			if !boundedFacetCoordinate(c) {
				return nil, false, nil
			}
		}
	}
	for _, tri := range tris {
		for _, vi := range tri {
			ok, err := boundedFacetCharge(budget, 1)
			if err != nil || !ok {
				return nil, false, err
			}
			if vi < 0 || vi >= vertexCount {
				return nil, false, nil
			}
		}
		ok, err := boundedFacetCharge(budget, 1)
		if err != nil || !ok {
			return nil, false, err
		}
		if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] {
			return nil, false, nil
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}

	// An edge has exactly one directed owner and one reverse owner. Store the
	// owner index plus one so a zero map value means absent. Filling each
	// adjacency slot at its original local edge index preserves the legacy
	// component walk and determinant order.
	ok, err := boundedFacetCharge(budget, uint64(incidenceCount))
	if err != nil || !ok {
		return nil, false, err
	}
	owner := make(map[[2]int]int, incidenceCount)
	for i, tri := range tris {
		for edge := range 3 {
			ok, err := boundedFacetCharge(budget, 1)
			if err != nil || !ok {
				return nil, false, err
			}
			key := [2]int{tri[edge], tri[(edge+1)%3]}
			if owner[key] != 0 {
				return nil, false, nil
			}
			owner[key] = i + 1
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}

	ok, err = boundedFacetCharge(budget, uint64(facetCount+incidenceCount))
	if err != nil || !ok {
		return nil, false, err
	}
	adj := make([][]int, facetCount)
	neighbors := make([]int, incidenceCount)
	for i, tri := range tris {
		adj[i] = neighbors[3*i : 3*i+3]
		for edge := range 3 {
			ok, err := boundedFacetCharge(budget, 1)
			if err != nil || !ok {
				return nil, false, err
			}
			twin := owner[[2]int{tri[(edge+1)%3], tri[edge]}]
			if twin == 0 || twin == i+1 {
				return nil, false, nil
			}
			adj[i][edge] = twin - 1
		}
	}

	ok, err = boundedFacetCharge(budget, uint64(vertexCount+1))
	if err != nil || !ok {
		return nil, false, err
	}
	audit := &MeshAudit{XVerts: make([]proof.Xpt, vertexCount), Adj: adj}
	for i, v := range verts {
		ok, err := boundedFacetCharge(budget, boundedFacetLiftWork+1)
		if err != nil || !ok {
			return nil, false, err
		}
		audit.XVerts[i] = proof.XptOf(v)
	}

	ok, err = boundedFacetCharge(budget, uint64(facetCount+2*facetCount+1))
	if err != nil || !ok {
		return nil, false, err
	}
	audit.Comp = make([]int, facetCount)
	queue := make([]int, 0, facetCount)
	list := make([]int, 0, facetCount)
	for i := range audit.Comp {
		ok, err := boundedFacetCharge(budget, 1)
		if err != nil || !ok {
			return nil, false, err
		}
		audit.Comp[i] = -1
	}
	queue = append(queue, 0)
	audit.Comp[0] = 0
	for head := 0; head < len(queue); head++ {
		ok, err := boundedFacetCharge(budget, 2)
		if err != nil || !ok {
			return nil, false, err
		}
		facet := queue[head]
		list = append(list, facet)
		for _, next := range adj[facet] {
			ok, err := boundedFacetCharge(budget, 1)
			if err != nil || !ok {
				return nil, false, err
			}
			if audit.Comp[next] == -1 {
				ok, err := boundedFacetCharge(budget, 1)
				if err != nil || !ok {
					return nil, false, err
				}
				audit.Comp[next] = 0
				queue = append(queue, next)
			}
		}
	}
	for _, component := range audit.Comp {
		ok, err := boundedFacetCharge(budget, 1)
		if err != nil || !ok {
			return nil, false, err
		}
		if component != 0 {
			return nil, false, nil
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}

	ok, err = boundedFacetCharge(budget, uint64(facetCount+1+1+2))
	if err != nil || !ok {
		return nil, false, err
	}
	audit.Members = make([][]int, 1)
	audit.Members[0] = list
	audit.Det = make([]proof.Dyadic, facetCount)
	audit.CompVol = make([]*big.Rat, 1)
	audit.Contains = make([][]bool, 1)
	audit.Contains[0] = make([]bool, 1)
	var total proof.DyAcc
	for _, facet := range list {
		ok, err := boundedFacetCharge(budget, boundedFacetDetWork+1)
		if err != nil || !ok {
			return nil, false, err
		}
		tri := tris[facet]
		cross := proof.Xcross(audit.XVerts[tri[1]], audit.XVerts[tri[2]])
		term, dyadic := proof.XdotDyadic(audit.XVerts[tri[0]], cross)
		if !dyadic {
			return nil, false, nil
		}
		audit.Det[facet] = term
		total.Add(term)
	}
	ok, err = boundedFacetCharge(budget, boundedComponentWork)
	if err != nil || !ok {
		return nil, false, err
	}
	volume := total.Rat()
	volume.Mul(volume, big.NewRat(1, 6))
	if volume.Sign() <= 0 || volume.Num().BitLen() > boundedVolumeRatBits ||
		volume.Denom().BitLen() > boundedVolumeRatBits {
		return nil, false, nil
	}
	audit.CompVol[0] = volume
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}
	return audit, true, nil
}

// VolumeFromAuditBounded reads the one-component private held audit through
// the caller's shared budget. It preserves VolumeFromAudit's exact value and
// computed bound for admitted inputs. budget must be non-nil and its ErrFn
// must report operation cancellation before wall exhaustion, as
// NewWallWorkBudgetWithOperation does. This entry point has no separate ctx.
func VolumeFromAuditBounded(budget *proofbound.WorkBudget, audit *MeshAudit,
	volSymDiff float64) (float64, float64, *big.Rat, bool, error) {
	if budget == nil {
		return 0, 0, nil, false, errors.New("bounded volume requires budget")
	}
	if ok, err := boundedFacetPoll(budget); err != nil || !ok {
		return 0, 0, nil, false, err
	}
	if audit == nil || len(audit.CompVol) != 1 || audit.CompVol[0] == nil ||
		proofbound.IsNonFinite(volSymDiff) || volSymDiff < 0 {
		return 0, 0, nil, false, nil
	}
	component := audit.CompVol[0]
	if component.Sign() <= 0 || component.Num().BitLen() > boundedVolumeRatBits ||
		component.Denom().BitLen() > boundedVolumeRatBits {
		return 0, 0, nil, false, nil
	}
	ok, err := boundedFacetCharge(budget, boundedVolumeReadWork)
	if err != nil || !ok {
		if errors.Is(err, proofbound.ErrWallWorkBudget) {
			return 0, 0, nil, false, nil
		}
		return 0, 0, nil, false, err
	}
	volumeRat := new(big.Rat).Add(new(big.Rat), component)
	volumeFloat, _ := volumeRat.Float64()
	if proofbound.IsNonFinite(volumeFloat) || volumeFloat <= 0 {
		return 0, 0, nil, false, nil
	}
	bound := proofbound.AbsSumUpper(volSymDiff, proofbound.RatAbsDiff(volumeRat, volumeFloat))
	if proofbound.IsNonFinite(bound) {
		return 0, 0, nil, false, nil
	}
	// Reaching the ceiling on this final charged operation is permitted.
	// A cancellation reported by the shared operation budget still wins.
	if stop := budget.Err(); stop != nil && !errors.Is(stop, proofbound.ErrWallWorkBudget) {
		return 0, 0, nil, false, stop
	}
	return volumeFloat, bound, volumeRat, true, nil
}
