package facetedrecord

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

var errBoundedArithmetic = errors.New("bounded faceted arithmetic refused")

// AssembleBodyBounded is the private direct-shell form of AssembleBody. The
// caller supplies its already-audited mesh and the same budget used upstream.
// A false result leaves the candidate unpublished; ordinary build errors
// retain their original error value.
func AssembleBodyBounded(ctx context.Context, budget *proofbound.WorkBudget, env topo.Env, in Mesh,
	audit *facetproof.MeshAudit, volume measurement.Measurement, volRat *big.Rat,
) (*topo.Body, []int, float64, bool, error) {
	if err := ctx.Err(); err != nil {
		return nil, nil, 0, false, err
	}
	if budget == nil || audit == nil || len(in.Tris) > 512 || len(in.Verts) > 3*len(in.Tris) ||
		len(audit.Members) > len(in.Tris) {
		return nil, nil, 0, false, nil
	}
	// Charge every input and audit entry before walking it below. The 8F
	// term covers facets, source groups, determinant records, three corners,
	// three adjacencies, and component members. The extra F reserves the
	// membership bitmap; C and 2C² cover length and containment checks.
	if _, err := budget.StepN(uint64(len(in.Verts) + 9*len(in.Tris) +
		len(audit.Members) + 2*len(audit.Members)*len(audit.Members))); err != nil {
		return nil, nil, 0, false, boundedBuildError(ctx, err)
	}
	memberCount := 0
	for _, members := range audit.Members {
		if len(members) > len(in.Tris)-memberCount {
			return nil, nil, 0, false, nil
		}
		memberCount += len(members)
	}
	if memberCount != len(in.Tris) {
		return nil, nil, 0, false, nil
	}
	if !boundedFacetedInput(in, audit, volRat) {
		return nil, nil, 0, false, nil
	}
	originCopies, ok := boundedOriginCopies(in)
	if !ok {
		return nil, nil, 0, false, nil
	}
	if err := reserveBoundedAssembly(budget, len(in.Verts), len(in.Tris), len(audit.Members), originCopies); err != nil {
		return nil, nil, 0, false, boundedBuildError(ctx, err)
	}
	for _, p := range audit.XVerts {
		if p.X == nil || p.Y == nil || p.Z == nil || p.W == nil || p.W.Sign() <= 0 ||
			p.X.BitLen() > 512 || p.Y.BitLen() > 512 || p.Z.BitLen() > 512 || p.W.BitLen() > 512 ||
			p.W.BitLen() != int(p.W.TrailingZeroBits())+1 {
			return nil, nil, 0, false, nil
		}
	}
	for _, d := range audit.Det {
		if d.Exp() < -512 || d.Exp() > 512 {
			return nil, nil, 0, false, nil
		}
		if mantissa := d.Mant(); mantissa != nil && mantissa.BitLen() > 512 {
			return nil, nil, 0, false, nil
		}
	}
	if err := budget.Err(); err != nil {
		return nil, nil, 0, false, boundedBuildError(ctx, err)
	}
	body, faceOf, diameter, err := assembleBody(ctx, budget, env, in, audit, volume, volRat)
	if err != nil {
		if (errors.Is(err, proofbound.ErrWallWorkBudget) || errors.Is(err, errBoundedArithmetic)) && ctx.Err() == nil {
			return nil, nil, 0, false, nil
		}
		return nil, nil, 0, false, err
	}
	return body, faceOf, diameter, true, nil
}

func boundedBuildError(ctx context.Context, err error) error {
	if cancellation := ctx.Err(); cancellation != nil {
		return cancellation
	}
	if errors.Is(err, proofbound.ErrWallWorkBudget) {
		return nil
	}
	return err
}

// boundedFacetedInput refuses a source too large for the private 512-facet
// route before any dynamic builder allocation. Finite coordinates and bounds
// are restricted to exponents [-128,128]. Combined with the 512-bit audited
// integers below, 512 terms, and a 512-bit volume, even unreduced moment and
// centroid products stay below the route's 8,192-bit arithmetic ceiling.
func boundedFacetedInput(in Mesh, audit *facetproof.MeshAudit, volRat *big.Rat) bool {
	v, f := len(in.Verts), len(in.Tris)
	if v == 0 || f == 0 || f > 512 || v > 3*f ||
		len(in.VertexBound) != v || len(in.Src) != f || audit == nil || volRat == nil ||
		len(audit.XVerts) != v || len(audit.Det) != f || len(audit.Comp) != f || len(audit.Adj) != f ||
		len(audit.Members) == 0 || len(audit.Members) > f ||
		len(audit.CompVol) != len(audit.Members) || len(audit.Contains) != len(audit.Members) ||
		volRat.Sign() <= 0 || volRat.Num().BitLen() > 512 || volRat.Denom().BitLen() > 512 ||
		!boundedFloat(in.VolSymDiff) || in.VolSymDiff < 0 ||
		!boundedFloat(in.AreaSlack) || in.AreaSlack < 0 ||
		!boundedFloat(in.DPair) || in.DPair < 0 {
		return false
	}
	for i, p := range in.Verts {
		if !boundedFloat(p.X) || !boundedFloat(p.Y) || !boundedFloat(p.Z) ||
			!boundedFloat(in.VertexBound[i]) || in.VertexBound[i] < 0 {
			return false
		}
	}
	for _, t := range in.Tris {
		for _, index := range t {
			if index < 0 || index >= v {
				return false
			}
		}
	}
	for _, cv := range audit.CompVol {
		if cv == nil || cv.Num().BitLen() > 512 || cv.Denom().BitLen() > 512 {
			return false
		}
	}
	for _, row := range audit.Contains {
		if len(row) != len(audit.Members) {
			return false
		}
	}
	for _, component := range audit.Comp {
		if component < 0 || component >= len(audit.Members) {
			return false
		}
	}
	for _, adjacent := range audit.Adj {
		if len(adjacent) != 3 {
			return false
		}
		for _, facet := range adjacent {
			if facet < 0 || facet >= f {
				return false
			}
		}
	}
	seen := make([]bool, f) // Its capacity was charged before this input walk.
	for component, members := range audit.Members {
		for _, facet := range members {
			if facet < 0 || facet >= f || seen[facet] || audit.Comp[facet] != component {
				return false
			}
			seen[facet] = true
		}
	}
	return true
}

func boundedFloat(v float64) bool {
	if math.IsNaN(v) || math.IsInf(v, 0) {
		return false
	}
	if v == 0 {
		return true
	}
	_, exp := math.Frexp(v)
	return exp >= -128 && exp <= 128
}

// Every new face copies its group's origin slice. A source group can split
// into at most one patch per facet bearing that group, so this per-facet sum
// charges an upper bound before the first topological face is allocated.
func boundedOriginCopies(in Mesh) (uint64, bool) {
	var copies uint64
	for _, group := range in.Src {
		if group < 0 || group >= len(in.Groups) {
			return 0, false
		}
		var ok bool
		copies, ok = proofbound.WallCheckedAdd(copies,
			uint64(len(in.Groups[group].Origins))+uint64(len(in.Groups[group].Upstream)))
		if !ok {
			return 0, false
		}
	}
	return copies, true
}

// reserveBoundedAssembly charges an upper envelope of logical container
// slots before diameter, face, topology, and measurement allocations. A
// closed F-facet mesh has 3F directed halfedges and at most 3F/2 undirected
// edges. Its face patches, chains, and loops each number at most F, 3F/2,
// and 3F. The 96F envelope assigns 36F to the topology walk's maps and
// slices, 30F to face/patch/perimeter storage, 14F to topology nodes and
// coedge copies, and 16F to exact-moment and result temporaries. The 32V
// envelope covers diameter blocks, lifted vertices, per-vertex bounds,
// topology vertices, and exact extreme-bound temporaries. The separate 8C²
// term covers containment records for C components. Origins are charged by
// the sum of each facet's source group's origins, which is an upper bound on
// every patch-face copy.
func reserveBoundedAssembly(budget *proofbound.WorkBudget, verts, facets, components int, origins uint64) error {
	n := uint64(256 + 96*facets + 32*verts + 8*components*components)
	var ok bool
	n, ok = proofbound.WallCheckedAdd(n, origins)
	if !ok {
		return proofbound.ErrWallWorkBudget
	}
	_, err := budget.StepN(n)
	return err
}

// faceIndicesBounded uses the caller's counter for the final face-order map.
// Storage has already been included in reserveBoundedAssembly.
func faceIndicesBounded(ctx context.Context, budget *proofbound.WorkBudget,
	faces, facetFace []*topo.Face) ([]int, error) {
	flat := make(map[*topo.Face]int, len(faces))
	for i, f := range faces {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		flat[f] = i
	}
	out := make([]int, len(facetFace))
	for i, f := range facetFace {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		index, ok := flat[f]
		if !ok {
			return nil, fmt.Errorf(`%w: a facet maps to a face absent from the built body`, decaderr.ErrBooleanFailed)
		}
		out[i] = index
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return out, budget.Err()
}
