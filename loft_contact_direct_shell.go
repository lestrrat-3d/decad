package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

var (
	errLoftDirectLimit        = errors.New("direct Loft/prism bounded limit")
	errLoftDirectProofRefused = errors.New("direct Loft/prism proof refused")
)

type loftDirectOutputLimitKey struct{}

// Only a package-private, lower-only test allowance can reduce this limit.
func loftDirectOutputLimit(ctx context.Context) uint64 {
	if limit, ok := ctx.Value(loftDirectOutputLimitKey{}).(uint64); ok &&
		limit > 0 && limit < loftVolumeOutputSteps {
		return limit
	}
	return loftVolumeOutputSteps
}

// tryLoftContactDirectShellUnion publishes only the shell certified from the
// same live prism and Loft records. A source geometry miss leaves the existing
// Boolean paths available; a certified source with a refused proof does not.
func tryLoftContactDirectShellUnion(ctx context.Context, op meshbool.OperationKind,
	d *Document, ref producerID, first, second *Body) (*Body, bool, error) {
	if op != meshbool.OpUnion {
		return nil, false, nil
	}
	prism, loft := first, second
	pp, prismOK := prism.n.Payload.(prismPayload)
	lp, loftOK := loft.n.Payload.(loftPayload)
	if !prismOK || !loftOK {
		prism, loft = second, first
		pp, prismOK = prism.n.Payload.(prismPayload)
		lp, loftOK = loft.n.Payload.(loftPayload)
		if !prismOK || !loftOK {
			return nil, false, nil
		}
	}
	operation := proofbound.NewWorkBudget(ctx)
	boundary, admitted, err := certifyLoftOutputBoundaryForDirectShell(
		ctx, first, second, operation)
	if err != nil {
		return loftContactBoundedResult(ctx, err)
	}
	if !admitted {
		return nil, false, nil
	}
	proof, admitted, err := loftVolumeCompose(ctx, operation, pp, lp, boundary)
	if err != nil {
		return loftContactBoundedResult(ctx, err)
	}
	if !admitted {
		return loftContactBoundedResult(ctx, errLoftDirectProofRefused)
	}
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}
	if proof.totalOutputVisits >= loftVolumeOutputSteps {
		return loftContactBoundedResult(ctx, errLoftDirectLimit)
	}
	budget := proofbound.NewWallWorkBudgetWithOperation(
		loftVolumeOutputSteps-proof.totalOutputVisits, operation)
	if err := budget.Err(); err != nil {
		return loftContactBoundedResult(ctx, err)
	}
	body, admitted, err := loftContactFacetedBody(ctx, d, ref, first, second,
		prism, loft, lp, proof, budget)
	if err != nil {
		return loftContactBoundedResult(ctx, err)
	}
	if !admitted {
		return loftContactBoundedResult(ctx, errLoftDirectProofRefused)
	}
	return body, true, nil
}

func loftContactFacetedBody(ctx context.Context, d *Document, ref producerID,
	first, second, prism, loft *Body, lp loftPayload,
	proof loftVolumeProof, budget *proofbound.WorkBudget) (*Body, bool, error) {
	shell := proof.boundary.shell
	if len(shell.faces) != len(shell.triangles) ||
		len(shell.vertices) == 0 || len(shell.vertices) > 512 ||
		len(shell.triangles) == 0 || len(shell.triangles) > 512 ||
		proof.boundary.heldAudit == nil ||
		len(proof.boundary.auditedVerts) != len(shell.vertices) ||
		len(proof.boundary.auditedTris) != len(shell.triangles) {
		return nil, false, nil
	}
	prismFaces, ok, err := loftContactSourceFaces(prism, budget)
	if err != nil || !ok {
		return loftContactBoundedResult(ctx, err)
	}
	loftFaces, ok, err := loftContactSourceFaces(loft, budget)
	if err != nil || !ok {
		return loftContactBoundedResult(ctx, err)
	}
	// loftPartitionLines allocates its result before its own per-line visits.
	if len(lp.Profile0.Outer.Segments) < 3 || len(lp.Profile0.Outer.Segments) > 32 ||
		len(lp.Profile1.Outer.Segments) < 3 || len(lp.Profile1.Outer.Segments) > 32 {
		return nil, false, nil
	}
	if _, err := budget.StepN(uint64(len(lp.Profile0.Outer.Segments) +
		len(lp.Profile1.Outer.Segments))); err != nil {
		return loftContactBoundedResult(ctx, err)
	}
	lower, ok, err := loftPartitionLines(budget, lp.Profile0.Outer)
	if err != nil || !ok {
		return loftContactBoundedResult(ctx, err)
	}
	upper, ok, err := loftPartitionLines(budget, lp.Profile1.Outer)
	if err != nil || !ok || len(lower) != len(upper) {
		return loftContactBoundedResult(ctx, err)
	}
	if _, err := budget.StepN(uint64(2 * len(shell.vertices))); err != nil {
		return loftContactBoundedResult(ctx, err)
	}
	if err := budget.Err(); err != nil {
		return loftContactBoundedResult(ctx, err)
	}
	verts := make([]r3.Vec, len(shell.vertices))
	beta := make([]float64, len(shell.vertices))
	for i, vertex := range shell.vertices {
		if err := budget.Step(); err != nil {
			return loftContactBoundedResult(ctx, err)
		}
		if math.Float64bits(vertex.held.X) != math.Float64bits(proof.boundary.auditedVerts[i].X) ||
			math.Float64bits(vertex.held.Y) != math.Float64bits(proof.boundary.auditedVerts[i].Y) ||
			math.Float64bits(vertex.held.Z) != math.Float64bits(proof.boundary.auditedVerts[i].Z) {
			return nil, false, nil
		}
		verts[i], beta[i] = vertex.held, vertex.bound
	}
	// Reserve the conversion's slices and lookup before any allocation.
	if _, err := budget.StepN(uint64(3 * len(shell.triangles))); err != nil {
		return loftContactBoundedResult(ctx, err)
	}
	if err := budget.Err(); err != nil {
		return loftContactBoundedResult(ctx, err)
	}
	groups := make([]facetGroup, 0, len(shell.triangles))
	groupOf := make(map[[2]*Face]int, len(shell.triangles))
	src := make([]int, len(shell.triangles))
	for i, face := range shell.faces {
		// A wall may format two distinct source roles before it looks them up.
		if _, err := budget.StepN(3); err != nil {
			return loftContactBoundedResult(ctx, err)
		}
		roles, owner, found, err := loftContactSourceRoles(face, shell.triangles[i], shell,
			lower, upper, proof.boundary.source.capLevels[0],
			proof.boundary.source.capLevels[1], budget)
		if err != nil {
			return loftContactBoundedResult(ctx, err)
		}
		if !found {
			return nil, false, nil
		}
		var sources [2]*Face
		for index, role := range roles {
			if role == "" {
				continue
			}
			if owner == 0 {
				sources[index] = prismFaces[role]
			} else {
				sources[index] = loftFaces[role]
			}
			if sources[index] == nil {
				return nil, false, nil
			}
		}
		group, exists := groupOf[sources]
		if !exists {
			group = len(groups)
			if err := budget.Step(); err != nil {
				return loftContactBoundedResult(ctx, err)
			}
			groupOf[sources] = group
			entry := facetGroup{Planar: true}
			for _, source := range sources {
				if source == nil {
					continue
				}
				if _, err := budget.StepN(uint64(len(source.n.Origins))); err != nil {
					return loftContactBoundedResult(ctx, err)
				}
				entry.Origins = append(entry.Origins, source.n.Origins...)
				entry.Planar = entry.Planar && source.isPlanar()
			}
			if err := budget.Step(); err != nil {
				return loftContactBoundedResult(ctx, err)
			}
			groups = append(groups, entry)
		}
		src[i] = group
	}
	if _, err := budget.StepN(2); err != nil {
		return loftContactBoundedResult(ctx, err)
	}
	_, dPair, err := pairChordTolerance(ctx, first, second)
	if err != nil {
		return nil, true, err
	}
	if _, err := budget.StepN(uint64(2 * len(shell.triangles))); err != nil {
		return loftContactBoundedResult(ctx, err)
	}
	if err := budget.Err(); err != nil {
		return loftContactBoundedResult(ctx, err)
	}
	tris := make([][3]int, len(shell.triangles))
	for i, triangle := range shell.triangles {
		if err := budget.Step(); err != nil {
			return loftContactBoundedResult(ctx, err)
		}
		if triangle != proof.boundary.auditedTris[i] {
			return nil, false, nil
		}
		tris[i] = triangle
	}
	payload := facetedPayload{
		Verts: verts, VertexBound: beta,
		Tris: tris,
		Src:  src, Groups: groups,
		MeshBound:  facetproof.FacetBoundMax(shell.triangles, beta),
		VolSymDiff: proof.symDiffUpper, DPair: dPair,
		Xform: r3.Identity(),
	}
	audit := proof.boundary.heldAudit
	volF, bound, volumeRat, admitted, err := facetproof.VolumeFromAuditBounded(
		budget, audit, payload.VolSymDiff)
	if err != nil || !admitted {
		return loftContactBoundedResult(ctx, err)
	}
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}
	volume := measurementToInternal(Measurement{
		Value: units.CubicMillimeters(volF), Exactness: exactnessOf(bound),
		Bound: units.CubicMillimeters(bound),
	})
	built, admitted, err := facetedrecord.BuildWithProofBounded(ctx, budget,
		envOf(documentNode(d), ref),
		payload, audit, volume, volumeRat)
	if err != nil || !admitted {
		return loftContactBoundedResult(ctx, err)
	}
	if err := ctx.Err(); err != nil {
		return nil, true, err
	}
	if err := budget.Step(); err != nil {
		return loftContactBoundedResult(ctx, err)
	}
	body, err := bodyResult(built, nil)
	if ctxErr := ctx.Err(); ctxErr != nil {
		return nil, false, ctxErr
	}
	return body, true, err
}

// loftContactBoundedResult stops this private route on a bounded refusal.
// Cancellation and ordinary validation errors keep their original result.
func loftContactBoundedResult(ctx context.Context, err error) (*Body, bool, error) {
	if ctxErr := ctx.Err(); ctxErr != nil {
		return nil, false, ctxErr
	}
	if errors.Is(err, proofbound.ErrWallWorkBudget) || errors.Is(err, errLoftDirectLimit) {
		return nil, false, fmt.Errorf("%w: direct Loft/prism bounded proof limit", ErrUnsupported)
	}
	if errors.Is(err, errLoftDirectProofRefused) {
		return nil, false, fmt.Errorf("%w: direct Loft/prism proof refused after source certification", ErrUnsupported)
	}
	return nil, false, err
}

func loftContactSourceFaces(body *Body, budget *proofbound.WorkBudget) (map[string]*Face, bool, error) {
	var roles int
	for _, lump := range body.n.Lumps {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		for _, shell := range lump.Shells {
			if err := budget.Step(); err != nil {
				return nil, false, err
			}
			for _, face := range shell.Faces {
				if err := budget.Step(); err != nil {
					return nil, false, err
				}
				if len(face.Origins) > loftVolumeOutputSteps ||
					roles > loftVolumeOutputSteps-len(face.Origins) {
					return nil, false, nil
				}
				roles += len(face.Origins)
				if _, err := budget.StepN(uint64(len(face.Origins))); err != nil {
					return nil, false, err
				}
			}
		}
	}
	// body.Faces and Face.originRefs both copy data. Walk the born nodes
	// directly after charging for the map and its source-role entries.
	if err := budget.Err(); err != nil {
		return nil, false, err
	}
	if err := budget.Step(); err != nil {
		return nil, false, err
	}
	faces := make(map[string]*Face, roles)
	for _, lump := range body.n.Lumps {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		for _, shell := range lump.Shells {
			if err := budget.Step(); err != nil {
				return nil, false, err
			}
			for _, node := range shell.Faces {
				face := faceHandle(node)
				for _, origin := range node.Origins {
					if err := budget.Step(); err != nil {
						return nil, false, err
					}
					if origin.Producer != body.n.Origin.Producer {
						continue
					}
					if previous := faces[origin.Role]; previous != nil && previous != face {
						return nil, false, nil
					}
					faces[origin.Role] = face
				}
			}
		}
	}
	return faces, true, nil
}

func loftContactSourceRoles(face loftOutputFace, triangle [3]int,
	shell loftOutputShell, lower, upper []sectionrecord.LineSeg, z0, z1 *big.Rat,
	budget *proofbound.WorkBudget) ([2]string, int, bool, error) {
	if face.kind == 0 {
		switch {
		case face.owner == 0 && face.level == 0, face.owner == 1 && face.level == 0:
			return [2]string{provenance.RoleCapStart}, face.owner, true, nil
		case face.owner == 1 && face.level == 1, face.owner == 0 && face.level == 2:
			return [2]string{provenance.RoleCapEnd}, face.owner, true, nil
		}
		return [2]string{}, 0, false, nil
	}
	if face.kind != 1 || face.segment < 0 {
		return [2]string{}, 0, false, nil
	}
	if face.owner == 0 {
		return [2]string{fmt.Sprintf("side(0,%d)", face.segment)}, 0, true, nil
	}
	if face.owner != 1 || face.segment >= len(lower) || face.segment >= len(upper) {
		return [2]string{}, 0, false, nil
	}
	sides, ok, err := loftContactWallTriangleSides(triangle, shell, lower[face.segment],
		upper[face.segment], z0, z1, budget)
	if !ok {
		return [2]string{}, 0, false, err
	}
	var roles [2]string
	for side := range 2 {
		if sides&(1<<side) != 0 {
			roles[side] = fmt.Sprintf("side(0,%d,%d)", face.segment, side)
		}
	}
	if roles[0] == "" {
		roles[0], roles[1] = roles[1], ""
	}
	return roles, 1, true, nil
}

// The original Loft gives each wall cell two faces. An output triangle that
// crosses their exact diagonal carries both contributing source roles.
func loftContactWallTriangleSides(triangle [3]int, shell loftOutputShell,
	lower, upper sectionrecord.LineSeg, z0, z1 *big.Rat,
	budget *proofbound.WorkBudget) (uint8, bool, error) {
	// The height and four source corners create nine rationals before the
	// per-vertex visits below. Reserve those operations before allocating.
	if _, err := budget.StepN(9); err != nil {
		return 0, false, err
	}
	c := &loftcontact.Arithmetic{OK: true}
	height := c.Sub(z1, z0)
	if !c.OK || height.Sign() <= 0 {
		return 0, false, nil
	}
	lowStart, ok0 := loftcontact.PointOf(lower.Start)
	lowEnd, ok1 := loftcontact.PointOf(lower.End)
	highStart, ok2 := loftcontact.PointOf(upper.Start)
	highEnd, ok3 := loftcontact.PointOf(upper.End)
	if !ok0 || !ok1 || !ok2 || !ok3 {
		return 0, false, nil
	}
	positive, negative := false, false
	for _, id := range triangle {
		// Each point can perform at most 45 rational operations: four
		// two-coordinate interpolations (24), parameter arithmetic (6),
		// four constant comparisons (8), equality (2), and the diagonal
		// side expression (5). Reserve those and the vertex read before any
		// operation allocates its result.
		if _, err := budget.StepN(46); err != nil {
			return 0, false, err
		}
		if id < 0 || id >= len(shell.vertices) {
			return 0, false, nil
		}
		point := shell.vertices[id].point
		s := c.Quo(c.Sub(point.z, z0), height)
		start := c.Interpolate(lowStart, highStart, s)
		end := c.Interpolate(lowEnd, highEnd, s)
		du, dv := c.Sub(end.U, start.U), c.Sub(end.V, start.V)
		var t *big.Rat
		switch {
		case du.Sign() != 0:
			t = c.Quo(c.Sub(point.u, start.U), du)
		case dv.Sign() != 0:
			t = c.Quo(c.Sub(point.v, start.V), dv)
		default:
			return 0, false, nil
		}
		reconstructed := c.Interpolate(start, end, t)
		if !c.OK || c.Cmp(s, loftOutputRat(0)) < 0 ||
			c.Cmp(s, loftOutputRat(1)) > 0 || c.Cmp(t, loftOutputRat(0)) < 0 ||
			c.Cmp(t, loftOutputRat(1)) > 0 ||
			!c.SamePoint(loftcontact.Point{U: point.u, V: point.v}, reconstructed) {
			return 0, false, nil
		}
		diagonal := c.Interpolate(lowStart, highEnd, s)
		// The admitted wall is planar and convex. Its horizontal sections
		// stay parallel, so this sign is the side of the original straight
		// source diagonal throughout each output triangle.
		parallel := c.Add(c.Mul(c.Sub(point.u, diagonal.U), du),
			c.Mul(c.Sub(point.v, diagonal.V), dv))
		if !c.OK {
			return 0, false, nil
		}
		sign := parallel.Sign()
		positive = positive || sign > 0
		negative = negative || sign < 0
	}
	if !c.OK || !positive && !negative {
		return 0, false, nil
	}
	var sides uint8
	if positive {
		sides |= 1
	}
	if negative {
		sides |= 2
	}
	return sides, true, nil
}
