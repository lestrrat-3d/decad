package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// loftOutputBoundaryProof is private, transient and has no occupied-volume or
// measurement claim. The source records must remain live throughout its call.
type loftOutputBoundaryProof struct {
	source       loftSourcePartition
	shell        loftOutputShell
	workVisits   uint64
	heldAudit    *facetproof.MeshAudit
	auditedVerts []r3.Vec
	auditedTris  [][3]int
}

// The private proof keeps its existing audit behavior. The direct-shell
// mode charges and retains the held audit for the result builder.
func certifyLoftOutputBoundary(ctx context.Context, first, second *Body,
	operation *proofbound.WorkBudget) (loftOutputBoundaryProof, bool, error) {
	return certifyLoftOutputBoundaryMode(ctx, first, second, operation, false, nil)
}

func certifyLoftOutputBoundaryForDirectShell(ctx context.Context, first, second *Body,
	operation *proofbound.WorkBudget) (loftOutputBoundaryProof, bool, error) {
	var sourceCertified bool
	proof, admitted, err := certifyLoftOutputBoundaryMode(ctx, first, second,
		operation, true, &sourceCertified)
	if err == nil && !admitted && sourceCertified {
		return loftOutputBoundaryProof{}, false, errLoftDirectProofRefused
	}
	return proof, admitted, err
}

func certifyLoftOutputBoundaryMode(ctx context.Context, first, second *Body,
	operation *proofbound.WorkBudget, directShell bool,
	sourceCertified *bool) (loftOutputBoundaryProof, bool, error) {
	var no loftOutputBoundaryProof
	arithmetic := &loftVolumeArithmetic{ok: true}
	if err := ctx.Err(); err != nil {
		return no, false, err
	}
	if first == nil || second == nil || operation == nil {
		return no, false, nil
	}
	var source loftSourcePartition
	var ok bool
	var err error
	if directShell {
		source, ok, err = certifyLoftSourcePartitionForDirectShell(
			ctx, first, second, operation, arithmetic)
	} else {
		source, ok, err = certifyLoftSourcePartition(
			ctx, first, second, operation, arithmetic)
	}
	if err != nil {
		return no, false, err
	}
	if !arithmetic.ok {
		if directShell {
			return no, false, errLoftDirectLimit
		}
		return no, false, nil
	}
	if !ok {
		return no, false, nil
	}
	if sourceCertified != nil {
		*sourceCertified = true
	}
	var pp prismPayload
	var lp loftPayload
	if p, yes := first.n.Payload.(prismPayload); yes {
		pp = p
		lp, ok = second.n.Payload.(loftPayload)
	} else {
		lp, ok = first.n.Payload.(loftPayload)
		if !ok {
			return no, false, nil
		}
		pp, ok = second.n.Payload.(prismPayload)
	}
	if !ok {
		return no, false, nil
	}
	outputLimit := uint64(65_536)
	if directShell {
		outputLimit = loftDirectOutputLimit(ctx)
	}
	limit := proofbound.NewWallWorkBudgetWithOperation(outputLimit, operation)
	var visits uint64
	budget := &proofbound.WorkBudget{
		StepFn: func() error {
			if err := limit.Step(); err != nil {
				return err
			}
			visits++
			return nil
		},
		ErrFn: limit.Err,
	}
	prismLines, ok, err := loftPartitionLines(budget, pp.Profile.Outer)
	if err != nil || !ok {
		return no, false, err
	}
	bottom, ok, err := loftPartitionLines(budget, lp.Profile0.Outer)
	if err != nil || !ok {
		return no, false, err
	}
	top, ok, err := loftPartitionLines(budget, lp.Profile1.Outer)
	if err != nil || !ok || len(top) != len(bottom) {
		return no, false, err
	}
	shell := loftOutputNewShell()
	for cap := range 3 {
		ok, err = loftOutputCap(ctx, budget, pp, prismLines, bottom, top, source, &shell, cap, arithmetic)
		if err != nil || !ok || !arithmetic.ok {
			return no, false, err
		}
	}
	ok, err = loftOutputSides(budget, pp, prismLines, bottom, top, source, &shell, arithmetic)
	if err != nil || !ok || !arithmetic.ok {
		return no, false, err
	}
	ok, err = loftOutputAuditSource(budget, shell, source, arithmetic)
	if err != nil || !ok || !arithmetic.ok {
		return no, false, err
	}
	if err := budget.Err(); err != nil {
		return no, false, err
	}
	if directShell {
		if _, err := budget.StepN(uint64(len(shell.vertices) + len(shell.triangles))); err != nil {
			return no, false, err
		}
		if err := budget.Err(); err != nil {
			return no, false, err
		}
	}
	verts := make([]r3.Vec, len(shell.vertices))
	for i, v := range shell.vertices {
		if err := budget.Step(); err != nil {
			return no, false, err
		}
		verts[i] = v.held
	}
	var auditedTris [][3]int
	if directShell {
		auditedTris = append([][3]int(nil), shell.triangles...)
	}
	// The exact source labels and orientation are already fixed. Neither audit
	// may reverse a triangle to repair an ownership mistake.
	if directShell {
		if err := loftmesh.LoftCrossingAuditPrivateBounded(budget, verts, shell.triangles); err != nil {
			return no, false, err
		}
	} else {
		if err := loftmesh.LoftCrossingAuditBounded(operation, verts, shell.triangles,
			512, 1_000_000); err != nil {
			return no, false, err
		}
	}
	var audit *facetproof.MeshAudit
	if directShell {
		var audited bool
		audit, audited, err = facetproof.AuditFacetedMeshBounded(ctx, budget, verts, auditedTris)
		if err != nil || !audited {
			return no, false, err
		}
	} else {
		audit, err = facetproof.AuditFacetedMesh(ctx, verts, shell.triangles)
		if err != nil {
			return no, false, err
		}
	}
	if audit == nil || len(audit.Members) != 1 {
		return no, false, nil
	}
	if err := budget.Err(); err != nil {
		return no, false, err
	}
	if !arithmetic.ok {
		return no, false, nil
	}
	out := loftOutputBoundaryProof{source: source, shell: shell, workVisits: visits}
	if directShell {
		out.heldAudit, out.auditedVerts, out.auditedTris = audit, verts, auditedTris
	}
	return out, true, nil
}

func loftOutputSides(budget *proofbound.WorkBudget, pp prismPayload,
	prismLines, bottom, top []sectionrecord.LineSeg, source loftSourcePartition,
	shell *loftOutputShell, checked ...*loftVolumeArithmetic) (bool, error) {
	arithmetic := loftReplayArithmetic(checked)
	z0, zL, zP := source.capLevels[0], source.capLevels[1], source.capLevels[2]
	var lineCut, carrierCut [2][2]*big.Rat
	for k, seam := range source.seams {
		for t := range 2 {
			if err := budget.Step(); err != nil {
				return false, err
			}
			var ok bool
			lineCut[t][k], carrierCut[t][k], ok = seam.At(loftOutputRat(int64(t)))
			if !ok || !arithmetic.valid(lineCut[t][k]) || !arithmetic.valid(carrierCut[t][k]) {
				return false, nil
			}
		}
		mid, midCarrier, ok := seam.At(big.NewRat(1, 2))
		if !ok || !arithmetic.valid(mid) || !arithmetic.valid(midCarrier) {
			return false, nil
		}
		moving := source.crossing[k]
		midStart, okStart := loftOutputMovingPoint(bottom[moving].Start, top[moving].Start,
			big.NewRat(1, 2))
		midEnd, okEnd := loftOutputMovingPoint(bottom[moving].End, top[moving].End,
			big.NewRat(1, 2))
		carrierPoint := arithmetic.partitionLinePoint(prismLines[source.carrier], midCarrier)
		interpolated := arithmetic.partitionInterpolate(midStart, midEnd, mid)
		if !okStart || !okEnd || !arithmetic.partitionSamePoint(interpolated, carrierPoint) || !arithmetic.ok {
			return false, nil
		}
		linear := arithmetic.quo(arithmetic.add(carrierCut[0][k], carrierCut[1][k]), loftOutputRat(2))
		if arithmetic.cmp(midCarrier, linear) != 0 || !arithmetic.ok {
			return false, nil
		}
		// CertifyLineSeam establishes unique transverse intersections over the
		// interval. Its carrier numerator is quadratic and denominator linear;
		// agreement at 0, 1/2 and 1 with this line is a polynomial identity.
	}
	for i, line := range prismLines {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if i != source.carrier {
			for band, levels := range [][2]*big.Rat{{z0, zL}, {zL, zP}} {
				ok, err := loftOutputLineQuad(budget, pp.Frame, shell, line, line,
					loftOutputRat(0), loftOutputRat(1), loftOutputRat(0), loftOutputRat(1),
					levels[0], levels[1], loftOutputFace{kind: 1, owner: 0, segment: i, level: band})
				if err != nil || !ok {
					return false, err
				}
			}
			continue
		}
		for part, bounds := range [][4]*big.Rat{
			{loftOutputRat(0), carrierCut[0][0], loftOutputRat(0), carrierCut[1][0]},
			{carrierCut[0][1], loftOutputRat(1), carrierCut[1][1], loftOutputRat(1)},
		} {
			ok, err := loftOutputLineQuad(budget, pp.Frame, shell, line, line,
				bounds[0], bounds[1], bounds[2], bounds[3], z0, zL,
				loftOutputFace{kind: 1, owner: 0, segment: i, level: part})
			if err != nil || !ok {
				return false, err
			}
		}
		breaks := [4]*big.Rat{loftOutputRat(0), carrierCut[1][0], carrierCut[1][1], loftOutputRat(1)}
		for part := range 3 {
			ok, err := loftOutputLineQuad(budget, pp.Frame, shell, line, line,
				breaks[part], breaks[part+1], breaks[part], breaks[part+1], zL, zP,
				loftOutputFace{kind: 1, owner: 0, segment: i, level: part + 2})
			if err != nil || !ok {
				return false, err
			}
		}
	}
	for i := range bottom {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if source.walls[i] == -1 {
			continue
		}
		if !loftOutputPlanarWall(bottom[i], top[i], zL) {
			return false, nil
		}
		lo, hi := loftOutputRat(0), loftOutputRat(1)
		upperLo, upperHi := loftOutputRat(0), loftOutputRat(1)
		switch source.walls[i] {
		case 0:
			k := 0
			if source.crossing[1] == i {
				k = 1
			} else if source.crossing[0] != i {
				return false, nil
			}
			side := arithmetic.partitionSide(prismLines[source.carrier], bottom[i].Start)
			if !arithmetic.ok || side.Sign() == 0 {
				return false, nil
			}
			if side.Sign() < 0 {
				hi, upperHi = lineCut[0][k], lineCut[1][k]
			} else {
				lo, upperLo = lineCut[0][k], lineCut[1][k]
			}
		case 1:
		default:
			return false, nil
		}
		ok, err := loftOutputLineQuad(budget, pp.Frame, shell, bottom[i], top[i],
			lo, hi, upperLo, upperHi, z0, zL,
			loftOutputFace{kind: 1, owner: 1, segment: i})
		if err != nil || !ok {
			return false, err
		}
	}
	return arithmetic.ok, nil
}

func loftOutputLineQuad(budget *proofbound.WorkBudget, frame r3.Frame,
	shell *loftOutputShell, low, high sectionrecord.LineSeg,
	lo0, hi0, lo1, hi1, z0, z1 *big.Rat, face loftOutputFace) (bool, error) {
	if err := budget.Step(); err != nil {
		return false, err
	}
	arithmetic := &loftVolumeArithmetic{ok: true}
	if arithmetic.cmp(lo0, hi0) >= 0 || arithmetic.cmp(lo1, hi1) >= 0 ||
		arithmetic.cmp(z0, z1) >= 0 || !arithmetic.ok {
		return false, nil
	}
	params := [4]*big.Rat{lo0, hi0, hi1, lo1}
	lines := [4]sectionrecord.LineSeg{low, low, high, high}
	levels := [4]*big.Rat{z0, z0, z1, z1}
	var quad [4]loftOutputPoint
	for i := range quad {
		p := arithmetic.partitionLinePoint(lines[i], params[i])
		if !arithmetic.ok {
			return false, nil
		}
		quad[i] = loftOutputP2(p, levels[i])
	}
	if !loftOutputQuadPlanarConvex(quad) {
		return false, nil
	}
	for _, tri := range [][3]int{{0, 1, 2}, {0, 2, 3}} {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if !shell.triangle(frame,
			[3]loftOutputPoint{quad[tri[0]], quad[tri[1]], quad[tri[2]]}, face) {
			return false, nil
		}
	}
	return true, nil
}

func loftOutputDet3(arithmetic *loftVolumeArithmetic, a, b, c [3]*big.Rat) *big.Rat {
	x := arithmetic.sub(arithmetic.mul(b[1], c[2]), arithmetic.mul(b[2], c[1]))
	y := arithmetic.sub(arithmetic.mul(b[2], c[0]), arithmetic.mul(b[0], c[2]))
	z := arithmetic.sub(arithmetic.mul(b[0], c[1]), arithmetic.mul(b[1], c[0]))
	return arithmetic.add(arithmetic.add(arithmetic.mul(a[0], x),
		arithmetic.mul(a[1], y)), arithmetic.mul(a[2], z))
}

func loftOutputCrossVec(arithmetic *loftVolumeArithmetic, a, b [3]*big.Rat) [3]*big.Rat {
	var c [3]*big.Rat
	for i := range c {
		j, k := (i+1)%3, (i+2)%3
		c[i] = arithmetic.sub(arithmetic.mul(a[j], b[k]), arithmetic.mul(a[k], b[j]))
	}
	return c
}

func loftOutputWorldDelta(arithmetic *loftVolumeArithmetic, a, b [3]*big.Rat) [3]*big.Rat {
	var d [3]*big.Rat
	for i := range d {
		d[i] = arithmetic.sub(b[i], a[i])
	}
	return d
}

func loftOutputHeldExact(v r3.Vec) [3]*big.Rat {
	return [3]*big.Rat{proofarith.FloatRat(v.X), proofarith.FloatRat(v.Y), proofarith.FloatRat(v.Z)}
}

func loftOutputNormalsAgree(a, b, c loftOutputVertex) bool {
	arithmetic := &loftVolumeArithmetic{ok: true}
	source := loftOutputCrossVec(arithmetic, loftOutputWorldDelta(arithmetic, a.world, b.world),
		loftOutputWorldDelta(arithmetic, a.world, c.world))
	ha, hb, hc := loftOutputHeldExact(a.held), loftOutputHeldExact(b.held), loftOutputHeldExact(c.held)
	held := loftOutputCrossVec(arithmetic, loftOutputWorldDelta(arithmetic, ha, hb),
		loftOutputWorldDelta(arithmetic, ha, hc))
	dot := new(big.Rat)
	for i := range source {
		dot = arithmetic.add(dot, arithmetic.mul(source[i], held[i]))
	}
	return arithmetic.ok && dot.Sign() > 0
}

func loftOutputMovingPoint(low, high sectionrecord.Point2, t *big.Rat) (loftPartitionPoint, bool) {
	a, okA := loftPartitionPointOf(low)
	b, okB := loftPartitionPointOf(high)
	if !okA || !okB {
		return loftPartitionPoint{}, false
	}
	arithmetic := &loftVolumeArithmetic{ok: true}
	point := loftPartitionPoint{
		u: arithmetic.add(a.u, arithmetic.mul(t, arithmetic.sub(b.u, a.u))),
		v: arithmetic.add(a.v, arithmetic.mul(t, arithmetic.sub(b.v, a.v))),
	}
	return point, arithmetic.ok
}

func loftOutputDelta(arithmetic *loftVolumeArithmetic, a, b loftOutputPoint) [3]*big.Rat {
	return [3]*big.Rat{arithmetic.sub(b.u, a.u), arithmetic.sub(b.v, a.v),
		arithmetic.sub(b.z, a.z)}
}

func loftOutputPlanarWall(low, high sectionrecord.LineSeg, z *big.Rat) bool {
	a, okA := loftPartitionPointOf(low.Start)
	b, okB := loftPartitionPointOf(low.End)
	c, okC := loftPartitionPointOf(high.Start)
	d, okD := loftPartitionPointOf(high.End)
	if !okA || !okB || !okC || !okD {
		return false
	}
	p, q, r, s := loftOutputP2(a, loftOutputRat(0)), loftOutputP2(b, loftOutputRat(0)),
		loftOutputP2(c, z), loftOutputP2(d, z)
	arithmetic := &loftVolumeArithmetic{ok: true}
	det := loftOutputDet3(arithmetic, loftOutputDelta(arithmetic, p, q),
		loftOutputDelta(arithmetic, p, r), loftOutputDelta(arithmetic, p, s))
	return arithmetic.ok && det.Sign() == 0
}

func loftOutputQuadPlanarConvex(q [4]loftOutputPoint) bool {
	arithmetic := &loftVolumeArithmetic{ok: true}
	if loftOutputDet3(arithmetic, loftOutputDelta(arithmetic, q[0], q[1]),
		loftOutputDelta(arithmetic, q[0], q[2]),
		loftOutputDelta(arithmetic, q[0], q[3])).Sign() != 0 || !arithmetic.ok ||
		loftOutputCrossZero([3]loftOutputPoint{q[0], q[1], q[2]}) ||
		loftOutputCrossZero([3]loftOutputPoint{q[0], q[2], q[3]}) {
		return false
	}
	n0 := loftOutputCrossVec(arithmetic, loftOutputDelta(arithmetic, q[0], q[1]),
		loftOutputDelta(arithmetic, q[0], q[2]))
	n1 := loftOutputCrossVec(arithmetic, loftOutputDelta(arithmetic, q[0], q[2]),
		loftOutputDelta(arithmetic, q[0], q[3]))
	dot := new(big.Rat)
	for i := range n0 {
		dot = arithmetic.add(dot, arithmetic.mul(n0[i], n1[i]))
	}
	// Monotone height and ordered edge parameters then make this a simple
	// planar quadrilateral; matching normals establish both triangle sides.
	return arithmetic.ok && dot.Sign() > 0
}

func loftOutputAuditSource(budget *proofbound.WorkBudget,
	shell loftOutputShell, source loftSourcePartition, checked ...*loftVolumeArithmetic) (bool, error) {
	arithmetic := loftReplayArithmetic(checked)
	if len(shell.triangles) == 0 || len(shell.triangles) > 512 ||
		len(shell.vertices) == 0 || len(shell.vertices) > 512 ||
		len(shell.faces) != len(shell.triangles) ||
		len(shell.triangleBounds) != len(shell.triangles) {
		return false, nil
	}
	directed := make(map[[2]int]int, 3*len(shell.triangles))
	incident := make([][]int, len(shell.vertices))
	incidences := make([]map[loftOutputFace]struct{}, len(shell.vertices))
	for fi, tri := range shell.triangles {
		if err := budget.Step(); err != nil {
			return false, err
		}
		for k, vertex := range tri {
			if vertex < 0 || vertex >= len(shell.vertices) {
				return false, nil
			}
			incident[vertex] = append(incident[vertex], fi)
			if incidences[vertex] == nil {
				incidences[vertex] = make(map[loftOutputFace]struct{})
			}
			incidences[vertex][shell.faces[fi]] = struct{}{}
			edge := [2]int{vertex, tri[(k+1)%3]}
			if _, duplicate := directed[edge]; duplicate {
				return false, nil
			}
			directed[edge] = fi
		}
		if shell.triangleBounds[fi] < 0 || proofbound.IsNonFinite(shell.triangleBounds[fi]) ||
			shell.triangleBounds[fi] != max(shell.vertices[tri[0]].bound,
				shell.vertices[tri[1]].bound, shell.vertices[tri[2]].bound) {
			return false, nil
		}
		if !loftOutputNormalsAgree(shell.vertices[tri[0]],
			shell.vertices[tri[1]], shell.vertices[tri[2]]) {
			return false, nil
		}
	}
	for edge, fi := range directed {
		if err := budget.Step(); err != nil {
			return false, err
		}
		peer, found := directed[[2]int{edge[1], edge[0]}]
		if !found || !loftOutputEdgeOwnersCompatible(shell, source, fi, peer, edge, arithmetic) ||
			!arithmetic.ok {
			return false, nil
		}
	}
	for vertex, faces := range incident {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if len(faces) < 3 || len(shell.vertices[vertex].incidences) != len(incidences[vertex]) {
			return false, nil
		}
		for face := range incidences[vertex] {
			if err := budget.Step(); err != nil {
				return false, err
			}
			if _, found := shell.vertices[vertex].incidences[face]; !found {
				return false, nil
			}
		}
		seen := map[int]struct{}{faces[0]: {}}
		queue := []int{faces[0]}
		for len(queue) != 0 {
			fi := queue[0]
			queue = queue[1:]
			tri := shell.triangles[fi]
			for k, v := range tri {
				if v != vertex {
					continue
				}
				for _, edge := range [][2]int{{vertex, tri[(k+1)%3]}, {tri[(k+2)%3], vertex}} {
					if err := budget.Step(); err != nil {
						return false, err
					}
					peer := directed[[2]int{edge[1], edge[0]}]
					if _, found := seen[peer]; !found {
						seen[peer] = struct{}{}
						queue = append(queue, peer)
					}
				}
			}
		}
		if len(seen) != len(faces) {
			return false, nil
		}
	}
	for i, a := range shell.triangles {
		if shell.faces[i].kind != 0 {
			continue
		}
		for j := range i {
			if err := budget.Step(); err != nil {
				return false, err
			}
			if shell.faces[j].kind != 0 || shell.faces[j].level != shell.faces[i].level {
				continue
			}
			p, q := [3]loftPartitionPoint{}, [3]loftPartitionPoint{}
			for k := range 3 {
				p[k] = loftPartitionPoint{shell.vertices[a[k]].point.u, shell.vertices[a[k]].point.v}
				v := shell.vertices[shell.triangles[j][k]].point
				q[k] = loftPartitionPoint{v.u, v.v}
			}
			if shell.faces[i].level == 0 {
				p[1], p[2] = p[2], p[1]
				q[1], q[2] = q[2], q[1]
			}
			overlap, err := loftOutputTrianglesOverlap(budget, p, q, arithmetic)
			if err != nil || overlap || !arithmetic.ok {
				return false, err
			}
		}
	}
	return loftOutputMiddleContact(budget, shell, source.capLevels[1], directed, arithmetic)
}

func loftOutputEdgeOwnersCompatible(shell loftOutputShell, source loftSourcePartition,
	fi, peer int, edge [2]int, arithmetic ...*loftVolumeArithmetic) bool {
	c := loftReplayArithmetic(arithmetic)
	a, b := shell.faces[fi], shell.faces[peer]
	if a.kind == b.kind && a.owner == b.owner {
		return true
	}
	p := shell.vertices[edge[0]].point
	q := shell.vertices[edge[1]].point
	if a.kind == 0 && b.kind == 0 && a.level == b.level {
		return true // two selected cells share a certified internal cap edge
	}
	if a.kind == 0 && b.kind == 1 || a.kind == 1 && b.kind == 0 {
		capFace, side := a, b
		if b.kind == 0 {
			capFace, side = b, a
		}
		if capFace.level < 0 || capFace.level >= len(source.capLevels) ||
			c.cmp(p.z, q.z) != 0 || c.cmp(p.z, source.capLevels[capFace.level]) != 0 || !c.ok {
			return false
		}
		if capFace.owner == side.owner {
			return true
		}
		if capFace.level != 1 || capFace.owner != 1 || side.owner != 0 ||
			side.segment != source.carrier || side.level != 3 {
			return false
		}
		return loftOutputJunctionEdge(p, q, source.junction[1], source.capLevels[1])
	}
	if a.kind == 1 && b.kind == 1 && a.owner != b.owner {
		prism, loft := a, b
		if b.owner == 0 {
			prism, loft = b, a
		}
		if prism.owner != 0 || loft.owner != 1 || prism.segment != source.carrier {
			return false
		}
		for k, crossed := range source.crossing {
			if loft.segment == crossed && prism.level == k &&
				loftOutputJunctionEdge(p, q,
					[2]sectionrecord.Point2{source.junction[0][k], source.junction[1][k]},
					source.capLevels[0], source.capLevels[1]) {
				return true
			}
		}
		return false
	}
	return false
}

func loftOutputJunctionEdge(a, b loftOutputPoint, junction [2]sectionrecord.Point2,
	levels ...*big.Rat) bool {
	if len(levels) == 1 {
		levels = []*big.Rat{levels[0], levels[0]}
	}
	p, okP := loftPartitionPointOf(junction[0])
	q, okQ := loftPartitionPointOf(junction[1])
	if !okP || !okQ {
		return false
	}
	x, y := loftOutputP2(p, levels[0]), loftOutputP2(q, levels[1])
	return loftOutputSamePoint(a, x) && loftOutputSamePoint(b, y) ||
		loftOutputSamePoint(a, y) && loftOutputSamePoint(b, x)
}

func loftOutputMiddleContact(budget *proofbound.WorkBudget,
	shell loftOutputShell, level *big.Rat, directed map[[2]int]int,
	arithmetic ...*loftVolumeArithmetic) (bool, error) {
	c := loftReplayArithmetic(arithmetic)
	if !c.valid(level) {
		return false, nil
	}
	boundary := make(map[string][2]int)
	capVertices := make(map[string]struct{})
	for fi, tri := range shell.triangles {
		if shell.faces[fi].kind != 0 || shell.faces[fi].level != 1 {
			continue
		}
		for k, v := range tri {
			if err := budget.Step(); err != nil {
				return false, err
			}
			e := [2]int{v, tri[(k+1)%3]}
			peer := directed[[2]int{e[1], e[0]}]
			if shell.faces[peer].kind == 1 {
				boundary[loftOutputEdgeKey(shell, e)] = e
				capVertices[loftOutputPointKey(shell.vertices[e[0]].point)] = struct{}{}
				capVertices[loftOutputPointKey(shell.vertices[e[1]].point)] = struct{}{}
			}
		}
	}
	if len(boundary) == 0 {
		return false, nil
	}
	for fi, tri := range shell.triangles {
		if shell.faces[fi].kind != 1 {
			continue
		}
		trace, coplanar, err := loftOutputPlaneTrace(budget, shell, tri, level, c)
		if err != nil || coplanar || !c.ok {
			return false, err
		}
		if len(trace) == 0 {
			continue
		}
		for ci, cap := range shell.triangles {
			if err := budget.Step(); err != nil {
				return false, err
			}
			if shell.faces[ci].kind != 0 || shell.faces[ci].level != 1 {
				continue
			}
			lo, hi, hit := loftOutputTraceInCapTriangle(trace, cap, shell, c)
			if !c.ok {
				return false, nil
			}
			if !hit {
				continue
			}
			if c.cmp(lo, hi) == 0 {
				if !c.ok {
					return false, nil
				}
				p := trace[0]
				if len(trace) == 2 {
					p = c.partitionInterpolate(trace[0], trace[1], lo)
				}
				if !c.ok {
					return false, nil
				}
				if _, found := capVertices[loftOutputPointKey(loftOutputP2(p, level))]; !found {
					return false, nil
				}
				continue
			}
			a, b := c.partitionInterpolate(trace[0], trace[1], lo),
				c.partitionInterpolate(trace[0], trace[1], hi)
			if !c.ok {
				return false, nil
			}
			key := loftOutputP2EdgeKey(a, b, level)
			edge, found := boundary[key]
			if !found || directed[[2]int{edge[1], edge[0]}] != fi {
				return false, nil
			}
		}
	}
	return c.ok, nil
}

func loftOutputEdgeKey(shell loftOutputShell, edge [2]int) string {
	a, b := shell.vertices[edge[0]].point, shell.vertices[edge[1]].point
	if loftOutputPointKey(a) > loftOutputPointKey(b) {
		a, b = b, a
	}
	return loftOutputPointKey(a) + ":" + loftOutputPointKey(b)
}

func loftOutputP2EdgeKey(a, b loftPartitionPoint, z *big.Rat) string {
	p, q := loftOutputP2(a, z), loftOutputP2(b, z)
	if loftOutputPointKey(p) > loftOutputPointKey(q) {
		p, q = q, p
	}
	return loftOutputPointKey(p) + ":" + loftOutputPointKey(q)
}

func loftOutputPlaneTrace(budget *proofbound.WorkBudget, shell loftOutputShell,
	tri [3]int, z *big.Rat, arithmetic ...*loftVolumeArithmetic) ([]loftPartitionPoint, bool, error) {
	c := loftReplayArithmetic(arithmetic)
	var points []loftPartitionPoint
	add := func(p loftPartitionPoint) {
		for _, q := range points {
			if c.partitionSamePoint(p, q) {
				return
			}
		}
		points = append(points, p)
	}
	onPlane := 0
	for i, vertex := range tri {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		a := shell.vertices[vertex].point
		b := shell.vertices[tri[(i+1)%3]].point
		as, bs := c.cmp(a.z, z), c.cmp(b.z, z)
		if !c.ok {
			return nil, false, nil
		}
		if as == 0 {
			onPlane++
			add(loftPartitionPoint{a.u, a.v})
		}
		if as*bs < 0 {
			t := c.quo(c.sub(z, a.z), c.sub(b.z, a.z))
			add(c.partitionInterpolate(loftPartitionPoint{a.u, a.v}, loftPartitionPoint{b.u, b.v}, t))
			if !c.ok {
				return nil, false, nil
			}
		}
	}
	return points, onPlane == 3 || len(points) > 2, nil
}

func loftOutputTraceInCapTriangle(trace []loftPartitionPoint,
	capTriangle [3]int, shell loftOutputShell,
	arithmetic ...*loftVolumeArithmetic) (*big.Rat, *big.Rat, bool) {
	c := loftReplayArithmetic(arithmetic)
	lo, hi := loftOutputRat(0), loftOutputRat(1)
	if len(trace) == 1 {
		hi = loftOutputRat(0)
	}
	for k, v := range capTriangle {
		a, b := shell.vertices[v].point, shell.vertices[capTriangle[(k+1)%3]].point
		start := c.partitionCross(loftPartitionPoint{a.u, a.v},
			loftPartitionPoint{b.u, b.v}, trace[0])
		end := start
		if len(trace) == 2 {
			end = c.partitionCross(loftPartitionPoint{a.u, a.v},
				loftPartitionPoint{b.u, b.v}, trace[1])
		}
		if !c.ok {
			return nil, nil, false
		}
		if start.Sign() < 0 && end.Sign() < 0 {
			return nil, nil, false
		}
		if start.Sign() == end.Sign() || start.Sign() >= 0 && end.Sign() >= 0 {
			continue
		}
		t := c.quo(start, c.sub(start, end))
		if !c.ok {
			return nil, nil, false
		}
		if start.Sign() < 0 {
			lo = loftOutputMaxRat(lo, t, c)
		} else {
			hi = loftOutputMinRat(hi, t, c)
		}
		if c.cmp(lo, hi) > 0 || !c.ok {
			return nil, nil, false
		}
	}
	return lo, hi, true
}
