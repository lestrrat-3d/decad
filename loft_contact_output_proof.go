package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// loftOutputBoundaryProof is private, transient and has no occupied-volume or
// measurement claim. The source records must remain live throughout its call.
type loftOutputBoundaryProof struct {
	source       loftcontact.SourcePartition
	shell        loftcontact.OutputShell
	workVisits   uint64
	heldAudit    *facetproof.MeshAudit
	auditedVerts []r3.Vec
	auditedTris  [][3]int
}

// Both proof modes charge their held audits to the output budget.
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
	arithmetic := &loftcontact.Arithmetic{OK: true}
	if err := ctx.Err(); err != nil {
		return no, false, err
	}
	if first == nil || second == nil || operation == nil {
		return no, false, nil
	}
	var source loftcontact.SourcePartition
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
	if !arithmetic.OK {
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
	shell := loftcontact.NewOutputShell()
	for cap := range 3 {
		ok, err = loftcontact.OutputCap(ctx, budget, pp.Frame, prismLines, bottom, top, source, &shell, cap, arithmetic)
		if err != nil || !ok || !arithmetic.OK {
			return no, false, err
		}
	}
	ok, err = loftOutputSides(budget, pp, prismLines, bottom, top, source, &shell, arithmetic)
	if err != nil || !ok || !arithmetic.OK {
		return no, false, err
	}
	ok, err = loftOutputAuditSource(budget, shell, source, arithmetic)
	if err != nil || !ok || !arithmetic.OK {
		return no, false, err
	}
	if err := budget.Err(); err != nil {
		return no, false, err
	}
	if directShell {
		if _, err := budget.StepN(uint64(len(shell.Vertices) + len(shell.Triangles))); err != nil {
			return no, false, err
		}
		if err := budget.Err(); err != nil {
			return no, false, err
		}
	}
	verts := make([]r3.Vec, len(shell.Vertices))
	for i, v := range shell.Vertices {
		if err := budget.Step(); err != nil {
			return no, false, err
		}
		verts[i] = v.Held
	}
	var auditedTris [][3]int
	if directShell {
		auditedTris = append([][3]int(nil), shell.Triangles...)
	}
	// The exact source labels and orientation are already fixed. Neither audit
	// may reverse a triangle to repair an ownership mistake.
	if err := loftmesh.LoftCrossingAuditPrivateBounded(budget, verts, shell.Triangles); err != nil {
		return no, false, err
	}
	auditTris := shell.Triangles
	if directShell {
		auditTris = auditedTris
	}
	audit, audited, err := facetproof.AuditFacetedMeshBounded(ctx, budget, verts, auditTris)
	if err != nil || !audited {
		return no, false, err
	}
	if audit == nil || len(audit.Members) != 1 {
		return no, false, nil
	}
	if err := budget.Err(); err != nil {
		return no, false, err
	}
	if !arithmetic.OK {
		return no, false, nil
	}
	out := loftOutputBoundaryProof{source: source, shell: shell, workVisits: visits}
	if directShell {
		out.heldAudit, out.auditedVerts, out.auditedTris = audit, verts, auditedTris
	}
	return out, true, nil
}

func loftOutputSides(budget *proofbound.WorkBudget, pp prismPayload,
	prismLines, bottom, top []sectionrecord.LineSeg, source loftcontact.SourcePartition,
	shell *loftcontact.OutputShell, checked ...*loftcontact.Arithmetic) (bool, error) {
	arithmetic := loftcontact.ReplayArithmetic(checked)
	z0, zL, zP := source.CapLevels[0], source.CapLevels[1], source.CapLevels[2]
	var lineCut, carrierCut [2][2]*big.Rat
	for k, seam := range source.Seams {
		for t := range 2 {
			if err := budget.Step(); err != nil {
				return false, err
			}
			var ok bool
			lineCut[t][k], carrierCut[t][k], ok = seam.At(big.NewRat(int64(t), 1))
			if !ok || !arithmetic.Valid(lineCut[t][k]) || !arithmetic.Valid(carrierCut[t][k]) {
				return false, nil
			}
		}
		mid, midCarrier, ok := seam.At(big.NewRat(1, 2))
		if !ok || !arithmetic.Valid(mid) || !arithmetic.Valid(midCarrier) {
			return false, nil
		}
		moving := source.Crossing[k]
		midStart, okStart := loftOutputMovingPoint(bottom[moving].Start, top[moving].Start,
			big.NewRat(1, 2))
		midEnd, okEnd := loftOutputMovingPoint(bottom[moving].End, top[moving].End,
			big.NewRat(1, 2))
		carrierPoint := arithmetic.LinePoint(prismLines[source.Carrier], midCarrier)
		interpolated := arithmetic.Interpolate(midStart, midEnd, mid)
		if !okStart || !okEnd || !arithmetic.SamePoint(interpolated, carrierPoint) || !arithmetic.OK {
			return false, nil
		}
		linear := arithmetic.Quo(arithmetic.Add(carrierCut[0][k], carrierCut[1][k]), big.NewRat(2, 1))
		if arithmetic.Cmp(midCarrier, linear) != 0 || !arithmetic.OK {
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
		if i != source.Carrier {
			for band, levels := range [][2]*big.Rat{{z0, zL}, {zL, zP}} {
				ok, err := loftOutputLineQuad(budget, pp.Frame, shell, line, line,
					big.NewRat(0, 1), big.NewRat(1, 1), big.NewRat(0, 1), big.NewRat(1, 1),
					levels[0], levels[1], loftcontact.OutputFace{Kind: 1, Owner: 0, Segment: i, Level: band})
				if err != nil || !ok {
					return false, err
				}
			}
			continue
		}
		for part, bounds := range [][4]*big.Rat{
			{big.NewRat(0, 1), carrierCut[0][0], big.NewRat(0, 1), carrierCut[1][0]},
			{carrierCut[0][1], big.NewRat(1, 1), carrierCut[1][1], big.NewRat(1, 1)},
		} {
			ok, err := loftOutputLineQuad(budget, pp.Frame, shell, line, line,
				bounds[0], bounds[1], bounds[2], bounds[3], z0, zL,
				loftcontact.OutputFace{Kind: 1, Owner: 0, Segment: i, Level: part})
			if err != nil || !ok {
				return false, err
			}
		}
		breaks := [4]*big.Rat{big.NewRat(0, 1), carrierCut[1][0], carrierCut[1][1], big.NewRat(1, 1)}
		for part := range 3 {
			ok, err := loftOutputLineQuad(budget, pp.Frame, shell, line, line,
				breaks[part], breaks[part+1], breaks[part], breaks[part+1], zL, zP,
				loftcontact.OutputFace{Kind: 1, Owner: 0, Segment: i, Level: part + 2})
			if err != nil || !ok {
				return false, err
			}
		}
	}
	for i := range bottom {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if source.Walls[i] == -1 {
			continue
		}
		if !loftOutputPlanarWall(bottom[i], top[i], zL) {
			return false, nil
		}
		lo, hi := big.NewRat(0, 1), big.NewRat(1, 1)
		upperLo, upperHi := big.NewRat(0, 1), big.NewRat(1, 1)
		switch source.Walls[i] {
		case 0:
			k := 0
			if source.Crossing[1] == i {
				k = 1
			} else if source.Crossing[0] != i {
				return false, nil
			}
			side := arithmetic.LineSide(prismLines[source.Carrier], bottom[i].Start)
			if !arithmetic.OK || side.Sign() == 0 {
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
			loftcontact.OutputFace{Kind: 1, Owner: 1, Segment: i})
		if err != nil || !ok {
			return false, err
		}
	}
	return arithmetic.OK, nil
}

func loftOutputLineQuad(budget *proofbound.WorkBudget, frame r3.Frame,
	shell *loftcontact.OutputShell, low, high sectionrecord.LineSeg,
	lo0, hi0, lo1, hi1, z0, z1 *big.Rat, face loftcontact.OutputFace) (bool, error) {
	if err := budget.Step(); err != nil {
		return false, err
	}
	arithmetic := &loftcontact.Arithmetic{OK: true}
	if arithmetic.Cmp(lo0, hi0) >= 0 || arithmetic.Cmp(lo1, hi1) >= 0 ||
		arithmetic.Cmp(z0, z1) >= 0 || !arithmetic.OK {
		return false, nil
	}
	params := [4]*big.Rat{lo0, hi0, hi1, lo1}
	lines := [4]sectionrecord.LineSeg{low, low, high, high}
	levels := [4]*big.Rat{z0, z0, z1, z1}
	var quad [4]loftcontact.OutputPoint
	for i := range quad {
		p := arithmetic.LinePoint(lines[i], params[i])
		if !arithmetic.OK {
			return false, nil
		}
		quad[i] = loftcontact.OutputP2(p, levels[i])
	}
	if !loftOutputQuadPlanarConvex(quad) {
		return false, nil
	}
	for _, tri := range [][3]int{{0, 1, 2}, {0, 2, 3}} {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if !shell.Triangle(frame,
			[3]loftcontact.OutputPoint{quad[tri[0]], quad[tri[1]], quad[tri[2]]}, face) {
			return false, nil
		}
	}
	return true, nil
}

func loftOutputDet3(arithmetic *loftcontact.Arithmetic, a, b, c [3]*big.Rat) *big.Rat {
	x := arithmetic.Sub(arithmetic.Mul(b[1], c[2]), arithmetic.Mul(b[2], c[1]))
	y := arithmetic.Sub(arithmetic.Mul(b[2], c[0]), arithmetic.Mul(b[0], c[2]))
	z := arithmetic.Sub(arithmetic.Mul(b[0], c[1]), arithmetic.Mul(b[1], c[0]))
	return arithmetic.Add(arithmetic.Add(arithmetic.Mul(a[0], x),
		arithmetic.Mul(a[1], y)), arithmetic.Mul(a[2], z))
}

func loftOutputCrossVec(arithmetic *loftcontact.Arithmetic, a, b [3]*big.Rat) [3]*big.Rat {
	var c [3]*big.Rat
	for i := range c {
		j, k := (i+1)%3, (i+2)%3
		c[i] = arithmetic.Sub(arithmetic.Mul(a[j], b[k]), arithmetic.Mul(a[k], b[j]))
	}
	return c
}

func loftOutputWorldDelta(arithmetic *loftcontact.Arithmetic, a, b [3]*big.Rat) [3]*big.Rat {
	var d [3]*big.Rat
	for i := range d {
		d[i] = arithmetic.Sub(b[i], a[i])
	}
	return d
}

func loftOutputHeldExact(v r3.Vec) [3]*big.Rat {
	return [3]*big.Rat{proofarith.FloatRat(v.X), proofarith.FloatRat(v.Y), proofarith.FloatRat(v.Z)}
}

func loftOutputNormalsAgree(a, b, c loftcontact.OutputVertex) bool {
	arithmetic := &loftcontact.Arithmetic{OK: true}
	source := loftOutputCrossVec(arithmetic, loftOutputWorldDelta(arithmetic, a.World, b.World),
		loftOutputWorldDelta(arithmetic, a.World, c.World))
	ha, hb, hc := loftOutputHeldExact(a.Held), loftOutputHeldExact(b.Held), loftOutputHeldExact(c.Held)
	held := loftOutputCrossVec(arithmetic, loftOutputWorldDelta(arithmetic, ha, hb),
		loftOutputWorldDelta(arithmetic, ha, hc))
	dot := new(big.Rat)
	for i := range source {
		dot = arithmetic.Add(dot, arithmetic.Mul(source[i], held[i]))
	}
	return arithmetic.OK && dot.Sign() > 0
}

func loftOutputMovingPoint(low, high sectionrecord.Point2, t *big.Rat) (loftcontact.Point, bool) {
	a, okA := loftcontact.PointOf(low)
	b, okB := loftcontact.PointOf(high)
	if !okA || !okB {
		return loftcontact.Point{}, false
	}
	arithmetic := &loftcontact.Arithmetic{OK: true}
	point := loftcontact.Point{
		U: arithmetic.Add(a.U, arithmetic.Mul(t, arithmetic.Sub(b.U, a.U))),
		V: arithmetic.Add(a.V, arithmetic.Mul(t, arithmetic.Sub(b.V, a.V))),
	}
	return point, arithmetic.OK
}

func loftOutputDelta(arithmetic *loftcontact.Arithmetic, a, b loftcontact.OutputPoint) [3]*big.Rat {
	return [3]*big.Rat{arithmetic.Sub(b.U, a.U), arithmetic.Sub(b.V, a.V),
		arithmetic.Sub(b.Z, a.Z)}
}

func loftOutputPlanarWall(low, high sectionrecord.LineSeg, z *big.Rat) bool {
	a, okA := loftcontact.PointOf(low.Start)
	b, okB := loftcontact.PointOf(low.End)
	c, okC := loftcontact.PointOf(high.Start)
	d, okD := loftcontact.PointOf(high.End)
	if !okA || !okB || !okC || !okD {
		return false
	}
	p, q, r, s := loftcontact.OutputP2(a, big.NewRat(0, 1)), loftcontact.OutputP2(b, big.NewRat(0, 1)),
		loftcontact.OutputP2(c, z), loftcontact.OutputP2(d, z)
	arithmetic := &loftcontact.Arithmetic{OK: true}
	det := loftOutputDet3(arithmetic, loftOutputDelta(arithmetic, p, q),
		loftOutputDelta(arithmetic, p, r), loftOutputDelta(arithmetic, p, s))
	return arithmetic.OK && det.Sign() == 0
}

func loftOutputQuadPlanarConvex(q [4]loftcontact.OutputPoint) bool {
	arithmetic := &loftcontact.Arithmetic{OK: true}
	if loftOutputDet3(arithmetic, loftOutputDelta(arithmetic, q[0], q[1]),
		loftOutputDelta(arithmetic, q[0], q[2]),
		loftOutputDelta(arithmetic, q[0], q[3])).Sign() != 0 || !arithmetic.OK ||
		loftcontact.OutputCrossZero([3]loftcontact.OutputPoint{q[0], q[1], q[2]}) ||
		loftcontact.OutputCrossZero([3]loftcontact.OutputPoint{q[0], q[2], q[3]}) {
		return false
	}
	n0 := loftOutputCrossVec(arithmetic, loftOutputDelta(arithmetic, q[0], q[1]),
		loftOutputDelta(arithmetic, q[0], q[2]))
	n1 := loftOutputCrossVec(arithmetic, loftOutputDelta(arithmetic, q[0], q[2]),
		loftOutputDelta(arithmetic, q[0], q[3]))
	dot := new(big.Rat)
	for i := range n0 {
		dot = arithmetic.Add(dot, arithmetic.Mul(n0[i], n1[i]))
	}
	// Monotone height and ordered edge parameters then make this a simple
	// planar quadrilateral; matching normals establish both triangle sides.
	return arithmetic.OK && dot.Sign() > 0
}

func loftOutputAuditSource(budget *proofbound.WorkBudget,
	shell loftcontact.OutputShell, source loftcontact.SourcePartition, checked ...*loftcontact.Arithmetic) (bool, error) {
	arithmetic := loftcontact.ReplayArithmetic(checked)
	if len(shell.Triangles) == 0 || len(shell.Triangles) > 512 ||
		len(shell.Vertices) == 0 || len(shell.Vertices) > 512 ||
		len(shell.Faces) != len(shell.Triangles) ||
		len(shell.TriangleBounds) != len(shell.Triangles) {
		return false, nil
	}
	directed := make(map[[2]int]int, 3*len(shell.Triangles))
	incident := make([][]int, len(shell.Vertices))
	incidences := make([]map[loftcontact.OutputFace]struct{}, len(shell.Vertices))
	for fi, tri := range shell.Triangles {
		if err := budget.Step(); err != nil {
			return false, err
		}
		for k, vertex := range tri {
			if vertex < 0 || vertex >= len(shell.Vertices) {
				return false, nil
			}
			incident[vertex] = append(incident[vertex], fi)
			if incidences[vertex] == nil {
				incidences[vertex] = make(map[loftcontact.OutputFace]struct{})
			}
			incidences[vertex][shell.Faces[fi]] = struct{}{}
			edge := [2]int{vertex, tri[(k+1)%3]}
			if _, duplicate := directed[edge]; duplicate {
				return false, nil
			}
			directed[edge] = fi
		}
		if shell.TriangleBounds[fi] < 0 || proofbound.IsNonFinite(shell.TriangleBounds[fi]) ||
			shell.TriangleBounds[fi] != max(shell.Vertices[tri[0]].Bound,
				shell.Vertices[tri[1]].Bound, shell.Vertices[tri[2]].Bound) {
			return false, nil
		}
		if !loftOutputNormalsAgree(shell.Vertices[tri[0]],
			shell.Vertices[tri[1]], shell.Vertices[tri[2]]) {
			return false, nil
		}
	}
	for edge, fi := range directed {
		if err := budget.Step(); err != nil {
			return false, err
		}
		peer, found := directed[[2]int{edge[1], edge[0]}]
		if !found || !loftOutputEdgeOwnersCompatible(shell, source, fi, peer, edge, arithmetic) ||
			!arithmetic.OK {
			return false, nil
		}
	}
	for vertex, faces := range incident {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if len(faces) < 3 || len(shell.Vertices[vertex].Incidences) != len(incidences[vertex]) {
			return false, nil
		}
		for face := range incidences[vertex] {
			if err := budget.Step(); err != nil {
				return false, err
			}
			if _, found := shell.Vertices[vertex].Incidences[face]; !found {
				return false, nil
			}
		}
		seen := map[int]struct{}{faces[0]: {}}
		queue := []int{faces[0]}
		for len(queue) != 0 {
			fi := queue[0]
			queue = queue[1:]
			tri := shell.Triangles[fi]
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
	for i, a := range shell.Triangles {
		if shell.Faces[i].Kind != 0 {
			continue
		}
		for j := range i {
			if err := budget.Step(); err != nil {
				return false, err
			}
			if shell.Faces[j].Kind != 0 || shell.Faces[j].Level != shell.Faces[i].Level {
				continue
			}
			p, q := [3]loftcontact.Point{}, [3]loftcontact.Point{}
			for k := range 3 {
				p[k] = loftcontact.Point{U: shell.Vertices[a[k]].Point.U, V: shell.Vertices[a[k]].Point.V}
				v := shell.Vertices[shell.Triangles[j][k]].Point
				q[k] = loftcontact.Point{U: v.U, V: v.V}
			}
			if shell.Faces[i].Level == 0 {
				p[1], p[2] = p[2], p[1]
				q[1], q[2] = q[2], q[1]
			}
			overlap, err := loftcontact.TrianglesOverlap(budget, p, q, arithmetic)
			if err != nil || overlap || !arithmetic.OK {
				return false, err
			}
		}
	}
	return loftOutputMiddleContact(budget, shell, source.CapLevels[1], directed, arithmetic)
}

func loftOutputEdgeOwnersCompatible(shell loftcontact.OutputShell, source loftcontact.SourcePartition,
	fi, peer int, edge [2]int, arithmetic ...*loftcontact.Arithmetic) bool {
	c := loftcontact.ReplayArithmetic(arithmetic)
	a, b := shell.Faces[fi], shell.Faces[peer]
	if a.Kind == b.Kind && a.Owner == b.Owner {
		return true
	}
	p := shell.Vertices[edge[0]].Point
	q := shell.Vertices[edge[1]].Point
	if a.Kind == 0 && b.Kind == 0 && a.Level == b.Level {
		return true // two selected cells share a certified internal cap edge
	}
	if a.Kind == 0 && b.Kind == 1 || a.Kind == 1 && b.Kind == 0 {
		capFace, side := a, b
		if b.Kind == 0 {
			capFace, side = b, a
		}
		if capFace.Level < 0 || capFace.Level >= len(source.CapLevels) ||
			c.Cmp(p.Z, q.Z) != 0 || c.Cmp(p.Z, source.CapLevels[capFace.Level]) != 0 || !c.OK {
			return false
		}
		if capFace.Owner == side.Owner {
			return true
		}
		if capFace.Level != 1 || capFace.Owner != 1 || side.Owner != 0 ||
			side.Segment != source.Carrier || side.Level != 3 {
			return false
		}
		return loftOutputJunctionEdge(p, q, source.Junction[1], source.CapLevels[1])
	}
	if a.Kind == 1 && b.Kind == 1 && a.Owner != b.Owner {
		prism, loft := a, b
		if b.Owner == 0 {
			prism, loft = b, a
		}
		if prism.Owner != 0 || loft.Owner != 1 || prism.Segment != source.Carrier {
			return false
		}
		for k, crossed := range source.Crossing {
			if loft.Segment == crossed && prism.Level == k &&
				loftOutputJunctionEdge(p, q,
					[2]sectionrecord.Point2{source.Junction[0][k], source.Junction[1][k]},
					source.CapLevels[0], source.CapLevels[1]) {
				return true
			}
		}
		return false
	}
	return false
}

func loftOutputJunctionEdge(a, b loftcontact.OutputPoint, junction [2]sectionrecord.Point2,
	levels ...*big.Rat) bool {
	if len(levels) == 1 {
		levels = []*big.Rat{levels[0], levels[0]}
	}
	p, okP := loftcontact.PointOf(junction[0])
	q, okQ := loftcontact.PointOf(junction[1])
	if !okP || !okQ {
		return false
	}
	x, y := loftcontact.OutputP2(p, levels[0]), loftcontact.OutputP2(q, levels[1])
	return loftcontact.OutputSamePoint(a, x) && loftcontact.OutputSamePoint(b, y) ||
		loftcontact.OutputSamePoint(a, y) && loftcontact.OutputSamePoint(b, x)
}

func loftOutputMiddleContact(budget *proofbound.WorkBudget,
	shell loftcontact.OutputShell, level *big.Rat, directed map[[2]int]int,
	arithmetic ...*loftcontact.Arithmetic) (bool, error) {
	c := loftcontact.ReplayArithmetic(arithmetic)
	if !c.Valid(level) {
		return false, nil
	}
	boundary := make(map[string][2]int)
	capVertices := make(map[string]struct{})
	for fi, tri := range shell.Triangles {
		if shell.Faces[fi].Kind != 0 || shell.Faces[fi].Level != 1 {
			continue
		}
		for k, v := range tri {
			if err := budget.Step(); err != nil {
				return false, err
			}
			e := [2]int{v, tri[(k+1)%3]}
			peer := directed[[2]int{e[1], e[0]}]
			if shell.Faces[peer].Kind == 1 {
				boundary[loftOutputEdgeKey(shell, e)] = e
				capVertices[loftcontact.OutputPointKey(shell.Vertices[e[0]].Point)] = struct{}{}
				capVertices[loftcontact.OutputPointKey(shell.Vertices[e[1]].Point)] = struct{}{}
			}
		}
	}
	if len(boundary) == 0 {
		return false, nil
	}
	for fi, tri := range shell.Triangles {
		if shell.Faces[fi].Kind != 1 {
			continue
		}
		trace, coplanar, err := loftOutputPlaneTrace(budget, shell, tri, level, c)
		if err != nil || coplanar || !c.OK {
			return false, err
		}
		if len(trace) == 0 {
			continue
		}
		for ci, cap := range shell.Triangles {
			if err := budget.Step(); err != nil {
				return false, err
			}
			if shell.Faces[ci].Kind != 0 || shell.Faces[ci].Level != 1 {
				continue
			}
			lo, hi, hit := loftOutputTraceInCapTriangle(trace, cap, shell, c)
			if !c.OK {
				return false, nil
			}
			if !hit {
				continue
			}
			if c.Cmp(lo, hi) == 0 {
				if !c.OK {
					return false, nil
				}
				p := trace[0]
				if len(trace) == 2 {
					p = c.Interpolate(trace[0], trace[1], lo)
				}
				if !c.OK {
					return false, nil
				}
				if _, found := capVertices[loftcontact.OutputPointKey(loftcontact.OutputP2(p, level))]; !found {
					return false, nil
				}
				continue
			}
			a, b := c.Interpolate(trace[0], trace[1], lo),
				c.Interpolate(trace[0], trace[1], hi)
			if !c.OK {
				return false, nil
			}
			key := loftOutputP2EdgeKey(a, b, level)
			edge, found := boundary[key]
			if !found || directed[[2]int{edge[1], edge[0]}] != fi {
				return false, nil
			}
		}
	}
	return c.OK, nil
}

func loftOutputEdgeKey(shell loftcontact.OutputShell, edge [2]int) string {
	a, b := shell.Vertices[edge[0]].Point, shell.Vertices[edge[1]].Point
	if loftcontact.OutputPointKey(a) > loftcontact.OutputPointKey(b) {
		a, b = b, a
	}
	return loftcontact.OutputPointKey(a) + ":" + loftcontact.OutputPointKey(b)
}

func loftOutputP2EdgeKey(a, b loftcontact.Point, z *big.Rat) string {
	p, q := loftcontact.OutputP2(a, z), loftcontact.OutputP2(b, z)
	if loftcontact.OutputPointKey(p) > loftcontact.OutputPointKey(q) {
		p, q = q, p
	}
	return loftcontact.OutputPointKey(p) + ":" + loftcontact.OutputPointKey(q)
}

func loftOutputPlaneTrace(budget *proofbound.WorkBudget, shell loftcontact.OutputShell,
	tri [3]int, z *big.Rat, arithmetic ...*loftcontact.Arithmetic) ([]loftcontact.Point, bool, error) {
	c := loftcontact.ReplayArithmetic(arithmetic)
	var points []loftcontact.Point
	add := func(p loftcontact.Point) {
		for _, q := range points {
			if c.SamePoint(p, q) {
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
		a := shell.Vertices[vertex].Point
		b := shell.Vertices[tri[(i+1)%3]].Point
		as, bs := c.Cmp(a.Z, z), c.Cmp(b.Z, z)
		if !c.OK {
			return nil, false, nil
		}
		if as == 0 {
			onPlane++
			add(loftcontact.Point{U: a.U, V: a.V})
		}
		if as*bs < 0 {
			t := c.Quo(c.Sub(z, a.Z), c.Sub(b.Z, a.Z))
			add(c.Interpolate(loftcontact.Point{U: a.U, V: a.V}, loftcontact.Point{U: b.U, V: b.V}, t))
			if !c.OK {
				return nil, false, nil
			}
		}
	}
	return points, onPlane == 3 || len(points) > 2, nil
}

func loftOutputTraceInCapTriangle(trace []loftcontact.Point,
	capTriangle [3]int, shell loftcontact.OutputShell,
	arithmetic ...*loftcontact.Arithmetic) (*big.Rat, *big.Rat, bool) {
	c := loftcontact.ReplayArithmetic(arithmetic)
	lo, hi := big.NewRat(0, 1), big.NewRat(1, 1)
	if len(trace) == 1 {
		hi = big.NewRat(0, 1)
	}
	for k, v := range capTriangle {
		a, b := shell.Vertices[v].Point, shell.Vertices[capTriangle[(k+1)%3]].Point
		start := c.PointCross(loftcontact.Point{U: a.U, V: a.V},
			loftcontact.Point{U: b.U, V: b.V}, trace[0])
		end := start
		if len(trace) == 2 {
			end = c.PointCross(loftcontact.Point{U: a.U, V: a.V},
				loftcontact.Point{U: b.U, V: b.V}, trace[1])
		}
		if !c.OK {
			return nil, nil, false
		}
		if start.Sign() < 0 && end.Sign() < 0 {
			return nil, nil, false
		}
		if start.Sign() == end.Sign() || start.Sign() >= 0 && end.Sign() >= 0 {
			continue
		}
		t := c.Quo(start, c.Sub(start, end))
		if !c.OK {
			return nil, nil, false
		}
		if start.Sign() < 0 {
			lo = loftcontact.MaxRat(lo, t, c)
		} else {
			hi = loftcontact.MinRat(hi, t, c)
		}
		if c.Cmp(lo, hi) > 0 || !c.OK {
			return nil, nil, false
		}
	}
	return lo, hi, true
}
