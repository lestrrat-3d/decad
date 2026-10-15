package decad

import (
	"context"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This consumer builds the live graph and all-run shell, then adds one
// connected vertical source path to its planar Revolve face. It does not
// classify retained facets or publish a Boolean result.
func revolveLoftFacesBlankShellWithVerticalTrace(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate, selectedCellID int,
) (*revolveLoftFacesBlankSeamShell, *revolveLoftFacesContactGraph,
	*revolveLoftFacesVerticalTrace, bool, error) {
	ctx = revolveLoftFacesWithCallScope(ctx)
	shell, graph, built, err := revolveLoftFacesBlankShellWithAllRuns(ctx, proof)
	if err != nil || !built || shell == nil || graph == nil {
		return nil, nil, nil, false, err
	}
	trace, built, err := revolveLoftFacesInsertVerticalTrace(ctx, proof, shell, graph, selectedCellID)
	if err != nil || !built {
		return nil, nil, nil, false, err
	}
	return shell, graph, trace, true, nil
}

func revolveLoftFacesInsertVerticalTrace(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate, shell *revolveLoftFacesBlankSeamShell,
	graph *revolveLoftFacesContactGraph, selectedCellID int,
) (*revolveLoftFacesVerticalTrace, bool, error) {
	if ctx == nil || proof == nil || shell == nil || graph == nil ||
		selectedCellID < 0 || selectedCellID >= len(graph.cells) ||
		len(shell.triangleCells) != len(shell.mesh.Tris) ||
		len(shell.rays) != shell.rayCount {
		return nil, false, nil
	}
	current, err := revolveLoftFacesContactGraphCurrent(ctx, graph)
	if err != nil || !current || graph.blank != proof.blank || graph.tooth != proof.tooth {
		return nil, false, err
	}
	selected := graph.cells[selectedCellID]
	if selected.vertical == nil || selected.pairID < 0 ||
		selected.pairID >= len(graph.pairs) ||
		graph.pairs[selected.pairID].verdict != revolveLoftFacesPairSourceCandidate {
		return nil, false, nil
	}
	var event *revolveLoftFacesGraphArc
	var ring *revolveLoftFacesShellCapRing
	for i := range graph.arcs {
		candidate := &graph.arcs[i]
		if candidate.sectionIndex != selected.sectionIndex ||
			candidate.arc == nil ||
			candidate.arc.circle.blankFace != selected.blankFace {
			continue
		}
		if event != nil {
			return nil, false, nil
		}
		event = candidate
	}
	if event == nil || event.sectionIndex < 0 ||
		event.sectionIndex >= len(proof.section.spans) ||
		proof.section.spans[event.sectionIndex].start.U !=
			proof.section.spans[event.sectionIndex].end.U {
		return nil, false, nil
	}
	for i := range shell.capRings {
		candidate := &shell.capRings[i]
		if candidate.sectionIndex == event.sectionIndex &&
			candidate.endpointGraphIDs == event.endpointIDs {
			if ring != nil {
				return nil, false, nil
			}
			ring = candidate
		}
	}
	if ring == nil || ring.endpointCellIDs != event.endpointCellIDs ||
		ring.arc == nil ||
		!revolveLoftFacesToolCapOccupiedArcEqual(ring.arc, event.arc) {
		return nil, false, nil
	}
	pieces, chained := revolveLoftFacesVerticalTraceChain(graph, event)
	_, monotone := revolveLoftFacesVerticalTracePieceSigns(pieces)
	if !chained || !monotone ||
		!revolveLoftFacesVerticalTraceSourceProof(pieces, event.arc.circle.radius) {
		return nil, false, nil
	}
	selectedInChain := false
	for _, piece := range pieces {
		selectedInChain = selectedInChain || piece.cellID == selectedCellID
	}
	if !selectedInChain {
		return nil, false, nil
	}
	working := *shell
	working.mesh.Verts = slices.Clone(shell.mesh.Verts)
	working.mesh.Tris = slices.Clone(shell.mesh.Tris)
	working.mesh.Src = slices.Clone(shell.mesh.Src)
	working.mesh.Groups = slices.Clone(shell.mesh.Groups)
	working.vertexBounds = slices.Clone(shell.vertexBounds)
	working.triangleCells = slices.Clone(shell.triangleCells)
	nodes, edges, loftBound, built, err := revolveLoftFacesVerticalTracePath(
		ctx, proof, &working, pieces, *ring)
	if err != nil || !built {
		return nil, false, err
	}
	trace := &revolveLoftFacesVerticalTrace{
		sectionIndex: event.sectionIndex, capIDs: event.endpointIDs,
		vertices: make([]int, len(nodes)), vertexRays: make([]int, len(nodes)),
		cellIDs:    make([]int, len(pieces)),
		edgeWedges: make([]int, len(edges)), edgeCellIDs: make([]int, len(edges)),
		loftBound: loftBound}
	for i, piece := range pieces {
		trace.cellIDs[i] = piece.cellID
	}
	for i, node := range nodes {
		trace.vertices[i] = node.vertex
		trace.vertexRays[i] = node.ray
	}
	for i, edge := range edges {
		trace.edgeWedges[i] = edge.wedge
		trace.edgeCellIDs[i] = pieces[edge.piece].cellID
	}
	built, err = revolveLoftFacesVerticalTraceSplitWedges(ctx, &working, *ring,
		event.sectionIndex, nodes, edges)
	if err != nil || !built || len(working.triangleCells) != len(working.mesh.Tris) ||
		!revolveLoftFacesBlankShellEulerTwo(&working.mesh) {
		return nil, false, err
	}
	maxBound := 0.0
	for _, bound := range working.vertexBounds {
		if proofbound.IsNonFinite(bound) || bound < 0 {
			return nil, false, nil
		}
		maxBound = max(maxBound, bound)
	}
	working.faceBound = proofbound.AbsSumUpper(working.chordBound, maxBound)
	if proofbound.IsNonFinite(working.faceBound) || working.faceBound >= 0.1 {
		return nil, false, nil
	}
	if err := orientJoinedMesh(&working.mesh); err != nil {
		return nil, false, err
	}
	if err := loftmesh.LoftCrossingAuditBounded(proofbound.NewWorkBudget(ctx),
		working.mesh.Verts, working.mesh.Tris, len(working.mesh.Tris), 1<<20); err != nil {
		return nil, false, err
	}
	audit, err := facetproof.AuditFacetedMesh(ctx, working.mesh.Verts, working.mesh.Tris)
	if err != nil {
		return nil, false, err
	}
	if audit == nil || len(audit.Members) != 1 || audit.CompVol[0].Sign() <= 0 {
		return nil, false, nil
	}
	*shell = working
	return trace, true, nil
}

func revolveLoftFacesVerticalTraceHeldPoint(proof *revolveLoftFacesSourceCertificate,
	piece revolveLoftFacesVerticalTracePiece, hit revolveLoftFacesVerticalTraceHit,
	ray revolveLoftFacesRay, radius *big.Rat) (r3.Vec, float64, float64, bool) {
	var empty r3.Vec
	if radius == nil || radius.Sign() <= 0 ||
		piece.cell.band < 0 || piece.cell.band+1 >= len(proof.trim.Maps) {
		return empty, 0, 0, false
	}
	if piece.arc != nil {
		if piece.radius2 == nil || piece.radius2.Sign() <= 0 ||
			piece.radius2.Cmp(lfMul(radius, radius)) >= 0 {
			return empty, 0, 0, false
		}
		r2, _ := piece.radius2.Float64()
		if proofbound.IsNonFinite(r2) || r2 <= 0 {
			return empty, 0, 0, false
		}
		approx := lfF(math.Sqrt(r2))
		if approx.Sign() <= 0 {
			return empty, 0, 0, false
		}
		point, blankBound, valid := revolveLoftFacesMeridianRayPoint(
			[2]*big.Rat{ray.y, ray.z}, piece.x, approx)
		if !valid {
			return empty, 0, 0, false
		}
		residual := proofbound.RatFloatUp(lfDiv(
			lfAbs(lfSub(lfMul(approx, approx), piece.radius2)), approx))
		loftBound := proofbound.AbsSumUpper(blankBound, residual)
		return point, blankBound, loftBound,
			!proofbound.IsNonFinite(loftBound) && loftBound >= 0
	}
	if hit.lo == nil || hit.hi == nil || hit.lo.Cmp(hit.hi) > 0 {
		return empty, 0, 0, false
	}
	box, ok := loftface.JunctionBox(proof.trim.Maps[piece.cell.band],
		proof.trim.Maps[piece.cell.band+1], piece.cell.source,
		hit.lo, hit.hi, piece.time, piece.time)
	if !ok || box[0].Lo.Cmp(piece.x) != 0 || box[0].Hi.Cmp(piece.x) != 0 {
		return empty, 0, 0, false
	}
	midY := lfDiv(lfAdd(box[1].Lo, box[1].Hi), big.NewRat(2, 1))
	midZ := lfDiv(lfAdd(box[2].Lo, box[2].Hi), big.NewRat(2, 1))
	denom := lfAdd(lfMul(ray.y, ray.y), lfMul(ray.z, ray.z))
	if denom.Sign() <= 0 {
		return empty, 0, 0, false
	}
	scale := lfDiv(lfAdd(lfMul(midY, ray.y), lfMul(midZ, ray.z)), denom)
	if scale.Sign() <= 0 {
		return empty, 0, 0, false
	}
	exact := loftFaceRatVec{piece.x, lfMul(scale, ray.y), lfMul(scale, ray.z)}
	radial2 := lfAdd(lfMul(exact[1], exact[1]), lfMul(exact[2], exact[2]))
	if radial2.Sign() <= 0 || radial2.Cmp(lfMul(radius, radius)) >= 0 {
		return empty, 0, 0, false
	}
	held, blankBound, ok := revolveLoftFacesHeldPoint(exact)
	if !ok || proofbound.IsNonFinite(blankBound) {
		return empty, 0, 0, false
	}
	loftBound, valid := revolveLoftFacesVerticalTraceBoxDistance(box, held)
	return held, blankBound, loftBound, valid
}

func revolveLoftFacesVerticalTraceBoxDistance(box [3]loftface.Interval,
	held r3.Vec) (float64, bool) {
	coordinates := [3]float64{held.X, held.Y, held.Z}
	distance := lfZero()
	for axis, coordinate := range coordinates {
		interval := box[axis]
		if interval.Lo == nil || interval.Hi == nil || interval.Lo.Cmp(interval.Hi) > 0 {
			return 0, false
		}
		lo := lfAbs(lfSub(lfF(coordinate), interval.Lo))
		hi := lfAbs(lfSub(lfF(coordinate), interval.Hi))
		if lo.Cmp(hi) > 0 {
			distance = lfAdd(distance, lo)
		} else {
			distance = lfAdd(distance, hi)
		}
	}
	bound := proofbound.RatFloatUp(distance)
	return bound, !proofbound.IsNonFinite(bound) && bound >= 0
}
