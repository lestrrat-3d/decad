package booleaneval

import (
	"context"
	"math"
	"slices"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/gaptol"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

type preparedTagShift struct {
	target, tool                                                         *topo.Body
	targetID, toolID                                                     provenance.Producer
	selected, selectedCopy, decision, decisionCopy, charged, chargedCopy *topo.Mesh
	faces                                                                []*topo.Face
	pieces                                                               pieces
	shift                                                                pieceShift
	displacement, reach                                                  float64
	groups                                                               []facetedrecord.Group
	original                                                             []facetedrecord.Group
}

type preparedTagShiftKey struct{}

// EvaluateBrakeClevisReplay enables the private one-piece tag proof for this Cut.
func EvaluateBrakeClevisReplay(ctx context.Context, target, tool *topo.Body, cert *BrakeCandidateCertificate, b *proofbound.ReplayBudget) (Evaluation, error) {
	if b == nil || !b.IsBrakeReplay() || cert == nil || cert.state.Load() != 1 {
		return Evaluation{}, errBrakeEvalBrakeClevisReplayRequiresItsBudget
	}
	growth, ok := ctx.Value(authenticatedGrowthKey{}).(*AuthenticatedGrowth)
	if !ok || growth == nil || growth.slot != cert.slot || growth.pin != cert.pin {
		return Evaluation{}, errBrakeEvalAuthenticatedGrowthSourceMismatch
	}
	if err := growth.validate(ctx, target, b); err != nil {
		return Evaluation{}, err
	}
	if err := cert.validate(ctx, target, tool, b); err != nil {
		return Evaluation{}, err
	}
	if !cert.state.CompareAndSwap(1, 2) {
		return Evaluation{}, errBrakeEvalCandidateCertificateAlreadyConsumed
	}
	return evaluateReplay(context.WithValue(ctx, preparedTagShiftKey{}, cert), meshbool.OpCut, target, tool, .25, 0, 0, nil, b)
}

func prepareTagShift(ctx context.Context, target, tool *topo.Body, selected *topo.Mesh, faces []*topo.Face,
	pc pieces, facePiece *topo.FaceBoundTable, placed placedDepartures, reachB float64,
	ordered *ReplayShifts, b *proofbound.ReplayBudget) (*preparedTagShift, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	reach := gaptol.Reach(ctx)
	if target == nil || tool == nil || selected == nil || b == nil || !b.IsBrakeReplay() || ordered == nil ||
		ordered.budget != b || len(ordered.entries) != 1 || ordered.entries[0].piece != 0 || pc.count != 1 ||
		len(pc.ofVertex) != len(selected.Vertices) || len(pc.ofFacet) != len(selected.Triangles) ||
		len(selected.Source) != len(selected.Triangles) || !selected.BoundaryOK || !selected.SymDiffOK ||
		len(faces) == 0 || facePiece == nil || len(placed.departure) != 0 || len(placed.stated) != 0 ||
		!(reach > 0) || proofbound.IsNonFinite(reach) {
		return nil, errBrakeEvalPreparedTagShiftLacksASelectedOnePieceTool
	}
	sh := ordered.entries[0].shift
	translation := sh.Grow == 0 && sh.Move != (r3.Vec{}) && proofbound.FiniteVec(sh.Move)
	growth := sh.Move == (r3.Vec{}) && sh.Grow > 0 && !proofbound.IsNonFinite(sh.Grow)
	if !translation && !growth {
		return nil, errBrakeEvalPreparedShiftRequiresAFiniteTranslationOrPositiveGrowth
	}
	n := uint64(len(pc.ofVertex) + len(pc.ofFacet) + len(faces))
	if err := b.SpendWork(ctx, n); err != nil {
		return nil, err
	}
	if err := b.ReserveRecords(ctx, n+2*uint64(len(faces))+1); err != nil {
		return nil, err
	}
	for _, a := range [][2]uint64{{uint64(len(pc.ofVertex) + len(pc.ofFacet)), uint64(unsafe.Sizeof(int(0)))},
		{uint64(len(faces)), uint64(unsafe.Sizeof((*topo.Face)(nil)))},
		{uint64(len(faces)), uint64(unsafe.Sizeof(facetedrecord.Group{}))},
		{uint64(len(faces)), uint64(unsafe.Sizeof(facetedrecord.Group{}))}, {1, uint64(unsafe.Sizeof(preparedTagShift{}))}} {
		if err := b.ReserveArray(ctx, a[0], a[1]); err != nil {
			return nil, err
		}
	}
	for _, component := range pc.ofVertex {
		if component != 0 {
			return nil, errBrakeEvalChangedToolVertexComponent
		}
	}
	for _, component := range pc.ofFacet {
		if component != 0 {
			return nil, errBrakeEvalChangedToolFacetComponent
		}
	}
	p := &preparedTagShift{target: target, tool: tool, targetID: target.Origin.Producer, toolID: tool.Origin.Producer,
		selected: selected, faces: slices.Clone(faces), pieces: pieces{count: pc.count, ofVertex: slices.Clone(pc.ofVertex), ofFacet: slices.Clone(pc.ofFacet)},
		shift: sh, reach: reach, groups: make([]facetedrecord.Group, len(faces)), original: make([]facetedrecord.Group, len(faces))}
	for i, f := range faces {
		if f == nil || f.Body != tool || f.Surface == nil {
			return nil, errBrakeEvalPreparedSourceFaceMissing
		}
		piece, ok, err := facePiece.Lookup(ctx, f, b)
		if err != nil {
			return nil, err
		}
		if !ok || int(piece) != 0 {
			return nil, errBrakeEvalPreparedSourceFaceComponentChanged
		}
		if err := b.ReserveRecords(ctx, uint64(len(f.Origins)+len(f.Upstream))+1); err != nil {
			return nil, err
		}
		for _, size := range []uint64{uint64(len(f.Origins)), uint64(len(f.Upstream))} {
			if err := b.ReserveArray(ctx, size, uint64(unsafe.Sizeof(provenance.Ref{}))); err != nil {
				return nil, err
			}
		}
		if err := b.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(facetedrecord.SourcePatch{}))); err != nil {
			return nil, err
		}
		g := sourceGroup(tool, 1, i, f, true, placed)
		if translation && (g.Surface == nil || !g.Retained || g.Denoted != nil) {
			return nil, errBrakeEvalPreparedMovedSourceTagUnavailable
		}
		p.original[i] = g
		if growth {
			g = facetedrecord.Group{Origins: g.Origins, Upstream: g.Upstream, Source: g.Source}
		} else {
			g, err = replayMovedGroup(ctx, g, 0, 1, sh, selected.Vertices, reachB, b)
			if err != nil {
				return nil, err
			}
		}
		p.groups[i] = g
	}
	var err error
	p.selectedCopy, err = cloneShiftMeshReplay(ctx, selected, b)
	if err != nil {
		return nil, err
	}
	p.decision, p.charged, p.displacement, err = shiftOperandMeshReplay(ctx, selected, pc, ordered, nil, b)
	if err != nil {
		return nil, err
	}
	if p.displacement > reach || proofbound.IsNonFinite(p.displacement) {
		return nil, errBrakeEvalPreparedShiftExceedsReach
	}
	p.decisionCopy, err = cloneShiftMeshReplay(ctx, p.decision, b)
	if err != nil {
		return nil, err
	}
	p.chargedCopy, err = cloneShiftMeshReplay(ctx, p.charged, b)
	if err != nil {
		return nil, err
	}
	return p, nil
}

func (p *preparedTagShift) recheck(ctx context.Context, target, tool *topo.Body, selected *topo.Mesh, faces []*topo.Face,
	pc pieces, facePiece *topo.FaceBoundTable, ordered *ReplayShifts, b *proofbound.ReplayBudget) error {
	if p == nil || b == nil || target == nil || tool == nil || facePiece == nil || ordered == nil || ordered.budget != b || len(ordered.entries) != 1 ||
		ordered.entries[0].piece != 0 || target != p.target || tool != p.tool || selected != p.selected ||
		target.Origin.Producer != p.targetID || tool.Origin.Producer != p.toolID || pc.count != p.pieces.count ||
		math.Float64bits(ordered.entries[0].shift.Move.X) != math.Float64bits(p.shift.Move.X) ||
		math.Float64bits(ordered.entries[0].shift.Move.Y) != math.Float64bits(p.shift.Move.Y) ||
		math.Float64bits(ordered.entries[0].shift.Move.Z) != math.Float64bits(p.shift.Move.Z) ||
		math.Float64bits(ordered.entries[0].shift.Grow) != math.Float64bits(p.shift.Grow) ||
		math.Float64bits(gaptol.Reach(ctx)) != math.Float64bits(p.reach) || p.displacement > p.reach {
		return errBrakeEvalPreparedTagShiftChanged
	}
	if err := b.SpendWork(ctx, uint64(len(faces)+len(pc.ofVertex)+len(pc.ofFacet))); err != nil {
		return err
	}
	if !slices.Equal(faces, p.faces) || !slices.Equal(pc.ofVertex, p.pieces.ofVertex) || !slices.Equal(pc.ofFacet, p.pieces.ofFacet) {
		return errBrakeEvalPreparedSourceMappingChanged
	}
	for i, f := range faces {
		v, ok, err := facePiece.Lookup(ctx, f, b)
		if err != nil {
			return err
		}
		if !ok || int(v) != 0 || f.Body != tool {
			return errBrakeEvalPreparedFaceComponentChanged
		}
		original := p.original[i]
		current, admitted := facetedrecord.SourceTag(f, true)
		if p.shift.Grow == 0 {
			switch current.Surface.(type) {
			case surfacegeom.Plane, surfacegeom.Cylinder:
			default:
				return errBrakeEvalPreparedFaceTagChanged
			}
		}
		if p.shift.Grow == 0 && !admitted || current.Surface != original.Surface || current.NormalBound != original.NormalBound ||
			current.Retained != original.Retained || current.Reversed != original.Reversed || current.Denoted != original.Denoted ||
			(f.Surface.Kind() == surfacegeom.KindPlane || f.HeldPlanar) != original.Planar ||
			current.CapRecord != original.CapRecord || current.AxialDelta != original.AxialDelta || current.HasAxialDelta != original.HasAxialDelta ||
			!slices.Equal(f.Origins, original.Origins) || !slices.Equal(f.Upstream, original.Upstream) ||
			original.Source == nil || original.Source.Parent != f.Source || original.Source.Producer != p.toolID || original.Source.Ordinal != i || original.Source.Operand != 1 {
			return errBrakeEvalPreparedFaceMetadataChanged
		}
	}
	for _, pair := range [][2]*topo.Mesh{{p.selected, p.selectedCopy}, {p.decision, p.decisionCopy}, {p.charged, p.chargedCopy}} {
		if pair[0] == nil || pair[1] == nil {
			return errBrakeEvalPreparedMeshMissing
		}
		if err := b.SpendWork(ctx, uint64(len(pair[0].Vertices)+len(pair[0].Triangles)+len(pair[0].VertexBound)+len(pair[0].Source))); err != nil {
			return err
		}
		same, err := sameShiftMeshReplay(ctx, pair[0], pair[1], b)
		if err != nil {
			return err
		}
		if !same {
			return errBrakeEvalPreparedMeshChanged
		}
	}
	return nil
}
