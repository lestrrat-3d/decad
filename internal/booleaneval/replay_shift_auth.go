package booleaneval

import (
	"context"
	"math"
	"math/big"
	"slices"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// FixedShiftedTool carries only an authenticated original Cut tool and its
// exactly replayed decision and charged meshes.
type FixedShiftedTool struct {
	tool                        *topo.Body
	selected, decision, charged *topo.Mesh
	groups                      []facetedrecord.Group
	displacement                float64
	move                        r3.Vec
}

// RetainedTarget binds the accepted original Cut mesh to its operand and chord.
type RetainedTarget struct {
	body  *topo.Body
	mesh  *topo.Mesh
	chord float64
}

// RetainedCutTarget authenticates and copies the accepted Cut's target mesh.
func RetainedCutTarget(ctx context.Context, cert *ShiftCertificate, result, target *topo.Body, budget *proofbound.ReplayBudget) (*RetainedTarget, error) {
	if cert == nil || budget == nil || result == nil || target == nil || !cert.sealed || result.PrivateShift != cert ||
		cert.resultID != result.Origin.Producer || cert.target != target || cert.targetID != target.Origin.Producer ||
		cert.targetMesh == nil || !(cert.targetChord > 0) || proofbound.IsNonFinite(cert.targetChord) ||
		!cert.targetMesh.BoundaryOK || !cert.targetMesh.SymDiffOK {
		return nil, errShiftAuthRetainedCutTargetIsUnavailable
	}
	mesh, err := cloneShiftMeshReplay(ctx, cert.targetMesh, budget)
	if err != nil {
		return nil, err
	}
	return &RetainedTarget{body: target, mesh: mesh, chord: cert.targetChord}, nil
}

func retainedCutTargetSnapshotCopy(ctx context.Context, cert *ShiftCertificate, result, target *topo.Body, budget *proofbound.ReplayBudget) (*RetainedTarget, error) {
	if cert == nil || budget == nil || result == nil || target == nil || !cert.sealed ||
		cert.resultID != result.Origin.Producer || cert.target != target || cert.targetID != target.Origin.Producer ||
		cert.targetMesh == nil || !(cert.targetChord > 0) || proofbound.IsNonFinite(cert.targetChord) ||
		!cert.targetMesh.BoundaryOK || !cert.targetMesh.SymDiffOK {
		return nil, errShiftAuthRetainedCutTargetIsUnavailable
	}
	mesh, err := cloneShiftMeshReplay(ctx, cert.targetMesh, budget)
	if err != nil {
		return nil, err
	}
	return &RetainedTarget{body: target, mesh: mesh, chord: cert.targetChord}, nil
}

// AuthenticateStoredCut recreates the originally selected tool mesh, checks
// its public warning and source identity, and replays its exact translation.
func AuthenticateStoredCut(ctx context.Context, cert *ShiftCertificate, result, a, b *topo.Body, warning opdiagnostic.Record,
	budget *proofbound.ReplayBudget) (*FixedShiftedTool, error) {
	if cert == nil || budget == nil || result == nil || !cert.sealed || result.PrivateShift != cert || result.Origin.Producer != cert.resultID || !sameShiftWarning(warning, cert.publicWarning) {
		return nil, errShiftAuthShiftedCutCertificateIsMissing
	}
	tol, _, err := booloperand.PairChordReplay(ctx, a, b, budget)
	if err != nil {
		return nil, err
	}
	pair := tol * cert.scale
	selected, err := cloneShiftMeshReplay(ctx, cert.selected, budget)
	if err != nil {
		return nil, err
	}
	charged, d, err := cert.AuthenticateCutTranslation(ctx, a, b, selected, pair, cert.scale, warning, budget)
	if err != nil {
		return nil, err
	}
	slots := uint64(len(cert.groups) + len(cert.decision.Vertices) + len(cert.decision.Triangles) + len(cert.decision.VertexBound) + len(cert.decision.Source) + len(cert.decision.FaceBound) + 1)
	if err := budget.SpendWork(ctx, slots); err != nil {
		return nil, err
	}
	if err := budget.ReserveRecords(ctx, slots); err != nil {
		return nil, err
	}
	decision, err := cloneShiftMeshReplay(ctx, cert.decision, budget)
	if err != nil {
		return nil, err
	}
	groups, err := snapshotShiftGroups(ctx, cert.groups, budget)
	if err != nil {
		return nil, err
	}
	return &FixedShiftedTool{tool: b, selected: selected, decision: decision, charged: charged, groups: groups, displacement: d, move: cert.shift.Move}, nil
}

func authenticateStoredCutSnapshotCopy(ctx context.Context, cert *ShiftCertificate, result, a, b *topo.Body, warning opdiagnostic.Record,
	budget *proofbound.ReplayBudget) (*FixedShiftedTool, error) {
	if cert == nil || budget == nil || result == nil || !cert.sealed || result.Origin.Producer != cert.resultID || !sameShiftWarning(warning, cert.publicWarning) {
		return nil, errShiftAuthShiftedCutCertificateIsMissing
	}
	tol, _, err := booloperand.PairChordReplay(ctx, a, b, budget)
	if err != nil {
		return nil, err
	}
	pair := tol * cert.scale
	selected, err := cloneShiftMeshReplay(ctx, cert.selected, budget)
	if err != nil {
		return nil, err
	}
	charged, d, err := cert.AuthenticateCutTranslation(ctx, a, b, selected, pair, cert.scale, warning, budget)
	if err != nil {
		return nil, err
	}
	slots := uint64(len(cert.groups) + len(cert.decision.Vertices) + len(cert.decision.Triangles) + len(cert.decision.VertexBound) + len(cert.decision.Source) + len(cert.decision.FaceBound) + 1)
	if err := budget.SpendWork(ctx, slots); err != nil {
		return nil, err
	}
	if err := budget.ReserveRecords(ctx, slots); err != nil {
		return nil, err
	}
	decision, err := cloneShiftMeshReplay(ctx, cert.decision, budget)
	if err != nil {
		return nil, err
	}
	groups, err := snapshotShiftGroups(ctx, cert.groups, budget)
	if err != nil {
		return nil, err
	}
	return &FixedShiftedTool{tool: b, selected: selected, decision: decision, charged: charged, groups: groups, displacement: d, move: cert.shift.Move}, nil
}

// AuthenticateStoredGrowth checks a committed Cut's original pure growth.
// It does not authorize a different refinement or a new growth center.
func AuthenticateStoredGrowth(ctx context.Context, cert *ShiftCertificate, result, target, tool *topo.Body,
	warning opdiagnostic.Record, budget *proofbound.ReplayBudget) (float64, error) {
	if cert == nil || result == nil || target == nil || tool == nil || budget == nil || !cert.sealed ||
		result.PrivateShift != cert || result.Origin.Producer != cert.resultID || !cert.IsPureGrowth() ||
		!sameShiftWarning(warning, cert.publicWarning) {
		return 0, errShiftAuthGrownCutCertificateIsMissing
	}
	return cert.AuthenticatePrivateShift(ctx, target, tool, meshbool.OpCut, []opdiagnostic.Record{warning}, true, budget)
}

func authenticateStoredGrowthSnapshotCopy(ctx context.Context, cert *ShiftCertificate, result, target, tool *topo.Body,
	warning opdiagnostic.Record, budget *proofbound.ReplayBudget) (float64, error) {
	if cert == nil || result == nil || target == nil || tool == nil || budget == nil || !cert.sealed ||
		result.Origin.Producer != cert.resultID || !cert.IsPureGrowth() ||
		!sameShiftWarning(warning, cert.publicWarning) {
		return 0, errShiftAuthGrownCutCertificateIsMissing
	}
	return cert.AuthenticatePrivateShift(ctx, target, tool, meshbool.OpCut, []opdiagnostic.Record{warning}, true, budget)
}

// ReplayBox encloses the true body and its held mesh in exact dyadics.
type ReplayBox struct{ TrueLo, TrueHi, HeldLo, HeldHi [3]*big.Rat }

// Box returns the authenticated shifted tool's true and held enclosures.
func (t *FixedShiftedTool) Box(ctx context.Context, budget *proofbound.ReplayBudget) (ReplayBox, error) {
	if t == nil {
		return ReplayBox{}, errShiftAuthShiftedToolIsMissing
	}
	return replayBoxes(ctx, t.tool, t.charged, t.move, budget)
}

// ReplayBodyBoxes encloses a body and its already verified held mesh.
func ReplayBodyBoxes(ctx context.Context, body *topo.Body, mesh *topo.Mesh, budget *proofbound.ReplayBudget) (ReplayBox, error) {
	return replayBoxes(ctx, body, mesh, r3.Vec{}, budget)
}

func replayBoxes(ctx context.Context, body *topo.Body, mesh *topo.Mesh, move r3.Vec, budget *proofbound.ReplayBudget) (ReplayBox, error) {
	if body == nil || mesh == nil || budget == nil || len(mesh.Vertices) == 0 || !proofbound.FiniteVec(move) || !proofbound.FiniteVec(body.Bounds.Min) || !proofbound.FiniteVec(body.Bounds.Max) || proofbound.IsNonFinite(body.Bounds.Bound.Base()) || body.Bounds.Bound.Base() < 0 || proofbound.IsNonFinite(mesh.Bound) || mesh.Bound < 0 {
		return ReplayBox{}, errShiftAuthNoFiniteReplayEnclosure
	}
	if err := budget.ReserveRecords(ctx, 256); err != nil {
		return ReplayBox{}, err
	}
	if err := budget.SpendWork(ctx, 3*uint64(len(mesh.Vertices))+24); err != nil {
		return ReplayBox{}, err
	}
	lo, hi := [3]float64{mesh.Vertices[0].X, mesh.Vertices[0].Y, mesh.Vertices[0].Z}, [3]float64{mesh.Vertices[0].X, mesh.Vertices[0].Y, mesh.Vertices[0].Z}
	for _, v := range mesh.Vertices {
		if !proofbound.FiniteVec(v) {
			return ReplayBox{}, errShiftAuthNonfiniteReplayVertex
		}
		for i, x := range [3]float64{v.X, v.Y, v.Z} {
			lo[i] = min(lo[i], x)
			hi[i] = max(hi[i], x)
		}
	}
	out := ReplayBox{}
	lift := func(x float64) (*big.Rat, error) { return proof.ReplayFloatRat(ctx, x, budget) }
	calc := func(a, b, c float64, op byte) (*big.Rat, error) {
		x, err := lift(a)
		if err != nil {
			return nil, err
		}
		y, err := lift(b)
		if err != nil {
			return nil, err
		}
		z, err := lift(c)
		if err != nil {
			return nil, err
		}
		x, err = proof.ReplayRatArithmetic(ctx, x, y, '+', budget)
		if err != nil {
			return nil, err
		}
		return proof.ReplayRatArithmetic(ctx, x, z, op, budget)
	}
	for i := range 3 {
		bl, bh, delta := [3]float64{body.Bounds.Min.X, body.Bounds.Min.Y, body.Bounds.Min.Z}[i], [3]float64{body.Bounds.Max.X, body.Bounds.Max.Y, body.Bounds.Max.Z}[i], [3]float64{move.X, move.Y, move.Z}[i]
		var err error
		out.TrueLo[i], err = calc(bl, delta, body.Bounds.Bound.Base(), '-')
		if err != nil {
			return ReplayBox{}, err
		}
		out.TrueHi[i], err = calc(bh, delta, body.Bounds.Bound.Base(), '+')
		if err != nil {
			return ReplayBox{}, err
		}
		out.HeldLo[i], err = calc(lo[i], 0, mesh.Bound, '-')
		if err != nil {
			return ReplayBox{}, err
		}
		out.HeldHi[i], err = calc(hi[i], 0, mesh.Bound, '+')
		if err != nil {
			return ReplayBox{}, err
		}
	}
	return out, nil
}

// ReplayBoxesDisjointBounded checks both strict gaps with charged products.
func ReplayBoxesDisjointBounded(ctx context.Context, a, b ReplayBox, meter *proofbound.ReplayBudget) (bool, error) {
	if meter == nil {
		return false, errShiftAuthMissingReplayBoxMeter
	}
	trueGap, heldGap := false, false
	for i := range 3 {
		for _, p := range [][2]*big.Rat{{a.TrueHi[i], b.TrueLo[i]}, {b.TrueHi[i], a.TrueLo[i]}, {a.HeldHi[i], b.HeldLo[i]}, {b.HeldHi[i], a.HeldLo[i]}} {
			if p[0] == nil || p[1] == nil {
				return false, nil
			}
		}
		pairs := [4][2]*big.Rat{{a.TrueHi[i], b.TrueLo[i]}, {b.TrueHi[i], a.TrueLo[i]}, {a.HeldHi[i], b.HeldLo[i]}, {b.HeldHi[i], a.HeldLo[i]}}
		for j, p := range pairs {
			v, err := proof.ReplayRatCompare(ctx, p[0], p[1], meter)
			if err != nil {
				return false, err
			}
			if v < 0 {
				if j < 2 {
					trueGap = true
				} else {
					heldGap = true
				}
			}
		}
	}
	return trueGap && heldGap, nil
}

// ReplayBoxesDisjoint requires strict separation of both enclosures.
func ReplayBoxesDisjoint(a, b ReplayBox) bool {
	trueGap, heldGap := false, false
	for i := range 3 {
		if a.TrueLo[i] == nil || a.TrueHi[i] == nil || a.HeldLo[i] == nil || a.HeldHi[i] == nil || b.TrueLo[i] == nil || b.TrueHi[i] == nil || b.HeldLo[i] == nil || b.HeldHi[i] == nil {
			return false
		}
		trueGap = trueGap || a.TrueHi[i].Cmp(b.TrueLo[i]) < 0 || b.TrueHi[i].Cmp(a.TrueLo[i]) < 0
		heldGap = heldGap || a.HeldHi[i].Cmp(b.HeldLo[i]) < 0 || b.HeldHi[i].Cmp(a.HeldLo[i]) < 0
	}
	return trueGap && heldGap
}

func sameShiftWarning(a, b opdiagnostic.Record) bool {
	return a.Operation == b.Operation && a.PublicOperation == b.PublicOperation && a.Code == b.Code && a.Operand == b.Operand && a.Change == b.Change && a.AutomaticAllowance == b.AutomaticAllowance && a.ChainAllowance == nil && b.ChainAllowance == nil &&
		math.Float64bits(a.DisplacementBound) == math.Float64bits(b.DisplacementBound) && math.Float64bits(a.AllowedReach) == math.Float64bits(b.AllowedReach) && math.Float64bits(a.TouchedBound) == math.Float64bits(b.TouchedBound) && math.Float64bits(a.PairChordLimit) == math.Float64bits(b.PairChordLimit)
}

// AuthenticateCutTranslation checks the accepted warning and recorded chord
// before replaying the exact selected tool mesh.
func (c *ShiftCertificate) AuthenticateCutTranslation(ctx context.Context, a, b *topo.Body, selected *topo.Mesh,
	chord, scale float64, warning opdiagnostic.Record, budget *proofbound.ReplayBudget) (*topo.Mesh, float64, error) {
	if c == nil || warning.Operation != int(meshbool.OpCut) || warning.Code != opdiagnostic.CodeGapShiftApplied ||
		warning.Operand != 1 || warning.Change != opdiagnostic.ChangeGapShift ||
		math.Float64bits(warning.DisplacementBound) != math.Float64bits(c.displacement) ||
		math.Float64bits(warning.AllowedReach) != math.Float64bits(c.reach) ||
		math.Float64bits(chord) != math.Float64bits(c.chord) || math.Float64bits(scale) != math.Float64bits(c.scale) {
		return nil, 0, errShiftAuthShiftedCutWarningOrChordChanged
	}
	return c.replayTranslation(ctx, a, b, selected, budget)
}

// AuthenticatePrivateShift validates and replays one accepted private shift.
// GrowthOnly admits the clevis tool's one positive pure growth; otherwise the
// caller accepts the single finite adjustment selected by the Union.
func (c *ShiftCertificate) AuthenticatePrivateShift(ctx context.Context, a, b *topo.Body, op meshbool.OperationKind,
	warnings []opdiagnostic.Record, growthOnly bool, budget *proofbound.ReplayBudget) (float64, error) {
	if c == nil || budget == nil || a == nil || b == nil || len(warnings) > 2 || len(warnings) == 0 {
		return 0, errShiftAuthPrivateShiftEvidenceMissing
	}
	if growthOnly && (c.shift.Move != (r3.Vec{}) || !(c.shift.Grow > 0)) {
		return 0, errShiftAuthPrivateShiftIsNotPositivePureGrowth
	}
	var count, chains int
	for _, w := range warnings {
		switch {
		case w.Code == opdiagnostic.CodeGapShiftApplied && w.Operation == int(op) && w.Operand == 1 && w.Change == opdiagnostic.ChangeGapShift && math.Float64bits(w.DisplacementBound) == math.Float64bits(c.displacement) && math.Float64bits(w.AllowedReach) == math.Float64bits(c.reach):
			count++
		case !growthOnly && w.Code == opdiagnostic.CodeChainAllowanceApplied && w.Operation == int(op) && w.Operand == 1 && w.ChainAllowance != nil:
			chains++
			increment := c.charged.Bound - c.selected.Bound
			allowance := proofbound.AbsSumUpper(c.chord, increment)
			if chains != 1 || increment < 0 || proofbound.IsNonFinite(increment) || math.Float64bits(w.PairChordLimit) != math.Float64bits(c.chord) || math.Float64bits(*w.ChainAllowance) != math.Float64bits(allowance) || !(w.TouchedBound > c.chord) || w.TouchedBound > allowance {
				return 0, errShiftAuthPrivateShiftGateChanged
			}
		default:
			return 0, errShiftAuthPrivateShiftWarningChanged
		}
	}
	if count != 1 || c.displacement > c.reach {
		return 0, errShiftAuthPrivateShiftExceedsItsPermittedReach
	}
	tol, dPair, err := booloperand.PairChordReplay(ctx, a, b, budget)
	if err != nil {
		return 0, err
	}
	pair := tol * c.scale
	if math.Float64bits(pair) != math.Float64bits(c.chord) {
		return 0, errShiftAuthPrivateShiftPairChordChanged
	}
	chord, err := operandChordReplay(ctx, b, pair, dPair, c.scale, budget)
	if err != nil {
		return 0, err
	}
	selected, _, err := (replayMesher{budget: budget}).TessellateOperand(ctx, b, chord)
	if err != nil {
		return 0, err
	}
	charged, d, err := c.replayExactShift(ctx, a, b, selected, budget)
	if err != nil {
		return 0, err
	}
	increment := charged.Bound - selected.Bound
	if increment < 0 || proofbound.IsNonFinite(increment) {
		return 0, errShiftAuthShiftedMeshBoundDecreased
	}
	return max(d, increment), nil
}

// replayTranslation authenticates the source identities and every saved mesh
// field before returning the charged copy of a pure translation.
func (c *ShiftCertificate) replayTranslation(ctx context.Context, a, b *topo.Body, selected *topo.Mesh,
	budget *proofbound.ReplayBudget) (*topo.Mesh, float64, error) {
	if c == nil || !c.IsTranslation() {
		return nil, 0, errShiftAuthShiftCertificateIsNotATranslation
	}
	return c.replayExactShift(ctx, a, b, selected, budget)
}

func (c *ShiftCertificate) replayExactShift(ctx context.Context, a, b *topo.Body, selected *topo.Mesh, budget *proofbound.ReplayBudget) (*topo.Mesh, float64, error) {
	if c == nil || budget == nil || a == nil || b == nil || selected == nil || a != c.target || b != c.tool || a.Origin.Producer != c.targetID || b.Origin.Producer != c.toolID {
		return nil, 0, errShiftAuthShiftCertificateIdentityOrSourceChanged
	}
	if err := budget.SpendWork(ctx, uint64(len(c.selected.Vertices)+len(c.selected.Triangles)+len(c.selected.VertexBound)+len(c.selected.Source)+len(c.selected.FaceBound))); err != nil {
		return nil, 0, err
	}
	same, err := sameShiftMeshReplay(ctx, selected, c.selected, budget)
	if err != nil {
		return nil, 0, err
	}
	if !same {
		return nil, 0, errShiftAuthShiftCertificateSourceChanged
	}
	if err := budget.SpendWork(ctx, 2*uint64(len(selected.Vertices))); err != nil {
		return nil, 0, err
	}
	center, radius, factor := shiftGeometry(selected, c.component, c.shift)
	if math.Float64bits(center.X) != math.Float64bits(c.center.X) || math.Float64bits(center.Y) != math.Float64bits(c.center.Y) || math.Float64bits(center.Z) != math.Float64bits(c.center.Z) || math.Float64bits(radius) != math.Float64bits(c.radius) || math.Float64bits(factor) != math.Float64bits(c.factor) {
		return nil, 0, errShiftAuthShiftGeometryChanged
	}
	ordered, err := newReplayShifts(ctx, budget)
	if err != nil {
		return nil, 0, err
	}
	if err := ordered.set(ctx, c.shift); err != nil {
		return nil, 0, err
	}
	decision, charged, d, err := shiftOperandMeshReplay(ctx, c.selected, c.component, ordered, nil, budget)
	if err != nil {
		return nil, 0, err
	}
	comparison := uint64(len(c.decision.Vertices) + len(c.decision.Triangles) + len(c.decision.VertexBound) + len(c.decision.Source) + len(c.decision.FaceBound) + len(c.charged.Vertices) + len(c.charged.Triangles) + len(c.charged.VertexBound) + len(c.charged.Source) + len(c.charged.FaceBound))
	if err := budget.SpendWork(ctx, comparison); err != nil {
		return nil, 0, err
	}
	sameDecision, err := sameShiftMeshReplay(ctx, decision, c.decision, budget)
	if err != nil {
		return nil, 0, err
	}
	sameCharged, err := sameShiftMeshReplay(ctx, charged, c.charged, budget)
	if err != nil {
		return nil, 0, err
	}
	if math.Float64bits(d) != math.Float64bits(c.displacement) || !sameDecision || !sameCharged {
		return nil, 0, errShiftAuthShiftCertificateReplayChanged
	}
	return charged, d, nil
}

func sameShiftMesh(a, b *topo.Mesh) bool {
	if a == nil || b == nil || len(a.Vertices) != len(b.Vertices) || len(a.Triangles) != len(b.Triangles) || len(a.VertexBound) != len(b.VertexBound) || len(a.Source) != len(b.Source) || len(a.FaceBound) != len(b.FaceBound) ||
		(a.Vertices == nil) != (b.Vertices == nil) || (a.Triangles == nil) != (b.Triangles == nil) || (a.VertexBound == nil) != (b.VertexBound == nil) || (a.Source == nil) != (b.Source == nil) || (a.FaceBound == nil) != (b.FaceBound == nil) ||
		math.Float64bits(a.Bound) != math.Float64bits(b.Bound) || math.Float64bits(a.CoordBound) != math.Float64bits(b.CoordBound) || math.Float64bits(a.AreaSlack) != math.Float64bits(b.AreaSlack) || math.Float64bits(a.VolSymDiff) != math.Float64bits(b.VolSymDiff) || a.SymDiffOK != b.SymDiffOK || a.BoundaryOK != b.BoundaryOK {
		return false
	}
	for i := range a.Vertices {
		x, y := a.Vertices[i], b.Vertices[i]
		if math.Float64bits(x.X) != math.Float64bits(y.X) || math.Float64bits(x.Y) != math.Float64bits(y.Y) || math.Float64bits(x.Z) != math.Float64bits(y.Z) {
			return false
		}
	}
	for i := range a.Triangles {
		if a.Triangles[i] != b.Triangles[i] {
			return false
		}
	}
	for i := range a.VertexBound {
		if math.Float64bits(a.VertexBound[i]) != math.Float64bits(b.VertexBound[i]) {
			return false
		}
	}
	for i := range a.Source {
		if a.Source[i] != b.Source[i] {
			return false
		}
	}
	for f, v := range a.FaceBound {
		x, ok := b.FaceBound[f]
		if !ok || math.Float64bits(v) != math.Float64bits(x) {
			return false
		}
	}
	return true
}

// ShiftedBoxes encloses the authenticated tool after its private adjustment.
// The true enclosure is expanded by the conservative replay movement charge.
func (c *ShiftCertificate) ShiftedBoxes(ctx context.Context, tool *topo.Body, charge float64,
	budget *proofbound.ReplayBudget) (ReplayBox, error) {
	if c == nil || tool == nil || c.tool != tool || tool.Origin.Producer != c.toolID ||
		proofbound.IsNonFinite(charge) || charge < c.displacement || c.charged == nil || c.selected == nil ||
		charge < c.charged.Bound-c.selected.Bound {
		return ReplayBox{}, errShiftAuthInvalidPrivateShiftEnclosure
	}
	box, err := replayBoxes(ctx, tool, c.charged, r3.Vec{}, budget)
	if err != nil {
		return ReplayBox{}, err
	}
	if err := budget.SpendWork(ctx, 4096); err != nil {
		return ReplayBox{}, err
	}
	if err := budget.ReserveRecords(ctx, 48); err != nil {
		return ReplayBox{}, err
	}
	r, err := proof.ReplayFloatRat(ctx, charge, budget)
	if err != nil {
		return ReplayBox{}, err
	}
	for i := range 3 {
		box.TrueLo[i], err = proof.ReplayRatArithmetic(ctx, box.TrueLo[i], r, '-', budget)
		if err != nil {
			return ReplayBox{}, err
		}
		box.TrueHi[i], err = proof.ReplayRatArithmetic(ctx, box.TrueHi[i], r, '+', budget)
		if err != nil {
			return ReplayBox{}, err
		}
	}
	return box, nil
}

// Allowance returns the fixed tool's separate full-bound contact gate.
func (t *FixedShiftedTool) Allowance(base float64) (float64, bool) {
	if t == nil || t.selected == nil || t.charged == nil || !(base > 0) || proofbound.IsNonFinite(base) {
		return 0, false
	}
	delta := t.charged.Bound - t.selected.Bound
	if delta < 0 || proofbound.IsNonFinite(delta) {
		return 0, false
	}
	allowance := proofbound.AbsSumUpper(base, delta)
	return allowance, !proofbound.IsNonFinite(allowance)
}

// sameShiftMeshReplay compares map and fixed-table bounds without conversion.
func sameShiftMeshReplay(ctx context.Context, a, b *topo.Mesh, budget *proofbound.ReplayBudget) (bool, error) {
	if a == nil || b == nil || budget == nil || a.ReplayFaceBoundCount() != b.ReplayFaceBoundCount() {
		return false, nil
	}
	x, y := *a, *b
	x.FaceBound, y.FaceBound = nil, nil
	x.ReplayFaceBounds, y.ReplayFaceBounds = nil, nil
	if !sameShiftMesh(&x, &y) {
		return false, nil
	}
	match := true
	err := a.EachReplayFaceBound(ctx, budget, func(face *topo.Face, bound float64) error {
		other, ok, err := b.ReplayFaceBound(ctx, face, budget)
		if err != nil {
			return err
		}
		if !ok || math.Float64bits(bound) != math.Float64bits(other) {
			match = false
		}
		return nil
	})
	return match, err
}

// cloneShiftMeshReplay copies a mesh and moves its bounds into a fixed table.
func cloneShiftMeshReplay(ctx context.Context, m *topo.Mesh, budget *proofbound.ReplayBudget) (*topo.Mesh, error) {
	if m == nil || budget == nil {
		return nil, errShiftAuthMissingReplayMesh
	}
	var count uint64
	for _, n := range []int{len(m.Vertices), len(m.Triangles), len(m.VertexBound), len(m.Source)} {
		if uint64(n) > proofbound.MaxMultiwayRecordsPerCall-count {
			return nil, errShiftAuthRetainedMeshExceedsRecordLimit
		}
		count += uint64(n)
	}
	if err := budget.SpendWork(ctx, count); err != nil {
		return nil, err
	}
	if err := budget.ReserveRecords(ctx, count); err != nil {
		return nil, err
	}
	for _, item := range [][2]uint64{
		{uint64(len(m.Vertices)), uint64(unsafe.Sizeof(r3.Vec{}))},
		{uint64(len(m.Triangles)), uint64(unsafe.Sizeof([3]int{}))},
		{uint64(len(m.VertexBound)), uint64(unsafe.Sizeof(float64(0)))},
		{uint64(len(m.Source)), uint64(unsafe.Sizeof((*topo.Face)(nil)))},
		{1, uint64(unsafe.Sizeof(topo.Mesh{}))},
	} {
		if err := budget.ReserveArray(ctx, item[0], item[1]); err != nil {
			return nil, err
		}
	}
	faceCount := m.ReplayFaceBoundCount()
	table, err := topo.NewFaceBoundTable(ctx, faceCount, budget)
	if err != nil {
		return nil, err
	}
	if err := m.EachReplayFaceBound(ctx, budget, func(face *topo.Face, bound float64) error { return table.Set(ctx, face, bound, budget) }); err != nil {
		return nil, err
	}
	out := *m
	out.Owner = nil
	out.Vertices = slices.Clone(m.Vertices)
	out.Triangles = slices.Clone(m.Triangles)
	out.VertexBound = slices.Clone(m.VertexBound)
	out.Source = slices.Clone(m.Source)
	out.FaceBound = nil
	out.ReplayFaceBounds = table
	return &out, nil
}

// AuthenticateStoredCutSnapshot authenticates only the revalidated private copy.
func AuthenticateStoredCutSnapshot(ctx context.Context, snapshot *ShiftCertificateSnapshot, result, target, tool *topo.Body, warning opdiagnostic.Record, budget *proofbound.ReplayBudget) (*FixedShiftedTool, error) {
	cert, err := snapshot.Certificate(ctx, budget, result, target, tool)
	if err != nil {
		return nil, err
	}
	return authenticateStoredCutSnapshotCopy(ctx, cert, result, target, tool, warning, budget)
}

// RetainedCutTargetSnapshot reads the target from the revalidated private copy.
func RetainedCutTargetSnapshot(ctx context.Context, snapshot *ShiftCertificateSnapshot, result, target, tool *topo.Body, budget *proofbound.ReplayBudget) (*RetainedTarget, error) {
	cert, err := snapshot.Certificate(ctx, budget, result, target, tool)
	if err != nil {
		return nil, err
	}
	return retainedCutTargetSnapshotCopy(ctx, cert, result, target, budget)
}

// AuthenticateStoredGrowthSnapshot checks growth from a revalidated private copy.
func AuthenticateStoredGrowthSnapshot(ctx context.Context, snapshot *ShiftCertificateSnapshot, result, target, tool *topo.Body, warning opdiagnostic.Record, budget *proofbound.ReplayBudget) (float64, error) {
	cert, err := snapshot.Certificate(ctx, budget, result, target, tool)
	if err != nil {
		return 0, err
	}
	return authenticateStoredGrowthSnapshotCopy(ctx, cert, result, target, tool, warning, budget)
}

// RevalidateStoredWarning checks the admitted Cut and warning before publication.
func RevalidateStoredWarning(ctx context.Context, snapshot *ShiftCertificateSnapshot, result, target, tool *topo.Body,
	warning opdiagnostic.Record, budget *proofbound.ReplayBudget) error {
	cert, err := snapshot.Certificate(ctx, budget, result, target, tool)
	if err != nil {
		return err
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return err
	}
	if len(result.OperationWarnings) != 1 || !sameShiftWarning(warning, result.OperationWarnings[0]) || !sameShiftWarning(warning, cert.publicWarning) {
		return errShiftAuthStoredCutWarningChanged
	}
	return nil
}
