package booleaneval

import (
	"context"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/gaptol"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// ShiftCertificate keeps private copies of the selected, decision and
// charged meshes together with the exact shift and source identities.
type ShiftCertificate struct {
	target, tool                      *topo.Body
	targetID, toolID                  provenance.Producer
	selected, decision, charged       *topo.Mesh
	targetMesh                        *topo.Mesh
	targetChord                       float64
	component                         pieces
	shift                             pieceShift
	groups                            []facetedrecord.Group
	captureSeal                       *shiftCaptureSeal
	center                            r3.Vec
	radius, factor                    float64
	chord, scale, displacement, reach float64
	resultID                          provenance.Producer
	publicWarning                     opdiagnostic.Record
	sealed                            bool
}

// IsTranslation reports the narrow public-Cut capture class.
func (c *ShiftCertificate) IsTranslation() bool {
	return c != nil && c.shift.Move != (r3.Vec{}) && c.shift.Grow == 0
}

// IsPureGrowth reports a positive one-piece growth with no translation.
func (c *ShiftCertificate) IsPureGrowth() bool {
	return c != nil && c.shift.Move == (r3.Vec{}) && c.shift.Grow > 0 && !proofbound.IsNonFinite(c.shift.Grow)
}

// SealPublicCutWarning binds the unique shift warning to the committed Cut.
func (c *ShiftCertificate) SealPublicCutWarning(result provenance.Producer, warnings []opdiagnostic.Record, publicOperation int) bool {
	if c == nil || c.sealed || (!c.IsTranslation() && !c.IsPureGrowth()) {
		return false
	}
	var shift opdiagnostic.Record
	found := false
	for _, w := range warnings {
		if w.Code != opdiagnostic.CodeGapShiftApplied {
			continue
		}
		if found || w.Operation != int(meshbool.OpCut) || w.PublicOperation != publicOperation || w.Operand != 1 || w.Change != opdiagnostic.ChangeGapShift ||
			w.ChainAllowance != nil || math.Float64bits(w.TouchedBound) != 0 || math.Float64bits(w.PairChordLimit) != 0 ||
			math.Float64bits(w.DisplacementBound) != math.Float64bits(c.displacement) || math.Float64bits(w.AllowedReach) != math.Float64bits(c.reach) {
			return false
		}
		shift, found = w, true
	}
	if !found {
		return false
	}
	c.resultID = result
	c.publicWarning = shift
	c.sealed = true
	return true
}

func captureOrdinaryShiftMesh(ctx context.Context, mesh *topo.Mesh, budget *proofbound.ReplayBudget) (*topo.Mesh, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if budget == nil {
		return nil, decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return nil, err
	}
	if mesh == nil || mesh.ReplayFaceBounds != nil {
		return nil, decaderr.ErrUnsupported
	}
	return snapshotShiftMesh(ctx, mesh, budget)
}

func captureShiftCertificate(ctx context.Context, a, b *topo.Body, selected, decision, charged *topo.Mesh, pc pieces, groups []facetedrecord.Group, shifts map[int]pieceShift, chord, scale, displacement float64) *ShiftCertificate {
	trial := proofbound.NewReplayTrialBudget()
	if err := trial.SpendWork(ctx, 16); err != nil {
		return nil
	}
	if a == nil || b == nil || pc.count != 1 || len(shifts) != 1 || selected == nil || decision == nil || charged == nil {
		return nil
	}
	if len(pc.ofVertex) != len(selected.Vertices) || len(pc.ofFacet) != len(selected.Triangles) {
		return nil
	}
	shift, ok := shifts[0]
	if !ok || !proofbound.FiniteVec(shift.Move) || proofbound.IsNonFinite(shift.Grow) {
		return nil
	}
	var slots uint64
	add := func(n int) bool {
		if n < 0 || uint64(n) > proofbound.MaxMultiwayRecordsPerCall-slots {
			return false
		}
		slots += uint64(n)
		return true
	}
	for _, m := range [3]*topo.Mesh{selected, decision, charged} {
		if len(m.Vertices) > 65536 || len(m.Triangles) > 65536 {
			return nil
		}
		for _, n := range []int{len(m.Vertices), len(m.Triangles), len(m.VertexBound), len(m.Source), len(m.FaceBound), 1} {
			if !add(n) {
				return nil
			}
		}
	}
	for _, n := range []int{len(pc.ofVertex), len(pc.ofFacet), len(groups), 1} {
		if !add(n) {
			return nil
		}
	}
	for i := range groups {
		if err := trial.SpendWork(ctx, 1); err != nil {
			return nil
		}
		g := groups[i]
		if !add(len(g.Origins)) || !add(len(g.Upstream)) {
			return nil
		}
	}

	if err := trial.SpendWork(ctx, 2*uint64(len(selected.Vertices))); err != nil {
		return nil
	}
	center, radius, factor := shiftGeometry(selected, pc, shift)
	selectedCopy, err := captureOrdinaryShiftMesh(ctx, selected, trial)
	if err != nil {
		return nil
	}
	decisionCopy, err := captureOrdinaryShiftMesh(ctx, decision, trial)
	if err != nil {
		return nil
	}
	chargedCopy, err := captureOrdinaryShiftMesh(ctx, charged, trial)
	if err != nil {
		return nil
	}
	vertices, err := snapshotShiftSlice(ctx, pc.ofVertex, trial)
	if err != nil {
		return nil
	}
	facets, err := snapshotShiftSlice(ctx, pc.ofFacet, trial)
	if err != nil {
		return nil
	}
	copied, err := snapshotShiftGroups(ctx, groups, trial)
	if err != nil {
		return nil
	}
	seal, err := captureShiftGroupSeal(ctx, copied, trial)
	if err != nil {
		return nil
	}
	if err := trial.ReserveRecords(ctx, 1); err != nil {
		return nil
	}
	if err := trial.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(ShiftCertificate{}))); err != nil {
		return nil
	}
	return &ShiftCertificate{target: a, tool: b, targetID: a.Origin.Producer, toolID: b.Origin.Producer,
		selected: selectedCopy, decision: decisionCopy, charged: chargedCopy,
		component: pieces{ofVertex: vertices, ofFacet: facets, count: pc.count},
		shift:     shift, groups: copied, captureSeal: seal, center: center, radius: radius, factor: factor, chord: chord, scale: scale,
		displacement: displacement, reach: gaptol.Reach(ctx)}
}

func shiftGeometry(m *topo.Mesh, pc pieces, shift pieceShift) (r3.Vec, float64, float64) {
	if shift.Grow == 0 {
		return r3.Vec{}, 0, 0
	}
	var sum r3.Vec
	var n int
	for i, v := range m.Vertices {
		if pc.ofVertex[i] == 0 {
			sum = sum.Add(v)
			n++
		}
	}
	if n == 0 {
		return r3.Vec{}, 0, 0
	}
	center := sum.Scale(1 / float64(n))
	radius := 0.0
	for i, v := range m.Vertices {
		if pc.ofVertex[i] == 0 {
			radius = math.Max(radius, v.Sub(center).Len())
		}
	}
	if radius == 0 {
		return center, 0, 0
	}
	return center, radius, shift.Grow / radius
}
