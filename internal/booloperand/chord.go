// Package booloperand reads what the mesh boolean (agent-docs/evaluator-design.md
// §9) needs from one operand's own record before it meshes it: the operand's
// share of the pair chord tolerance, the held floor a restating operand cannot
// be meshed below, and whether the operand's payload class can prove the
// volume its mesh and the body differ by. It reads topology nodes only;
// internal/meshbool owns the pipeline that consumes these answers.
package booloperand

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/coilshell"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/revolvetess"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/sweepmitre"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/twistsweep"
	"github.com/lestrrat-3d/r3"
)

// HeldFloor returns the bound below which a RESTATING operand cannot give a
// mesh (agent-docs/faceted-vertex-bounds-design.md §5), and whether b restates one:
// a boolean result's MeshBound, or a mitred sweep's, a twisted sweep's or a
// coil's delta (agent-docs/helix-design.md Table CD row CD3). Its restatement
// returns the same vertices at any tolerance at or above the floor. Every
// other payload reports false, and its request stays the pair tolerance and
// the local gate does not read its facets.
func HeldFloor(b *topo.Body) (float64, bool) {
	switch p := b.Payload.(type) {
	case facetedrecord.Payload:
		return p.MeshBound, true
	case sweepmitre.Payload:
		return p.Delta, true
	case coilshell.Payload:
		return p.Delta, true
	case twistsweep.Payload:
		return p.Delta, true
	default:
		return 0, false
	}
}

// PairChord derives the evaluator-internal chord tolerance and the pair
// diameter from the operands' own bounds boxes (inflated by their proven
// bounds), each raised past the operand's ChordShare floor. All-planar
// operands chord nothing regardless, so the tolerance only shapes curved
// boundaries. A pair with no finite positive extent is ErrDegenerate.
func PairChord(ctx context.Context, a, b *topo.Body) (float64, float64, error) {
	tol, diameter, ok := meshbool.PairChordFrom(ChordShare(ctx, a), ChordShare(ctx, b))
	if !ok {
		return 0, 0, fmt.Errorf(`%w: the operand pair has no extent to derive a chord tolerance from`, decaderr.ErrDegenerate)
	}
	return tol, diameter, nil
}

// ChordShare reads b's share of PairChord: its bounds box inflated by the
// box's own proven bound, and the floor below which the pair tolerance may not
// fall.
//
// The floor raises the pair's tolerance past the operand's own reservations.
// An operand holding its section within a displacement of the one it denotes
// cannot be meshed within that displacement — tessellation reserves it from
// the requested tolerance (agent-docs/tessellation-design.md §5) — and this mesh
// path is the designated fallback for exactly those operands
// (agent-docs/prism-boolean-design.md §3.4). A diameter-derived tolerance under the
// displacement would refuse them. A revolve likewise reserves both of its
// coordinate stages out of the tolerance before it chords anything
// (agent-docs/tessellation-design.md §8), so a diameter-derived tolerance under
// that reservation refuses a body whose own geometry is perfectly ordinary —
// a small part modelled at a large coordinate is the case that reaches it.
// Nothing is lost by the coarser chording: each mesh reports the sagitta it
// actually took, and the reserved figure already dominates the bound it
// publishes.
func ChordShare(ctx context.Context, b *topo.Body) meshbool.ChordOperand {
	box := b.Bounds
	inf := box.Bound.Base()
	pad := r3.NewVec(inf, inf, inf)
	return meshbool.ChordOperand{
		Lo: box.Min.Sub(pad),
		Hi: box.Max.Add(pad),
		Floor: max(
			proofbound.ProductUpper(2, sectionDisplacement(b.Payload)),
			proofbound.ProductUpper(2, coordDisplacement(ctx, b.Payload)),
			heldPrimitiveFloor(b.Payload),
		),
	}
}

// sectionDisplacement is the proven displacement between a body's recorded
// boundary and the boundary its construction denotes: the analytic prism
// boolean's re-expression rounding and its surviving fragments' own cut
// parameters, and zero for every other body (agent-docs/prism-boolean-design.md §7).
func sectionDisplacement(payload topo.Payload) float64 {
	switch p := payload.(type) {
	case prismrecord.Payload:
		return p.SectionDelta
	case stackedrecord.Payload:
		return p.SectionDelta
	case breprecord.Payload:
		return p.SectionDelta()
	default:
		return 0
	}
}

// heldPrimitiveFloor is the proven held bound of an operand built as a fixed
// held mesh in one construction — a LoftFaces MeshBound, or a coil's, mitred
// sweep's, or twisted sweep's delta (agent-docs/faceted-vertex-bounds-design.md §5).
// Its restatement cannot be meshed closer than that, and every facet it
// touches the partner with holds at most that bound, so raising the pair
// tolerance to it is what lets the chain-depth gate admit the pair. A boolean
// result's MeshBound is not one: it grows with the chain, which is what the
// gate exists to refuse. Every other payload answers zero.
func heldPrimitiveFloor(payload topo.Payload) float64 {
	switch p := payload.(type) {
	case facetedrecord.Payload:
		if p.LoftFacesPrimitive {
			return p.MeshBound
		}
		return 0
	case coilshell.Payload:
		return p.Delta
	case sweepmitre.Payload:
		return p.Delta
	case twistsweep.Payload:
		return p.Delta
	default:
		return 0
	}
}

// coordDisplacement is the count-INDEPENDENT coordinate reservation an
// operand's own tessellation spends before it chords anything: for a revolve,
// agent-docs/tessellation-design.md §8's deltaC and deltaR ceilings composed
// (revolvetess.ResolveRevolve). Every other payload class answers zero, and so
// does a revolve whose resolution fails — the tessellation that follows
// refuses it with its own sentinel, and inventing a tolerance here would only
// hide which one.
func coordDisplacement(ctx context.Context, payload topo.Payload) float64 {
	rp, ok := payload.(revolveplan.Payload)
	if !ok {
		return 0
	}
	res, err := revolvetess.ResolveRevolve(ctx, rp)
	if err != nil {
		return 0
	}
	if rp.SectionDelta > 0 {
		// The revolve tessellator reserves its section displacement beside the
		// two coordinate stages (planRevolveWithRoles), so all three come out of one
		// tolerance.
		return proofbound.AbsSumUpper(res.DeltaCPrior, res.DeltaRPrior, rp.SectionDelta)
	}
	return proofbound.AbsSumUpper(res.DeltaCPrior, res.DeltaRPrior)
}
