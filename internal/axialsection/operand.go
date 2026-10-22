package axialsection

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// Operand is an axial extrusion read as its section and the two cap planes it
// lies between.
type Operand struct {
	Section momentinput.Profile
	// Frame0 and Frame1 are the planes the section is recorded on at each end,
	// and Plane0 and Plane1 their records. A prism has no second plane record of
	// its own, so FromLoft says whether these came from a loft.
	Frame0, Frame1 r3.Frame
	Plane0, Plane1 sectionrecord.PlaneRecord
	FromLoft       bool
	// Level0 and Level1 are the exact axial offsets of the end planes along the
	// frame normal, from Frame0 of the prism or loft.
	Level0, Level1 *big.Rat
}

// OperandOf reads a payload as an axial extrusion: a prism, or a loft whose two
// recorded profiles are identical and whose end planes differ only along their
// shared normal. ok is false for any other payload, or one that carries a
// bore, a blend, a section displacement or a placement.
func OperandOf(ctx context.Context, payload topo.Payload) (Operand, bool, error) {
	switch p := payload.(type) {
	case prismrecord.Payload:
		// A prism's SectionDelta is how far its recorded section sits from the solid it
		// stands for (a record moved by a pattern). This route rebuilds from records and
		// would drop that charge, so a prism that carries one is not an operand. A loft's
		// SectionDelta is its own chord displacement, which a rebuild recomputes.
		if p.SurfaceResult || p.SectionDelta != 0 || p.Z0Delta != 0 || p.Z1Delta != 0 ||
			p.Xform != r3.Identity() || len(p.BlendSegs) != 0 || len(p.Profile.Holes) != 0 ||
			!(p.Z1 > p.Z0) || !proofbound.FiniteVec(r3.Vec{X: p.Z0, Y: p.Z1}) {
			return Operand{}, false, nil
		}
		return Operand{
			Section: p.Profile, Frame0: p.Frame, Frame1: p.Frame,
			Level0: proofarith.FloatRat(p.Z0), Level1: proofarith.FloatRat(p.Z1),
		}, true, nil
	case loftrecord.Payload:
		if !p.AuthenticatedReconstruction || p.SurfaceResult || p.ThroughBoreCut || p.AnalyticBore ||
			p.ConstructionDelta != 0 || len(p.BlendSegs) != 0 || len(p.ConstructionArcUpper) != 0 ||
			len(p.ConstructionLengthExtra) != 0 || p.Xform != r3.Identity() ||
			len(p.Profile0.Holes) != 0 || len(p.Profile1.Holes) != 0 ||
			p.Frame0.U() != p.Frame1.U() || p.Frame0.V() != p.Frame1.V() {
			return Operand{}, false, nil
		}
		for _, offset := range p.Alignment {
			if offset != 0 {
				return Operand{}, false, nil
			}
		}
		budget := proofbound.NewWorkBudget(ctx)
		same, err := momentinput.ExactProfileEqual(budget, p.Profile0, p.Profile1)
		if err != nil || !same {
			return Operand{}, false, err
		}
		axis := prismplacement.SharedAxisOf(
			prismplacement.Operand{Frame: p.Frame0, Xform: p.Xform},
			prismplacement.Operand{Frame: p.Frame1, Xform: p.Xform})
		if !axis.OK || axis.Shift.Sign() <= 0 {
			return Operand{}, false, nil
		}
		return Operand{
			Section: p.Profile0, Frame0: p.Frame0, Frame1: p.Frame1, Plane0: p.Plane0, Plane1: p.Plane1,
			FromLoft: true, Level0: new(big.Rat), Level1: axis.Shift,
		}, true, nil
	}
	return Operand{}, false, nil
}

// Shared reports whether every operand lies between the same two planes, and the
// operand whose loft records those planes. Two operands share planes when their
// frames have equal section axes and their end planes sit at equal exact
// offsets along the normal.
func Shared(ops []Operand) (loft int, ok bool) {
	loft = -1
	ref := ops[0]
	for i, op := range ops {
		axis := prismplacement.SharedAxisOf(
			prismplacement.Operand{Frame: ref.Frame0, Xform: r3.Identity()},
			prismplacement.Operand{Frame: op.Frame0, Xform: r3.Identity()})
		if !axis.OK && op.Frame0.Origin() != ref.Frame0.Origin() {
			return -1, false
		}
		if !axis.OK {
			// Same origin: the shared-axis answer is "no shift", and the axes must agree.
			if op.Frame0.U() != ref.Frame0.U() || op.Frame0.V() != ref.Frame0.V() {
				return -1, false
			}
			axis.Shift = new(big.Rat)
		}
		if new(big.Rat).Add(axis.Shift, op.Level0).Cmp(new(big.Rat).Add(new(big.Rat), ref.Level0)) != 0 ||
			new(big.Rat).Add(axis.Shift, op.Level1).Cmp(ref.Level1) != 0 {
			return -1, false
		}
		if op.FromLoft && loft < 0 {
			loft = i
		}
	}
	return loft, loft >= 0
}
