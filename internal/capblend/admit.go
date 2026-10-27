package capblend

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// RequireNotReceiver is SX10 (agent-docs/modify-reach-design.md Table SX):
// another modify op on a Payload receiver is staged — the body
// exists, but composing another feature onto a cap-blend result is not built.
// Called before the ordinary prism payload cast in Fillet/Chamfer/Shell so the
// more specific reason leads the generic "not a prism" refusal.
func RequireNotReceiver(payload topo.Payload, op string) error {
	if _, ok := payload.(Payload); ok {
		return fmt.Errorf(`%w: this evaluator does not yet compose another modify op onto a cap-loop chamfer result; %s a receiver this evaluator built directly`, decaderr.ErrUnsupported, op)
	}
	return nil
}

// OccupiedVolumeAdmission lets capband inspect the payload's loops.
// A draft view's corners are read under its own sharp rule (offsetJoins), so
// a reflex line-line corner there is the miter it is built as, never the
// chamfer's apex arc, and its per-walk amounts let capband admit a
// near-tangent line–arc or arc–arc join whose sliver NearTangentVolume
// charges.
// The delayed join builder runs only after the wall checks, preserving the
// proof budget charges and the first refusal reported.
func OccupiedVolumeAdmission(budget *proofbound.WorkBudget, cbp Payload) (error, error) {
	var sharp capband.SharpAmounts
	if cbp.Draft {
		sharp = cbp.sharpAmounts
	}
	return capband.OccupiedVolumeAdmission(budget, cbp.Noun(), cbp.Loops(), cbp.StartLoops, cbp.EndLoops,
		func(li int, loop sectionrecord.LoopRecord) ([]survey2d.SideWalk, func() ([]bool, error), error) {
			cl, err := OneLoopCornerLoop(budget, loop, freeform.NewFreeformWork())
			if err != nil {
				return nil, nil, err
			}
			return cl.Walks, func() ([]bool, error) {
				joins, err := cbp.OffsetJoins(budget, li, cl, cbp.LoopOffset(li))
				if err != nil {
					return nil, err
				}
				arcs := make([]bool, len(joins))
				for i := range joins {
					arcs[i] = joins[i].Arc
				}
				return arcs, nil
			}, nil
		}, sharp)
}

// sharpAmounts is a draft view's per-walk amounts for loop li and the
// half-width of every moved walk's span, the numbers its cap motion reads
// (CapMotion).
func (cbp Payload) sharpAmounts(li int, walks []survey2d.SideWalk) ([]float64, float64) {
	return cbp.WalkAmounts(li, walks, cbp.LoopOffset(li)), cbp.LoopSetback(li).DcDelta
}

// WitnessPrisms lists one witness prism per loop of a cap-loop
// chamfer's receiver section, each over the interval its loop's own wall runs
// on the body. A chamfer band cuts its loop's wall back from the cap level to
// the side level, the cap level moved ds into the material
// (agent-docs/modify-reach-design.md §8.3.1), and leaves the wall between the two
// side levels untouched. A loop chamfered on the start cap therefore starts
// at z0 + ds, one chamfered on the end cap stops at z1 - ds, and an
// unchamfered loop runs from z0 to z1. Those are the float sums the build
// places each band's side level at, and Payload.AxialDelta bounds how
// far any of them sits from the level it denotes. The receiver's walls at the
// cap levels are not points of the body wherever a band cuts them, and a
// reading from them overstates the body's diameter: a 10 mm cube chamfered 2
// mm around its end cap is sqrt(264) across, against the receiver's
// sqrt(300).
func WitnessPrisms(pl Payload) []prismrecord.Payload {
	loops := pl.Loops()
	out := make([]prismrecord.Payload, 0, len(loops))
	for li, loop := range loops {
		witness := pl.PrismLike(pl.Z0, pl.Z1)
		witness.Profile = momentinput.Profile{Outer: loop}
		if pl.StartLoops[li] {
			witness.Z0 = pl.Z0 + pl.Start.Ds
		}
		if pl.EndLoops[li] {
			witness.Z1 = pl.Z1 - pl.End.Ds
		}
		out = append(out, witness)
	}
	return out
}
