package capblend

import (
	"context"
	"fmt"
	"math"
	"slices"
	"sync/atomic"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// DisplacedPayload is the cap-loop chamfer of a prism whose recorded section is
// displaced from the section it denotes, built over exact selected loops
// (agent-docs/prism-boolean-design.md Part K §8). Base is the chamfer record as
// Build makes it; LoopDelta[l] is the receiver record's h_loop of loop l (0 the
// outer loop, i+1 hole i), zero on every selected loop: Lemma H's bound on how
// far the denoted loop lies from the recorded one.
//
// The type is the guard. Base is a named field, never embedded, so no method
// of Payload is promoted: a type switch or assertion on Payload never matches
// a DisplacedPayload, and every reader not named in §8.8 gives its existing
// answer for a payload class it does not know. The exported methods are
// exactly Transform, Placed, SurveyClass, ExtentAlong and ExtentBoundedAlong.
type DisplacedPayload struct {
	Base      Payload
	LoopDelta []float64
}

// BuildDisplaced builds the cap-blend body of cbp over a displaced section
// (§8.4). loopDelta holds one entry per loop. It refuses with ErrUnsupported a
// wrong length, an entry that is negative, not finite or nonzero on a selected
// loop, and a base with Draft, Fillet or LoftSource set. With every entry zero
// it builds a plain Payload; otherwise the body's payload is a
// DisplacedPayload that stores loopDelta.
func BuildDisplaced(ctx context.Context, env topo.Env, cbp Payload, loopDelta []float64) (*topo.Body, error) {
	if err := validateLoopDelta(cbp, loopDelta); err != nil {
		return nil, err
	}
	if maxOf(loopDelta) == 0 {
		return build(ctx, env, cbp, nil)
	}
	return build(ctx, env, cbp, loopDelta)
}

func validateLoopDelta(cbp Payload, loopDelta []float64) error {
	if cbp.Draft || cbp.Fillet || cbp.LoftSource != nil {
		return fmt.Errorf(`%w: a displaced cap-loop chamfer takes no draft, fillet or loft-source base`, decaderr.ErrUnsupported)
	}
	if len(loopDelta) != len(cbp.Loops()) {
		return fmt.Errorf(`%w: a displaced cap-loop chamfer takes one displacement per loop, got %d for %d loops`,
			decaderr.ErrUnsupported, len(loopDelta), len(cbp.Loops()))
	}
	for li, d := range loopDelta {
		if proofbound.IsNonFinite(d) || d < 0 {
			return fmt.Errorf(`%w: the displacement of loop %d is %v, not a finite non-negative number`, decaderr.ErrUnsupported, li, d)
		}
		if d != 0 && (cbp.StartLoops[li] || cbp.EndLoops[li]) {
			return fmt.Errorf(`%w: loop %d is chamfered and so must be exact, yet carries displacement %v`, decaderr.ErrUnsupported, li, d)
		}
	}
	return nil
}

// maxOf is the largest entry of xs, zero for none.
func maxOf(xs []float64) float64 {
	m := 0.0
	for _, x := range xs {
		m = math.Max(m, x)
	}
	return m
}

// MaxLoopDelta is the largest per-loop displacement of dp. It is a function,
// not a method, so DisplacedPayload keeps exactly the methods §8.4 lists.
func MaxLoopDelta(dp DisplacedPayload) float64 { return maxOf(dp.LoopDelta) }

// Transform is the accumulated rigid placement.
func (dp DisplacedPayload) Transform() r3.Transform { return dp.Base.Xform }

// Placed re-evaluates the same record under the composed motion and keeps
// LoopDelta: it bounds a plane-local displacement, and the record moves with
// the body, as a prism's placement keeps its SectionDelta (§8.4).
func (dp DisplacedPayload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	base := dp.Base
	base.Xform = composed
	return BuildDisplaced(ctx, env, base, dp.LoopDelta)
}

// SurveyClass names the payload class a staged survey diagnostic reports.
func (DisplacedPayload) SurveyClass() string { return "displacedCapBlendPayload" }

// ExtentAlong refuses, as prismrecord.Payload.ExtentAlong refuses a nonzero
// SectionDelta, so a through-all stop with this body live refuses as it does
// for a displaced prism (§8.6).
func (dp DisplacedPayload) ExtentAlong(r3.Vec) (float64, float64, float64, error) {
	return 0, 0, 0, fmt.Errorf(`%w: a through-all stop cannot read the extent of a cap-loop chamfer whose recorded section is displaced from the section it denotes`, decaderr.ErrUnsupported)
}

// ExtentBoundedAlong is the base reading with each displaced loop's candidates
// charged its LoopDelta, and the placement's coordinate envelope widened by the
// largest one (§8.6).
func (dp DisplacedPayload) ExtentBoundedAlong(ctx context.Context, g r3.Vec, work *freeform.FreeformWork) (float64, float64, float64, error) {
	return dp.Base.extentBoundedAlong(ctx, g, work, nil, nil, dp.LoopDelta)
}

// disabledTransfer names the transfer checks a test switched off to show each
// one matters ("T1", "T2", "T3" and "ContourDelta", the contour displacement of a
// selected loop's segments); nothing in production sets it.
var disabledTransfer atomic.Pointer[[]string]

func transferOn(check string) bool {
	off := disabledTransfer.Load()
	return off == nil || !slices.Contains(*off, check)
}

// requireTransfer is §8.3's transfer condition on a mixed profile, run right
// after the offset audit of that profile passes. It makes the audit's verdict,
// which speaks about the recorded section, hold for the section the receiver
// denotes:
//
//   - T1 and T2: prismcells.CertifiedContactLoops over the mixed profile's
//     loops with each loop's h_loop, skipping pairs of two exact loops, and
//     adding to a selected loop's segments the displacement of its computed cap
//     contour (the number Build later stores as the loop's band delta);
//   - T3: every displaced loop's area enclosure excludes zero.
func requireTransfer(ctx context.Context, budget *proofbound.WorkBudget, cbp Payload,
	mixed momentinput.Profile, loopDelta []float64) error {
	loops := append([]sectionrecord.LoopRecord{mixed.Outer}, mixed.Holes...)
	if len(loops) != len(loopDelta) {
		return fmt.Errorf(`%w: the displacement list does not match the mixed section's loops`, decaderr.ErrUnsupported)
	}
	recorded := cbp.Loops()
	extra := make([]float64, len(loops))
	for li := range loops {
		if !cbp.StartLoops[li] && !cbp.EndLoops[li] {
			continue
		}
		if !transferOn("ContourDelta") {
			continue
		}
		setback := cbp.LoopSetback(li)
		delta, err := cbp.LoopContourDelta(ctx, li, recorded[li], setback.Dc, setback.DcDelta)
		if err != nil {
			return err
		}
		extra[li] = delta
	}
	if err := requireSameWalks(budget, cbp); err != nil {
		return err
	}
	if transferOn("T1") {
		floor, err := prismcells.ContactFloorOf(budget, loops)
		if err != nil {
			return err
		}
		if !(proofbound.ProductUpper(4, maxOf(loopDelta)) < floor) {
			return fmt.Errorf(`%w: four times the largest loop displacement %g is not below the contact floor %g (agent-docs/prism-boolean-design.md Part K §8.3, T1)`,
				decaderr.ErrUnsupported, maxOf(loopDelta), floor)
		}
	}
	if transferOn("T2") {
		apart, why, err := prismcells.CertifiedGapsLoops(budget, loops, loopDelta, extra)
		if err != nil {
			return err
		}
		if !apart {
			return fmt.Errorf(`%w: the chamfered section's displaced loops are not certified clear of the rest of the section (agent-docs/prism-boolean-design.md Part K §8.3, T2): %s`,
				decaderr.ErrUnsupported, why)
		}
	}
	for li, loop := range loops {
		if loopDelta[li] == 0 || !transferOn("T3") {
			continue
		}
		enc, ok, err := prismcells.LoopAreaEnclosure(budget, loop, loopDelta[li])
		if err != nil {
			return err
		}
		if !ok || enc.Lo.Sign() <= 0 && enc.Hi.Sign() >= 0 {
			return fmt.Errorf(`%w: the area of displaced loop %d is not enclosed away from zero (agent-docs/prism-boolean-design.md Part K §8.3, T3)`,
				decaderr.ErrUnsupported, li)
		}
	}
	return nil
}

// requireSameWalks is T2's walk-list check: the contour displacement read for a
// selected loop belongs to the walks capband.LoopWalks builds, while the
// mixed profile was offset from the walks offset2d.ProfileCornerLoopsBudget
// builds. The two must name the same segments walk for walk, else the
// displacement describes another decomposition.
func requireSameWalks(budget *proofbound.WorkBudget, cbp Payload) error {
	cornerLoops, err := offset2d.ProfileCornerLoopsBudget(budget, cbp.Profile)
	if err != nil {
		return err
	}
	for li, loop := range cbp.Loops() {
		if !cbp.StartLoops[li] && !cbp.EndLoops[li] {
			continue
		}
		walks, err := capband.LoopWalks(budget, loop, freeform.NewFreeformWork())
		if err != nil {
			return err
		}
		if !sameWalkSegments(cornerLoops[li].Walks, walks) {
			return fmt.Errorf(`%w: loop %d's corner walks and its cap-band walks name different segments (agent-docs/prism-boolean-design.md Part K §8.3, T2)`,
				decaderr.ErrUnsupported, li)
		}
	}
	return nil
}

func sameWalkSegments(a, b []survey2d.SideWalk) bool {
	return slices.EqualFunc(a, b, func(x, y survey2d.SideWalk) bool { return slices.Equal(x.Segs, y.Segs) })
}
