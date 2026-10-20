package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/clearance/pairkernel"
	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/decad/internal/pair/planar"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/r3"
)

// This file is the exact planar arm of Verify's pair partition
// (agent-docs/interference-design.md §3.2, agent-docs/clearance-design.md §7): a pair the
// analytic clearance kernel leaves undecided — because an operand has no
// carrier model there (a mitred sweep, a faceted Boolean result), or because
// its enumeration could not settle the pair — is handed to the exact planar
// relation of agent-docs/multibody-dynamics-design.md §9, over the same admitted
// snapshots Document.ContactPair reads (planarSolidAtPose,
// contact_faceted_pair.go). Every predicate there is an exact sign over
// dyadic coordinates, so a verdict it returns is a certificate, never a
// float estimate, and the one rounding — a positive gap's square root — is
// enclosed outward.
// planarPairVerdict decides an admitted pair (planarPairAdmits) through the
// exact planar relation and §10.4's displacement band. ok is false when the
// pair is not served, or when either body is refused a planar snapshot at
// its own placement: a sheet, a displaced section, or a held mesh the
// tessellator refuses.
//
// With δ the summed displacement of the two held boundaries, every true
// boundary point lies within δ of the held pair's, and a point off the held
// boundary by more than its body's displacement keeps its side of it
// (agent-docs/payload-verification-design.md §2.3's 1-Lipschitz rule). So:
//
//   - a held gap g, enclosed in [gLo, gHi], proves the true solids disjoint
//     only where gLo − δ > 0, and the true gap then lies in
//     [gLo − δ, gHi + δ], rounded outward; a held gap within δ proves
//     nothing and stays undecided;
//   - a held touch is reportvocab.PairTouching only at δ = 0, where the held boundary IS
//     the true one and the kernel's local-separation certificate is an
//     exact material-side claim; a touch within a band stays undecided;
//   - a held overlap is reportvocab.PairOverlapping at δ = 0, and otherwise only through
//     a vertex proven deeper than δ inside the other solid
//     (planar.PlanarDeepVertex), which survives any displacement within δ.
//
// The pair diameter the §7 gate reads is pairkernel.PairGateDiameter's, the
// same reading the interference path forms for this pair. The verdict never
// loosens the analytic kernel's: the caller consults this arm only for a
// reportvocab.PairUndecided result.
func planarPairVerdict(ctx context.Context, a, b *Body) (pairkernel.Result, bool, error) {
	if !pairkernel.PlanarPairAdmits(bodyNode(a), bodyNode(b)) {
		return pairkernel.Result{}, false, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return pairkernel.Result{}, false, err
	}
	// A zero chord admits no curved body (planarHeldMeshSolid): those keep
	// the analytic kernel's answer, and this arm reads only an all-planar
	// held boundary, which that chord reads exactly.
	sa, deltaA, okA, err := planarSolidAtPose(ctx, budget, a, r3.Identity(), 0)
	if err != nil || !okA {
		return pairkernel.Result{}, false, err
	}
	sb, deltaB, okB, err := planarSolidAtPose(ctx, budget, b, r3.Identity(), 0)
	if err != nil || !okB {
		return pairkernel.Result{}, false, err
	}
	result, err := planar.ClassifyPlanar(&sa, &sb, budget.Step)
	if err != nil {
		return pairkernel.Result{}, false, err
	}
	diam, err := pairkernel.PairGateDiameter(ctx, &a.n, &b.n, nil)
	if err != nil {
		return pairkernel.Result{}, false, err
	}
	delta := proofarith.DyAdd(deltaA, deltaB)
	undecided := pairkernel.Result{Diam: diam}
	switch result.Relation {
	case pair.Separated:
		res, ok := pairkernel.DisjointResult(result.Gap, delta.Rat(), diam)
		if !ok {
			return undecided, true, nil
		}
		return res, true, nil
	case pair.Touching:
		if delta.Sign() != 0 {
			return undecided, true, nil
		}
		return pairkernel.Result{Verdict: reportvocab.PairTouching, Exact: true, Diam: diam}, true, nil
	case pair.Overlapping:
		if delta.Sign() == 0 {
			return pairkernel.Result{Verdict: reportvocab.PairOverlapping, Diam: diam}, true, nil
		}
		deep, err := planar.PlanarDeepVertex(&sa, &sb, delta, budget.Step)
		if err != nil {
			return pairkernel.Result{}, false, err
		}
		if !deep {
			return undecided, true, nil
		}
		return pairkernel.Result{Verdict: reportvocab.PairOverlapping, Diam: diam}, true, nil
	default:
		return undecided, true, nil
	}
}
