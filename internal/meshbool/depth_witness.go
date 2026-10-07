package meshbool

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
)

// FacesNearMiss reports whether two faces' facet sets come within slack of each
// other in a way this evaluator cannot decide (docs/evaluator-design.md §9). When
// the facets stay farther than slack apart everywhere, no touch can be hiding and
// the pair is decided (near = false). When they come within slack — meeting or
// not — the pair is decided only if the facet sets provably INTERPENETRATE deeper
// than b = slack (ProvenDepthExceeds); a bare touch, a shallow meet at depth ≤ b,
// or a non-meeting near-miss stays undecidable (near = true). A coplanar
// face-on-face overlap is a tangency the exact mesh pass refuses as an
// unclassifiable contact (BooleanUnsupportedContact / ErrUnsupported), so it is
// left to that verdict rather than pre-empted here. That deferral is sound only
// while that refusal happens: docs/interference-design.md §5.2 states what must
// settle such a pair before the refusal is removed.
func FacesNearMiss(ctx context.Context, bmA *BoolMesh, fis []int, bmB *BoolMesh, fjs []int, slack float64, memo *ContactMemo) (bool, error) {
	nc, deferred, err := GatherNearContacts(ctx, bmA, fis, bmB, fjs, slack, memo)
	if err != nil || deferred {
		return false, err
	}
	if len(nc.CloseA) == 0 {
		return false, nil // the facets stay clear of each other: nothing hides
	}
	// A restating operand's facets within the slack are where the pair meets
	// it, so the local chain-depth gate reads them before any depth question
	// is asked (docs/faceted-vertex-bounds-design.md §5).
	if err := RefuseCoarseHeldContact(bmA, 0, nc.CloseA); err != nil {
		return false, err
	}
	if err := RefuseCoarseHeldContact(bmB, 1, nc.CloseB); err != nil {
		return false, err
	}
	deep, err := ProvenDepthExceeds(ctx, bmA, nc.CloseA, bmB, nc.CloseB, nc.Spans, slack)
	if err != nil {
		return false, err
	}
	return !deep, nil
}

// ProvenDepthExceeds is the reject-only depth discriminator behind the meet
// admission (docs/evaluator-design.md §9, docs/tessellation-design.md §11 step
// 4). It reports true ONLY when it has PROVEN that the two facet sets
// interpenetrate by a signed depth strictly greater than b: a concrete point of
// one operand's held facets, certified STRICTLY inside the other operand's solid,
// whose certified lower-bound distance to that solid's boundary exceeds b. Then
// even after each true surface is pulled back toward its own interior by its own
// bound the two still cross, so the true patches provably interpenetrate and the
// held meet is real. When it cannot prove such a witness it returns false and the
// meet is treated as undecidable and refused upstream — over-refuse, never
// over-admit.
//
// It looks for a witness in two places: the fixed sample points of every
// contacting facet (DeepWitnessInside), then points on each contact segment's
// two facets walked inward from that segment (SpanWitness). Both only nominate
// candidates; DeepWitnessAt alone certifies one.
func ProvenDepthExceeds(ctx context.Context, bmA *BoolMesh, closeA []int, bmB *BoolMesh, closeB []int, spans []ContactSpan, b float64) (bool, error) {
	deep, err := DeepWitnessInside(ctx, bmA, closeA, bmB, b)
	if err != nil || deep {
		return deep, err
	}
	deep, err = DeepWitnessInside(ctx, bmB, closeB, bmA, b)
	if err != nil || deep {
		return deep, err
	}
	return SpanWitness(ctx, bmA, bmB, spans, b)
}

// DeepWitnessInside reports whether any held-facet sample point of m's
// contacting facets is PROVEN to lie strictly inside the other operand's solid
// deeper than b. The candidates are each facet's vertices, edge midpoints and
// centroid: a facet's deepest penetration need not fall on a mesh vertex (a rod
// pierces a plate through the interior of its wall facets), so the midpoints and
// centroid are sampled too. Reject-only: sampling and the facet cap can only
// miss a witness and refuse, never admit a shallow meet.
func DeepWitnessInside(ctx context.Context, m *BoolMesh, closeFacets []int, other *BoolMesh, b float64) (bool, error) {
	all := AllFacets(other)
	for n, i := range closeFacets {
		if n >= MaxDepthWitnessFacets {
			break
		}
		if err := ctx.Err(); err != nil {
			return false, err
		}
		tri := m.Tris[i]
		for _, p := range facetSamplePoints(m.Xverts[tri[0]], m.Xverts[tri[1]], m.Xverts[tri[2]]) {
			deep, err := DeepWitnessAt(ctx, p, other, all, b)
			if err != nil || deep {
				return deep, err
			}
		}
	}
	return false, nil
}

// facetSamplePoints returns the exact candidate interior witnesses of one held
// facet: its three corners, its three edge midpoints and its centroid. Every one
// is an exact rational, so the parity test that reads it decides strict
// containment without rounding.
func facetSamplePoints(a, b, c proof.Xpt) []proof.Xpt {
	one, two := big.NewInt(1), big.NewInt(2)
	mid := func(p, q proof.Xpt) proof.Xpt { return Xlerp(p, q, one, two) }
	return []proof.Xpt{a, b, c, mid(a, b), mid(b, c), mid(c, a), XCentroid(a, b, c)}
}
