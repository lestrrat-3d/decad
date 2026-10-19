package meshbool

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// NearPairRole is a source certificate's verdict for one held face pair the
// tangency gate could not otherwise decide. The first curved spiral tooth
// join supplies the only table today; its caller must still audit the
// complete held result and refuse any near pair absent from the table.
type NearPairRole uint8

const (
	// NearPairUnknown is a pair the certificate says nothing about.
	NearPairUnknown NearPairRole = iota
	// NearPairCrossing is a pair whose true faces cross; a held near-miss
	// between them passes the gate.
	NearPairCrossing
	// NearPairSeparated is a pair whose true faces are disjoint; any held
	// facet contact between them is refused.
	NearPairSeparated
)

// RefuseUndecidableProximity is the tangency gate (docs/evaluator-design.md
// §9, docs/tessellation-design.md §11 step 4). ma and mb are the two operand
// tessellations bmA and bmB were prepared from, and memo is the call's
// contact memo over bmA and bmB.
//
// The mesh boolean decides every contact on the CHORDS, and a chord polygon
// lies strictly inside the curved surface it approximates — so a true tangency
// between two operands can vanish from the tessellation entirely, and a
// spurious meet can appear where the true surfaces stay apart (chording a
// concave hole wall inward adds held material into the void). Either way the
// verdict would be decided by where the chord samples happened to fall.
//
// The gate is reject-only, and it proves nothing it does not have. If the true
// surfaces of two faces touch, that point lies within δA of face A's facets and
// within δB of face B's, so the two facet sets come within b = δA + δB of each
// other. A face pair whose facets stay FARTHER apart than b hides no touch and
// is decided. Within b the gate decides the pair by the INTERPENETRATION DEPTH
// of the two facet sets: a held meet is admitted only when the facets provably
// cross with a signed depth strictly GREATER than b (ProvenDepthExceeds). Then
// A's true surface (within δA of its facets) and B's (within δB of its) still
// overlap even after each is pulled back toward its own interior by its own
// bound, so the true patches provably cross and the contact is real. The chord
// error alone, bounded by b, cannot open a penetration deeper than b, so a bare
// touch, a shallow meet at depth ≤ b, or a non-meeting near-miss within b proves
// nothing and is refused (ErrUnsupported). Deciding such a pair for real is the
// analytic clearance kernel's job (docs/clearance-design.md), not a chord's;
// loud beats silently wrong.
//
// The question is asked per analytic FACE pair, not per facet pair, and the
// depth is proven over the whole face's facets: a facet of one face can pass
// arbitrarily close to a facet of the other while the FACES plainly cross (a
// cylinder wall threading a cap's own triangulation diagonal), and that is no
// tangency at all.
//
// Two source certificates narrow the gate, and each still refuses any held
// facet contact it does not cover. A non-nil capFace names a LoftFaces cap of
// the first operand whose true surface the cone-trim source map proves
// disjoint from coneFace of the second; that pair is admitted only when no
// held facet of one touches a held facet of the other. roles is the spiral
// first-join table (nil for every other boolean): a separated pair gets the
// same no-contact check, and a crossing pair passes a near-miss.
//
// It may refuse a valid model whose operands genuinely pass within a chord
// tolerance of each other. That is the accepted price; the alternative is a
// verdict decided by chord placement.
func RefuseUndecidableProximity(ctx context.Context, ma, mb *topo.Mesh, bmA, bmB *BoolMesh,
	memo *ContactMemo, capFace, coneFace *topo.Face, roles map[[2]*topo.Face]NearPairRole) error {
	// One counter spans the grouping and the pair scan it feeds: the grouping
	// walks every facet and every new source face's edges, which is work the
	// §7.2 interval covers just as the pair scan is.
	budget := proofbound.NewWorkBudget(ctx)
	fa, err := facesOfMesh(budget, ma)
	if err != nil {
		return err
	}
	fb, err := facesOfMesh(budget, mb)
	if err != nil {
		return err
	}
	for _, ga := range fa {
		for _, gb := range fb {
			if err := budget.Step(); err != nil {
				return err
			}
			if capFace != nil && ga.source == capFace && gb.source == coneFace {
				if err := refuseFacetContact(ctx, bmA, bmB, ga.facets, gb.facets, memo); err != nil {
					return err
				}
				continue
			}
			role := roles[[2]*topo.Face{ga.source, gb.source}]
			if role == NearPairSeparated {
				// A true source separation cannot admit a held contact, even
				// when the held penetration is deeper than the near threshold.
				if err := refuseFacetContact(ctx, bmA, bmB, ga.facets, gb.facets, memo); err != nil {
					return err
				}
				continue
			}
			slack := ga.delta + gb.delta
			if slack <= 0 {
				// Both faces are held exactly (a planar face with straight
				// edges triangulates exactly; a Faceted face IS its polygons,
				// core §6.1). A tangency between them is visible to the exact
				// predicates, so there is nothing here a chord could hide.
				continue
			}
			// The exactly-tangent case sits ON the boundary d = δA + δB, and
			// both sides of that comparison are float-computed: pad the
			// threshold so the rounding can only ever ADD a refusal, never
			// drop one. A relative 1e-9 is nothing against any real clearance.
			near, err := FacesNearMiss(ctx, bmA, ga.facets, bmB, gb.facets, slack*(1+1e-9), memo)
			if err != nil {
				return err
			}
			if !near || role == NearPairCrossing {
				// A curved first-join crossing can pass this ambiguity only
				// when its result will audit the matching complete edge chain.
				continue
			}
			return fmt.Errorf(`%w: the operands' held facets come within the chord tolerance without provably interpenetrating deeper than it, so whether their true surfaces touch or cross is decided by where the chords fall — this evaluator refuses the question rather than answer it wrong`, decaderr.ErrUnsupported)
		}
	}
	return nil
}

// faceFacets is one analytic face of an operand: its facets, and the chord
// displacement between them and the true face.
type faceFacets struct {
	source *topo.Face
	delta  float64
	facets []int
}

// facesOfMesh groups a tessellation's facets by their source face, in first-
// appearance order, and charges each face the displacement the MESH proved for
// it: docs/tessellation-design.md §2's sourceBound(face) (topo.Mesh.FaceBound),
// which the tessellator composed from that face's own trim chording,
// coordinate construction, section and axial terms. The gate never re-infers
// a displacement from the surface kind — a planar carrier does not make a
// trimmed patch exact, and a zero here is the claim that the held polygon IS
// the true trimmed patch with stored coordinates that add nothing.
//
// A source face the mesh states no bound for is a broken evaluator, not a
// staged capability: it returns ErrBooleanFailed rather than an ErrUnsupported
// that Verify would hide as an undecided pair, and never a zero.
func facesOfMesh(budget *proofbound.WorkBudget, m *topo.Mesh) ([]faceFacets, error) {
	index := map[*topo.Face]int{}
	var out []faceFacets
	for i, f := range m.Source {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		k, ok := index[f]
		if !ok {
			delta, stated := m.FaceBound[f]
			if !stated {
				return nil, fmt.Errorf(`%w: an operand's tessellation states no displacement bound for one of its own source faces`, decaderr.ErrBooleanFailed)
			}
			k = len(out)
			index[f] = k
			out = append(out, faceFacets{source: f, delta: delta})
		}
		out[k].facets = append(out[k].facets, i)
	}
	return out, nil
}

// refuseFacetContact refuses any held contact between facets a of bmA and b
// of bmB, for a face pair whose true surfaces a source certificate proves
// disjoint: the held facets must be disjoint too before the pair skips the
// near-miss question. Both certificates share the cone-cap wording of the
// refusal.
func refuseFacetContact(ctx context.Context, bmA, bmB *BoolMesh, a, b []int,
	memo *ContactMemo) error {
	budget := proofbound.NewWorkBudget(ctx)
	for _, i := range a {
		for _, j := range b {
			if err := budget.Step(); err != nil {
				return err
			}
			if !BoxesWithin(bmA.Boxes[i], bmB.Boxes[j], 0) {
				continue
			}
			contact, err := memo.Classify(i, j)
			if err != nil {
				return err
			}
			if contact.Kind != ContactNone {
				return fmt.Errorf("%w: the certified cone cap has a held facet contact", decaderr.ErrUnsupported)
			}
		}
	}
	return nil
}
