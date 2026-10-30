package meshbool

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/gaptol"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// RefuseUndecidableProximity is the tangency gate (agent-docs/evaluator-design.md
// §9, agent-docs/tessellation-design.md §11 step 4). ma and mb are the two operand
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
// analytic clearance kernel's job (agent-docs/clearance-design.md), not a chord's;
// loud beats silently wrong.
//
// The question is asked per analytic FACE pair, not per facet pair, and the
// depth is proven over the whole face's facets: a facet of one face can pass
// arbitrarily close to a facet of the other while the FACES plainly cross (a
// cylinder wall threading a cap's own triangulation diagonal), and that is no
// tangency at all.
//
// One source certificate narrows the gate, and it still refuses any held
// facet contact it does not cover. A non-nil capFace names a LoftFaces cap of
// the first operand whose true surface the cone-trim source map proves
// disjoint from coneFace of the second; that pair is admitted only when no
// held facet of one touches a held facet of the other.
//
// It may refuse a valid model whose operands genuinely pass within a chord
// tolerance of each other. That is the accepted price; the alternative is a
// verdict decided by chord placement.
func RefuseUndecidableProximity(ctx context.Context, ma, mb *topo.Mesh, bmA, bmB *BoolMesh,
	memo *ContactMemo, capFace, coneFace *topo.Face) error {
	// One counter spans the grouping and the pair scan it feeds: the grouping
	// walks every facet and every new source face's edges, which is work the
	// §7.2 interval covers just as the pair scan is.
	refusal := &TouchContactError{
		What:   touchNear,
		Detail: `the operands' held facets come within the chord tolerance without provably interpenetrating deeper than it, so whether their true surfaces touch or cross is decided by where the chords fall — this evaluator refuses the question rather than answer it wrong`,
	}
	budget := proofbound.NewWorkBudget(ctx)
	fa, err := facesOfMesh(budget, ma, bmA.Boxes)
	if err != nil {
		return err
	}
	fb, err := facesOfMesh(budget, mb, bmB.Boxes)
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
			// Each face box contains all its facet boxes. BoxesWithin uses only
			// monotone subtraction and comparison, so a separated face pair has
			// no facet pair GatherNearContacts could queue at this same slack.
			// Reject it before allocating that scan's maps, buffers and callback.
			if !BoxesWithin(ga.box, gb.box, slack*(1+1e-9)) {
				continue
			}
			broadSlack := slack * (1 + 1e-9)
			broad, deferred, err := GatherNearContacts(ctx, bmA, ga.facets, bmB, gb.facets, broadSlack, memo)
			if err != nil {
				return err
			}
			if deferred || len(broad.CloseA) == 0 {
				continue
			}
			if err := refuseCoarseNearContacts(bmA, bmB, broad); err != nil {
				return err
			}
			reuseDepth := depthWitnessFits(broad)
			if !reuseDepth {
				deep, err := ProvenDepthExceeds(ctx, bmA, broad.CloseA, bmB, broad.CloseB, broad.Spans, broadSlack)
				if err != nil {
					return err
				}
				if deep {
					continue
				}
			}
			// The face-level slack covers the worst facet of either face, which a
			// face that a boolean already shifted or cut holds far from where it
			// meets here. A true touch lies within a facet pair's own bounds of
			// facets that come within the face-level slack. The broad set already
			// contains the facets needed to compute it.
			local := localSlackFromContacts(bmA, bmB, broad, broadSlack)
			narrowed := local < slack
			tightSlack := broadSlack
			contacts := broad
			if narrowed {
				tightSlack = local * (1 + 1e-9)
				contacts, deferred, err = GatherNearContacts(ctx, bmA, ga.facets, bmB, gb.facets, tightSlack, memo)
				if err != nil {
					return err
				}
				if deferred || len(contacts.CloseA) == 0 {
					if err := ctx.Err(); err != nil {
						return err
					}
					continue
				}
			}
			// A broad set under every depth-witness cap includes all tight
			// contacts and can safely supply a witness at the tighter threshold.
			// Larger sets keep the original two-stage depth search.
			if reuseDepth {
				deep, err := ProvenDepthExceeds(ctx, bmA, broad.CloseA, bmB, broad.CloseB, broad.Spans, tightSlack)
				if err != nil {
					return err
				}
				if deep {
					continue
				}
			} else if narrowed {
				if err := refuseCoarseNearContacts(bmA, bmB, contacts); err != nil {
					return err
				}
				deep, err := ProvenDepthExceeds(ctx, bmA, contacts.CloseA, bmB, contacts.CloseB, contacts.Spans, tightSlack)
				if err != nil {
					return err
				}
				if deep {
					continue
				}
			}
			if err := ctx.Err(); err != nil {
				return err
			}
			_, flat := ga.source.Surface.(surfacegeom.Plane)
			contactsOut := nearContactNormalsFromContacts(bmA, contacts, tightSlack, !flat, firstFacet(gb.facets))
			refusal.Contacts = append(refusal.Contacts, contactsOut...)
			if gaptol.Reach(ctx) <= 0 {
				return refusal
			}
			// A caller that may move the second operand moves each connected piece
			// of it on its own, so it needs the contacts of every piece.
		}
	}
	if len(refusal.Contacts) > 0 {
		return refusal
	}
	return nil
}

func refuseCoarseNearContacts(bmA, bmB *BoolMesh, nc NearContacts) error {
	if err := RefuseCoarseHeldContact(bmA, 0, nc.CloseA); err != nil {
		return err
	}
	return RefuseCoarseHeldContact(bmB, 1, nc.CloseB)
}

func depthWitnessFits(nc NearContacts) bool {
	return len(nc.CloseA) <= MaxDepthWitnessFacets &&
		len(nc.CloseB) <= MaxDepthWitnessFacets &&
		len(nc.Spans) <= MaxDepthWitnessSpans
}

// localSlackFromContacts returns the sum of the largest facet bound among the
// first face's near facets and the largest among the second's, or slack when
// either side has no near facets.
func localSlackFromContacts(bmA, bmB *BoolMesh, nc NearContacts, slack float64) float64 {
	if len(nc.CloseA) == 0 || len(nc.CloseB) == 0 {
		return slack
	}
	worst := func(m *BoolMesh, facets []int) float64 {
		w := 0.0
		for _, f := range facets {
			w = max(w, m.FacetBound[f])
		}
		return w
	}
	return proofbound.AbsSumUpper(worst(bmA, nc.CloseA), worst(bmB, nc.CloseB))
}

// firstFacet is the first of facets, or zero when there is none.
func firstFacet(facets []int) int {
	if len(facets) == 0 {
		return 0
	}
	return facets[0]
}

// nearContactNormalsFromContacts reads the first operand's outward unit normal
// at each near facet, once per distinct direction and at most maxTouchContacts.
// A caller allowed to move an operand uses them to choose the direction and
// distance; the second operand has to reach slack past the first. A curved face
// makes the contact decisive because moving away detaches the body. A flat face
// is not decisive because the gate cannot distinguish a body resting on it from
// one flush beneath it. The operation chooses the direction: Union moves into
// the first operand; Cut and Intersect move out.
func nearContactNormalsFromContacts(bmA *BoolMesh, nc NearContacts, slack float64, decisive bool, otherFacet int) []TouchContact {
	type key [3]int64
	seen := map[key]struct{}{}
	var out []TouchContact
	for _, i := range nc.CloseA {
		c := touchOf(bmA, i, otherFacet)
		c.Slack, c.Decisive = slack, decisive
		k := key{int64(math.Round(c.NormalA.X * 1e3)), int64(math.Round(c.NormalA.Y * 1e3)), int64(math.Round(c.NormalA.Z * 1e3))}
		if _, dup := seen[k]; dup {
			continue
		}
		seen[k] = struct{}{}
		out = append(out, c)
		if len(out) >= maxTouchContacts {
			break
		}
	}
	return out
}

// faceFacets is one analytic face of an operand: its facets, and the chord
// displacement between them and the true face.
type faceFacets struct {
	source *topo.Face
	delta  float64
	facets []int
	box    [2]r3.Vec
}

// facesOfMesh groups a tessellation's facets by their source face, in first-
// appearance order, and charges each face the displacement the MESH proved for
// it: agent-docs/tessellation-design.md §2's sourceBound(face) (topo.Mesh.FaceBound),
// which the tessellator composed from that face's own trim chording,
// coordinate construction, section and axial terms. The gate never re-infers
// a displacement from the surface kind — a planar carrier does not make a
// trimmed patch exact, and a zero here is the claim that the held polygon IS
// the true trimmed patch with stored coordinates that add nothing.
//
// A source face the mesh states no bound for is a broken evaluator, not a
// staged capability: it returns ErrBooleanFailed rather than an ErrUnsupported
// that Verify would hide as an undecided pair, and never a zero.
func facesOfMesh(budget *proofbound.WorkBudget, m *topo.Mesh, boxes [][2]r3.Vec) ([]faceFacets, error) {
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
			out = append(out, faceFacets{source: f, delta: delta, box: boxes[i]})
		}
		out[k].facets = append(out[k].facets, i)
		box := &out[k].box
		box[0].X = math.Min(box[0].X, boxes[i][0].X)
		box[0].Y = math.Min(box[0].Y, boxes[i][0].Y)
		box[0].Z = math.Min(box[0].Z, boxes[i][0].Z)
		box[1].X = math.Max(box[1].X, boxes[i][1].X)
		box[1].Y = math.Max(box[1].Y, boxes[i][1].Y)
		box[1].Z = math.Max(box[1].Z, boxes[i][1].Z)
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
