package meshbool

import (
	"context"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/gaptol"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

func RefuseUndecidableProximityReplay(ctx context.Context, ma, mb *topo.Mesh, bmA, bmB *BoolMesh,
	memo *ContactMemo, capFace, coneFace *topo.Face, replay *proofbound.ReplayBudget) error {
	if replay == nil {
		return errReplayProximityReplayNeedsASharedBudget
	}
	if capFace != nil && coneFace != nil {
		return errReplayReplayCapContactProofIsUnavailable
	}
	if err := replay.SpendWork(ctx, 8); err != nil {
		return err
	}
	if err := replay.ReserveRecords(ctx, 8); err != nil {
		return err
	}
	if err := replay.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(proofbound.WorkBudget{}))+256); err != nil {
		return err
	}
	if err := replay.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(TouchContactError{}))); err != nil {
		return err
	}
	return refuseUndecidableProximityCachedReplay(ctx, ma, mb, bmA, bmB, memo, capFace, coneFace, replay.WorkBudget(ctx), replay)
}

// reserveBrakeNearContactsFixed covers the runner closures and captured result
// cell before they are created at the gather call.
func reserveBrakeNearContactsFixed(ctx context.Context, replay *proofbound.ReplayBudget) error {
	if err := replay.SpendWork(ctx, 4); err != nil {
		return err
	}
	if err := replay.ReserveRecords(ctx, 4); err != nil {
		return err
	}
	return replay.ReserveArray(ctx, 1, 512)
}

// refuseUndecidableProximityShared uses one caller-owned work budget for the
// private multiway path; nil retains the original binary behavior.
func refuseUndecidableProximityCachedReplay(ctx context.Context, ma, mb *topo.Mesh, bmA, bmB *BoolMesh,
	memo *ContactMemo, capFace, coneFace *topo.Face, shared *proofbound.WorkBudget, replay *proofbound.ReplayBudget) error {
	// One counter spans the grouping and the pair scan it feeds: the grouping
	// walks every facet and every new source face's edges, which is work the
	// §7.2 interval covers just as the pair scan is.
	refusal := &TouchContactError{
		What:   touchNear,
		Detail: `the operands' held facets come within the chord tolerance without provably interpenetrating deeper than it, so whether their true surfaces touch or cross is decided by where the chords fall — this evaluator refuses the question rather than answer it wrong`,
	}
	budget := shared
	gather := func(a, b []int, slack float64) (NearContacts, bool, error) {
		if shared == nil {
			return GatherNearContacts(ctx, bmA, a, bmB, b, slack, memo)
		}
		if replay != nil {
			if err := reserveBrakeNearContactsFixed(ctx, replay); err != nil {
				return NearContacts{}, false, err
			}
			return gatherNearContactsSharedReplay(ctx, bmA, a, bmB, b, slack, memo, replayNearContactRunner(slack, replay), ContactBatchSize, shared, replay)
		}
		return gatherNearContactsShared(ctx, bmA, a, bmB, b, slack, memo,
			NearContactRunner(slack), ContactBatchSize, shared)
	}
	depth := func(nc NearContacts, slack float64) (bool, error) {
		if shared == nil {
			return ProvenDepthExceeds(ctx, bmA, nc.CloseA, bmB, nc.CloseB, nc.Spans, slack)
		}
		return provenDepthExceedsShared(ctx, bmA, nc.CloseA, bmB, nc.CloseB, nc.Spans, slack, shared)
	}
	chargeContacts := func(nc NearContacts) error {
		if shared == nil {
			return nil
		}
		_, err := shared.StepN(uint64(len(nc.CloseA)) + uint64(len(nc.CloseB)))
		return err
	}
	fa, err := brakefacesOfMesh(ctx, budget, ma, bmA.Boxes)
	if err != nil {
		return err
	}
	fb, err := brakefacesOfMesh(ctx, budget, mb, bmB.Boxes)
	if err != nil {
		return err
	}
	for _, ga := range fa {
		for _, gb := range fb {
			if err := budget.Step(); err != nil {
				return err
			}
			if capFace != nil && ga.source == capFace && gb.source == coneFace {
				if err := refuseFacetContactShared(ctx, bmA, bmB, ga.facets, gb.facets, memo, shared); err != nil {
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
			broad, deferred, err := gather(ga.facets, gb.facets, broadSlack)
			if err != nil {
				return err
			}
			if deferred || len(broad.CloseA) == 0 {
				continue
			}
			if err := chargeContacts(broad); err != nil {
				return err
			}
			if err := brakerefuseCoarseNearContacts(bmA, bmB, broad); err != nil {
				return err
			}
			reuseDepth := brakedepthWitnessFits(broad)
			if !reuseDepth {
				deep, err := depth(broad, broadSlack)
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
			if err := chargeContacts(broad); err != nil {
				return err
			}
			local := brakelocalSlackFromContacts(bmA, bmB, broad, broadSlack)
			narrowed := local < slack
			tightSlack := broadSlack
			contacts := broad
			if narrowed {
				tightSlack = local * (1 + 1e-9)
				contacts, deferred, err = gather(ga.facets, gb.facets, tightSlack)
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
				deep, err := depth(broad, tightSlack)
				if err != nil {
					return err
				}
				if deep {
					continue
				}
			} else if narrowed {
				if err := chargeContacts(contacts); err != nil {
					return err
				}
				if err := brakerefuseCoarseNearContacts(bmA, bmB, contacts); err != nil {
					return err
				}
				deep, err := depth(contacts, tightSlack)
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
			if err := chargeContacts(contacts); err != nil {
				return err
			}
			contactsOut, err := brakenearContactNormalsFromContacts(bmA, contacts, tightSlack, !flat, brakefirstFacet(gb.facets), budget)
			if err != nil {
				return err
			}
			refusal.Contacts, err = appendWork(refusal.Contacts, contactsOut, budget)
			if err != nil {
				return err
			}
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

func brakerefuseCoarseNearContacts(bmA, bmB *BoolMesh, nc NearContacts) error {
	if err := RefuseCoarseHeldContact(bmA, 0, nc.CloseA); err != nil {
		return err
	}
	return RefuseCoarseHeldContact(bmB, 1, nc.CloseB)
}

func brakedepthWitnessFits(nc NearContacts) bool {
	return len(nc.CloseA) <= MaxDepthWitnessFacets &&
		len(nc.CloseB) <= MaxDepthWitnessFacets &&
		len(nc.Spans) <= MaxDepthWitnessSpans
}

// brakelocalSlackFromContacts returns the sum of the largest facet bound among the
// first face's near facets and the largest among the second's, or slack when
// either side has no near facets.
func brakelocalSlackFromContacts(bmA, bmB *BoolMesh, nc NearContacts, slack float64) float64 {
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

// brakefirstFacet is the first of facets, or zero when there is none.
func brakefirstFacet(facets []int) int {
	if len(facets) == 0 {
		return 0
	}
	return facets[0]
}

// brakenearContactNormalsFromContacts reads the first operand's outward unit normal
// at each near facet, once per distinct direction and at most maxTouchContacts.
// A caller allowed to move an operand uses them to choose the direction and
// distance; the second operand has to reach slack past the first. A curved face
// makes the contact decisive because moving away detaches the body. A flat face
// is not decisive because the gate cannot distinguish a body resting on it from
// one flush beneath it. The operation chooses the direction: Union moves into
// the first operand; Cut and Intersect move out.
func brakenearContactNormalsFromContacts(bmA *BoolMesh, nc NearContacts, slack float64, decisive bool, otherFacet int, budget *proofbound.WorkBudget) ([]TouchContact, error) {
	type key [3]int64
	var seen [maxTouchContacts]key
	var n int
	if err := budget.Reserve(maxTouchContacts); err != nil {
		return nil, err
	}
	if err := budget.ReserveBytesMul(maxTouchContacts, uint64(unsafe.Sizeof(TouchContact{}))); err != nil {
		return nil, err
	}
	if _, err := budget.StepN(maxTouchContacts); err != nil {
		return nil, err
	}
	out := make([]TouchContact, 0, maxTouchContacts)
	for _, i := range nc.CloseA {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		c := touchOf(bmA, i, otherFacet)
		c.Slack, c.Decisive = slack, decisive
		k := key{int64(math.Round(c.NormalA.X * 1e3)), int64(math.Round(c.NormalA.Y * 1e3)), int64(math.Round(c.NormalA.Z * 1e3))}
		duplicate := false
		for j := range n {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			if seen[j] == k {
				duplicate = true
				break
			}
		}
		if duplicate {
			continue
		}
		seen[n] = k
		n++
		out = append(out, c)
		if n == maxTouchContacts {
			break
		}
	}
	return out, nil
}

// faceFacets is one analytic face of an operand: its facets, and the chord
// displacement between them and the true face.

// brakefacesOfMesh groups a tessellation's facets by their source face, in first-
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
type faceGroupingBudget struct{ work *proofbound.WorkBudget }

func (b faceGroupingBudget) ReserveRecords(_ context.Context, n uint64) error {
	return b.work.Reserve(n)
}
func (b faceGroupingBudget) ReserveArray(_ context.Context, n, size uint64) error {
	return b.work.ReserveBytesMul(n, size)
}
func (b faceGroupingBudget) SpendWork(_ context.Context, n uint64) error {
	_, err := b.work.StepN(n)
	return err
}

func brakefacesOfMesh(ctx context.Context, budget *proofbound.WorkBudget, m *topo.Mesh, boxes [][2]r3.Vec) ([]faceFacets, error) {
	var index map[*topo.Face]int
	var fixed *topo.FaceBoundTable
	fb := faceGroupingBudget{work: budget}
	var out []faceFacets
	if budget.Replay {
		if uint64(len(m.Source)) > 1<<53 {
			return nil, errReplayTooManyFaceGroups
		}
		var err error
		fixed, err = topo.NewFaceBoundTable(ctx, len(m.Source), fb)
		if err != nil {
			return nil, err
		}
		if err := budget.Reserve(uint64(len(m.Source))); err != nil {
			return nil, err
		}
		if err := budget.ReserveBytesMul(uint64(len(m.Source)), uint64(unsafe.Sizeof(faceFacets{}))); err != nil {
			return nil, err
		}
		if _, err := budget.StepN(uint64(len(m.Source))); err != nil {
			return nil, err
		}
		out = make([]faceFacets, 0, len(m.Source))
	} else {
		index = map[*topo.Face]int{}
	}
	for i, f := range m.Source {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		k, ok := index[f]
		if fixed != nil {
			value, found, err := fixed.Lookup(ctx, f, fb)
			if err != nil {
				return nil, err
			}
			ok = found
			k = int(value)
		}
		if !ok {
			delta, stated := m.FaceBound[f]
			if m.ReplayFaceBounds != nil {
				var err error
				delta, stated, err = m.ReplayFaceBounds.LookupWork(f, budget.Step)
				if err != nil {
					return nil, err
				}
			}
			if !stated {
				return nil, errReplayAnOperandSTessellationStatesNoDisplacement
			}
			k = len(out)
			if fixed != nil {
				if err := fixed.Set(ctx, f, float64(k), fb); err != nil {
					return nil, err
				}
			} else {
				index[f] = k
			}
			out = append(out, faceFacets{source: f, delta: delta, box: boxes[i]})
		}
		if fixed != nil && len(out[k].facets) == cap(out[k].facets) {
			n := max(1, 2*cap(out[k].facets))
			if err := budget.Reserve(uint64(n)); err != nil {
				return nil, err
			}
			if err := budget.ReserveBytesMul(uint64(n), uint64(unsafe.Sizeof(int(0)))); err != nil {
				return nil, err
			}
			if _, err := budget.StepN(uint64(len(out[k].facets))); err != nil {
				return nil, err
			}
			next := make([]int, len(out[k].facets), n)
			copy(next, out[k].facets)
			out[k].facets = next
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
func refuseFacetContactShared(ctx context.Context, bmA, bmB *BoolMesh, a, b []int,
	memo *ContactMemo, shared *proofbound.WorkBudget) error {
	budget := proofbound.NewWorkBudget(ctx)
	if shared != nil {
		budget = shared
	}
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
				return errReplayTheCertifiedConeCapHasAHeld
			}
		}
	}
	return nil
}
