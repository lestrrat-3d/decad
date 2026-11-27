package meshbool

import (
	"context"
	"fmt"
	"math"
	"slices"

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
// A coplanar face-on-face overlap is deferred by the caller's gather, and the
// deferral is backed in the mesh pass (agent-docs/coplanar-contact-design.md
// §6). A face pair with a summed reading of zero is skipped above: both faces
// are exact, so a coplanar facet pair between them is classified exactly when
// the mesh pass admits it (zero facet bounds, the coplanar switch on, and an
// attempt that is not replayed, shifted or a fixed tool), and refused with
// TouchCoplanar otherwise. A face pair with a positive sum holds an inexact
// face, and every coplanar facet pair in it holds a facet of that face, which
// the mesh pass refuses with TouchCoplanar unless the two faces are a flush
// pair (agent-docs/flush-contact-design.md §4.7): planar carrier faces in one
// exact plane whose held outlines keep their arrangement under their bounds,
// so the pass classifies their contact on held facets that carry the true
// arrangement. Every deferral is thus backed by exact classification, a flush
// admission, or a refusal. The one exception is the occupancy
// re-run of composeOccupancy, which widens the face bounds after the mesh pass;
// it drops its record whenever the pass admitted a coplanar point.
//
// It may refuse a valid model whose operands genuinely pass within a chord
// tolerance of each other. That is the accepted price; the alternative is a
// verdict decided by chord placement.
func RefuseUndecidableProximity(ctx context.Context, ma, mb *topo.Mesh, bmA, bmB *BoolMesh,
	memo *ContactMemo, capFace, coneFace *topo.Face) error {
	return RefuseProximityWithPlaneEvidence(ctx, ma, mb, bmA, bmB, memo, capFace, coneFace, nil)
}

// RefuseProximityWithPlaneEvidence permits exact plane separation only for source-aligned operands.
func RefuseProximityWithPlaneEvidence(ctx context.Context, ma, mb *topo.Mesh, bmA, bmB *BoolMesh,
	memo *ContactMemo, capFace, coneFace *topo.Face, planeBudget *proofbound.WorkBudget) error {
	return RefuseProximityWithFaceEvidence(ctx, ma, mb, bmA, bmB, memo, capFace, coneFace, planeBudget, nil)
}

// RefuseProximityWithFaceEvidence defers a face pair only when evidence proves
// the held and true contacts agree. The mesh pass must still classify it.
func RefuseProximityWithFaceEvidence(ctx context.Context, ma, mb *topo.Mesh, bmA, bmB *BoolMesh,
	memo *ContactMemo, capFace, coneFace *topo.Face, planeBudget *proofbound.WorkBudget,
	evidence func(*topo.Face, *topo.Face) (bool, error)) error {
	// One counter spans the grouping and the pair scan it feeds: the grouping
	// walks every facet and every new source face's edges, which is work the
	// §7.2 interval covers just as the pair scan is.
	refusal := &TouchContactError{
		What:   touchNear,
		Detail: `the operands' held facets come within the chord tolerance without provably interpenetrating deeper than it, so whether their true surfaces touch or cross is decided by where the chords fall — this evaluator refuses the question rather than answer it wrong`,
	}
	budget := proofbound.NewWorkBudget(ctx)
	fa, err := facesOfMesh(budget, ma, bmA.Boxes, bmA.FaceSurface)
	if err != nil {
		return err
	}
	fb, err := facesOfMesh(budget, mb, bmB.Boxes, bmB.FaceSurface)
	if err != nil {
		return err
	}
	var separation *proofbound.ReplayBudget
	examine := func(ga, gb faceFacets, slack float64, certified bool) error {
		if certified {
			return refuseFacetContact(ctx, bmA, bmB, ga.facets, gb.facets, memo)
		}
		if planeBudget != nil && carrierClaimPossible(ga.source) && carrierClaimPossible(gb.source) {
			separated, err := separatedCarrierFaces(ga, gb, bmA, bmB, planeBudget)
			if err != nil || separated {
				return err
			}
		}
		// A seat face against a wall that leaves its plane meets it only along the wall's rims (§16).
		if seatSkip(bmA, ga.facets, bmB, gb.facets) || seatSkip(bmB, gb.facets, bmA, ga.facets) {
			return nil
		}
		broadSlack := slack * (1 + 1e-9)
		var log []gatheredPair
		broad, deferred, err := gatherNearContactsLog(ctx, bmA, ga.facets, bmB, gb.facets, broadSlack, memo,
			NearContactRunner(broadSlack), ContactBatchSize, &log)
		if err != nil {
			return err
		}
		if deferred || len(broad.CloseA) == 0 {
			return nil
		}
		if (bmA.Gate > 0 && coarsestSlab(bmA, broad.CloseA) > bmA.Gate) || (bmB.Gate > 0 && coarsestSlab(bmB, broad.CloseB) > bmB.Gate) {
			if separation == nil {
				separation = proofbound.NewStrictReplayBudget()
			}
			apart, err := separatedCoarseFaces(ctx, bmA, bmB, ga.facets, gb.facets, memo, separation)
			if err != nil || apart {
				return err
			}
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
				return nil
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
			contacts, deferred, err = narrowNearContacts(ctx, bmA, ga.facets, bmB, gb.facets, tightSlack, memo, broad, log)
			if err != nil {
				return err
			}
			if deferred || len(contacts.CloseA) == 0 {
				if err := ctx.Err(); err != nil {
					return err
				}
				return nil
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
				return nil
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
				return nil
			}
		}
		if evidence != nil {
			matched, err := evidence(ga.source, gb.source)
			if err != nil || matched {
				return err
			}
		}
		if err := ctx.Err(); err != nil {
			return err
		}
		_, flat := ga.source.Surface.(surfacegeom.Plane)
		contactsOut := nearContactNormalsFromContacts(bmA, contacts, tightSlack, !flat, firstFacet(gb.facets),
			flatAcross(ga, bmB, gb))
		refusal.Contacts = append(refusal.Contacts, contactsOut...)
		if gaptol.Reach(ctx) <= 0 {
			return refusal
		}
		// A caller that may move the second operand moves each connected piece
		// of it on its own, so it needs the contacts of every piece.
		return nil
	}
	if err := gateFacePairs(budget, fa, fb, capFace, coneFace, examine); err != nil {
		return err
	}
	if len(refusal.Contacts) > 0 {
		return refusal
	}
	return nil
}

// gateFacePairs runs the gate's cheap rejections (gateFacePair) over every face pair of fa × fb, row by row in the
// order of fa and then of fb, and calls examine on each pair that passes them. It steps budget once per face pair,
// before that pair's rejections, so the context polls fall at the counts a scan of every pair gives them.
//
// A row of more than heldBoxLeafSize faces visits only the faces of fb that a box tree over their boxes keeps, in
// ascending index order, and steps budget past the faces it skips with StepN. The tree keeps every face whose box
// comes within reach of ga's, where reach pads ga.delta plus the largest delta in fb as gateFacePair pads a pair's
// slack. Rounded addition and multiplication by a positive constant are monotone, so reach is at least every pair's
// padded slack, and BoxesWithin only gains pairs as its slack grows. A face the tree skips therefore fails the box
// test, which does nothing but step, so the row calls examine on the same pairs in the same order between the same
// polls. The certified cap row runs its cone pair before the box test, and a non-finite delta has no finite reach
// to pad by, so those rows scan every pair.
func gateFacePairs(budget *proofbound.WorkBudget, fa, fb []faceFacets, capFace, coneFace *topo.Face,
	examine func(ga, gb faceFacets, slack float64, certified bool) error) error {
	indexed := len(fb) > heldBoxLeafSize
	maxDelta := math.Inf(-1)
	for _, gb := range fb {
		if proofbound.IsNonFinite(gb.delta) {
			indexed = false
			break
		}
		maxDelta = max(maxDelta, gb.delta)
	}
	var tree heldBoxTree
	var candidates, stack []int
	if indexed {
		boxes := make([][2]r3.Vec, len(fb))
		for k, gb := range fb {
			boxes[k] = gb.box
		}
		tree = newQueryBoxTree(boxes)
	}
	for _, ga := range fa {
		reach := (ga.delta + maxDelta) * (1 + 1e-9)
		if !indexed || capFace != nil && ga.source == capFace || proofbound.IsNonFinite(reach) {
			for _, gb := range fb {
				if err := budget.Step(); err != nil {
					return err
				}
				if err := gateFacePair(ga, gb, capFace, coneFace, examine); err != nil {
					return err
				}
			}
			continue
		}
		candidates, stack = tree.appendWithin(candidates[:0], stack, ga.box, reach)
		slices.Sort(candidates)
		done := 0
		for _, k := range candidates {
			if _, err := budget.StepN(uint64(k + 1 - done)); err != nil {
				return err
			}
			done = k + 1
			if err := gateFacePair(ga, fb[k], capFace, coneFace, examine); err != nil {
				return err
			}
		}
		if _, err := budget.StepN(uint64(len(fb) - done)); err != nil {
			return err
		}
	}
	return nil
}

// gateFacePair calls examine on one face pair unless a cheap rejection decides it: the certified cap pair goes to
// examine with certified set, and any other pair needs a positive summed bound and face boxes within its padded slack.
func gateFacePair(ga, gb faceFacets, capFace, coneFace *topo.Face,
	examine func(ga, gb faceFacets, slack float64, certified bool) error) error {
	if capFace != nil && ga.source == capFace && gb.source == coneFace {
		return examine(ga, gb, 0, true)
	}
	slack := ga.delta + gb.delta
	if slack <= 0 {
		// Both faces are held exactly (a planar face with straight
		// edges triangulates exactly; a Faceted face IS its polygons,
		// core §6.1). A tangency between them is visible to the exact
		// predicates, so there is nothing here a chord could hide.
		return nil
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
		return nil
	}
	return examine(ga, gb, slack, false)
}

// narrowNearContacts returns what GatherNearContacts(ctx, bmA, fis, bmB, fjs, slack, memo) returns when it runs
// right after the gather that returned broad without deferring and logged log, at a broadSlack no smaller than slack.
// It reads the answer from log instead of scanning and classifying again.
//
// Every pair the tight gather queues is in log, in log's order. Both gathers walk fis in order and the positions in
// fjs in ascending order. The tight gather queues a pair only if BoxesWithin keeps it at slack, and BoxesWithin
// keeps it at broadSlack too, because rounded subtraction is monotone. When it queries a box tree it also needs
// BoxesOverlap with the box expandBoxForSlack widens by slack, and the box widened by broadSlack contains that one.
// gatherUsesTree answers the same for both slacks once slack >= 0. So the tight queue is log restricted by the
// tight gather's own two tests, which narrowNearContacts applies.
//
// The broad gather stored every pair it classified, so each queued pair is a memo hit: the tight gather classifies
// nothing, stores nothing and reads the contacts log records. None is a ContactRegion, or the broad gather would
// have deferred. A ContactNone pair farther than broadSlack is farther than slack, because TriTriDistanceExceeds
// never turns from true to false as slack shrinks: its float reading, axis thresholds and exact comparison are
// each monotone in slack (agent-docs/evaluator-design.md §9). Any other ContactNone pair is measured at slack as
// the tight gather measures a memo hit.
//
// The tight gather polls ctx once per 256 facet pairs it scans, once per queued pair as it queues it and again as
// it reads it back, twice per batch it flushes, and once at the end. Nothing it reads between polls changes, so
// narrowNearContacts makes that many polls and returns the first error. A negative or NaN slack runs the gather.
func narrowNearContacts(ctx context.Context, bmA *BoolMesh, fis []int, bmB *BoolMesh, fjs []int, slack float64,
	memo *ContactMemo, broad NearContacts, log []gatheredPair) (NearContacts, bool, error) {
	if !(slack >= 0) {
		return GatherNearContacts(ctx, bmA, fis, bmB, fjs, slack, memo)
	}
	useTree := gatherUsesTree(len(fis), len(fjs), slack)
	var nc NearContacts
	seenA := map[int]struct{}{}
	seenB := map[int]struct{}{}
	queued := 0
	for _, p := range log {
		boxA, boxB := bmA.Boxes[p.I], bmB.Boxes[p.J]
		if useTree && !BoxesOverlap(expandBoxForSlack(boxA, slack), boxB) {
			continue
		}
		if !BoxesWithin(boxA, boxB, slack) {
			continue
		}
		queued++
		if p.Kind == ContactNone && (p.Far || TriTriDistanceExceeds(TriCorners(bmA, p.I), TriCorners(bmB, p.J), slack)) {
			continue
		}
		if p.Span >= 0 {
			nc.Spans = append(nc.Spans, broad.Spans[p.Span])
		}
		if _, seen := seenA[p.I]; !seen {
			seenA[p.I] = struct{}{}
			nc.CloseA = append(nc.CloseA, p.I)
		}
		if _, seen := seenB[p.J]; !seen {
			seenB[p.J] = struct{}{}
			nc.CloseB = append(nc.CloseB, p.J)
		}
	}
	batches := (queued + ContactBatchSize - 1) / ContactBatchSize
	for range len(fis)*len(fjs)/256 + 2*queued + 2*batches + 1 {
		if err := ctx.Err(); err != nil {
			return NearContacts{}, false, err
		}
	}
	return nc, false, nil
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
	// A facet's region near the other side's close facets is inside its own
	// box and within slack of their joint box. Where that region is clear of
	// the facet's chords, its true piece there is within its planar bound.
	near := func(m *BoolMesh, facets []int) [2]r3.Vec {
		lo, hi := m.Boxes[facets[0]][0], m.Boxes[facets[0]][1]
		for _, f := range facets[1:] {
			b := m.Boxes[f]
			lo = r3.Vec{X: math.Min(lo.X, b[0].X), Y: math.Min(lo.Y, b[0].Y), Z: math.Min(lo.Z, b[0].Z)}
			hi = r3.Vec{X: math.Max(hi.X, b[1].X), Y: math.Max(hi.Y, b[1].Y), Z: math.Max(hi.Z, b[1].Z)}
		}
		pad := proofbound.AbsSumUpper(slack, proofbound.ProductUpper(slack, 1e-9))
		return [2]r3.Vec{{X: lo.X - pad, Y: lo.Y - pad, Z: lo.Z - pad}, {X: hi.X + pad, Y: hi.Y + pad, Z: hi.Z + pad}}
	}
	worst := func(m *BoolMesh, facets []int, other [2]r3.Vec) float64 {
		w := 0.0
		for _, f := range facets {
			b := m.FacetBound[f]
			if m.PlanarOwn != nil && m.PlanarOwn[f] < b {
				box := m.Boxes[f]
				lo := r3.Vec{X: math.Max(box[0].X, other[0].X), Y: math.Max(box[0].Y, other[0].Y), Z: math.Max(box[0].Z, other[0].Z)}
				hi := r3.Vec{X: math.Min(box[1].X, other[1].X), Y: math.Min(box[1].Y, other[1].Y), Z: math.Min(box[1].Z, other[1].Z)}
				if ChordClear(lo, hi, m.PlanarChords[f], m.PlanarReach[f]) {
					b = m.PlanarOwn[f]
				}
			}
			w = max(w, b)
		}
		return w
	}
	return proofbound.AbsSumUpper(worst(bmA, nc.CloseA, near(bmB, nc.CloseB)), worst(bmB, nc.CloseB, near(bmA, nc.CloseA)))
}

// flatAcross is the direction in which moving the second operand opens a contact between a curved face ga of the
// first operand and a flat face gb of the second: away from the first operand, along the flat face's inward normal,
// which is the first operand's outward normal there (agent-docs/gap-tolerance-design.md §5.2 item 7). A curved face's
// own normals at its near facets fan round it, as a bore wall's do round its whole rim, so no one move opens them all,
// while the flat face states the single direction the contact opens in, whichever operand comes first. It is zero for
// every other pair, which keeps the first operand's normals, and for every Boolean but an Intersect (AcrossFlat): a
// Union's resting and tangent contacts close along the curved face's own normals.
func flatAcross(ga faceFacets, bmB *BoolMesh, gb faceFacets) r3.Vec {
	if !bmB.AcrossFlat {
		return r3.Vec{}
	}
	if _, flat := ga.source.Surface.(surfacegeom.Plane); flat {
		return r3.Vec{}
	}
	if _, flat := gb.source.Surface.(surfacegeom.Plane); !flat || len(gb.facets) == 0 {
		return r3.Vec{}
	}
	j := gb.facets[0]
	bmB.PrepareFloatNormal(j)
	n, ok := bmB.Fnorms[j].Normalize()
	if !ok {
		return r3.Vec{}
	}
	return n.Scale(-1)
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
//
// across, when nonzero, replaces each facet's normal: a curved face of the first operand near a flat face of the
// second reads the contact along the flat face's normal (flatAcross).
func nearContactNormalsFromContacts(bmA *BoolMesh, nc NearContacts, slack float64, decisive bool, otherFacet int,
	across r3.Vec) []TouchContact {
	type key [3]int64
	seen := map[key]struct{}{}
	var out []TouchContact
	for _, i := range nc.CloseA {
		c := touchOf(bmA, i, otherFacet)
		c.Slack, c.Decisive = slack, decisive
		if across != (r3.Vec{}) {
			c.NormalA = across
		}
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
func facesOfMesh(budget *proofbound.WorkBudget, m *topo.Mesh, boxes [][2]r3.Vec, surface map[*topo.Face]float64) ([]faceFacets, error) {
	// Every grouped face must have a FaceBound entry or the loop returns an
	// error, so len(m.FaceBound) bounds the group count. Capacity only.
	index := make(map[*topo.Face]int, len(m.FaceBound))
	out := make([]faceFacets, 0, len(m.FaceBound))
	for i, f := range m.Source {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		k, ok := index[f]
		if !ok {
			delta, stated := FaceGateDelta(m, surface, f)
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

// ChordClear reports whether the box [lo, hi] lies farther than reach, on
// some axis, from every chord's box. An empty box is clear.
func ChordClear(lo, hi r3.Vec, chords [][2]r3.Vec, reach float64) bool {
	if lo.X > hi.X || lo.Y > hi.Y || lo.Z > hi.Z {
		return true
	}
	for _, c := range chords {
		clo := r3.Vec{X: math.Min(c[0].X, c[1].X), Y: math.Min(c[0].Y, c[1].Y), Z: math.Min(c[0].Z, c[1].Z)}
		chi := r3.Vec{X: math.Max(c[0].X, c[1].X), Y: math.Max(c[0].Y, c[1].Y), Z: math.Max(c[0].Z, c[1].Z)}
		if !(clo.X-hi.X > reach || lo.X-chi.X > reach || clo.Y-hi.Y > reach || lo.Y-chi.Y > reach ||
			clo.Z-hi.Z > reach || lo.Z-chi.Z > reach) {
			return false
		}
	}
	return true
}
