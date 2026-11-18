package meshbool

import (
	"context"
	"fmt"
	"math"
	"slices"
	"sort"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

type pairCutsResult struct {
	CutsA, CutsB       map[int][]Xseg
	BlockedA, BlockedB map[[2]int]struct{}
	Rims               map[string]float64
	MaxRim             float64
	// MinSin is the smallest proven crossing sine over the pair's contact
	// segments, +Inf when none crosses (Stats.MinSin).
	MinSin float64
	// DeepTerm is Stats.DeepTerm.
	DeepTerm float64
	// Contacts is Stats.Contacts.
	Contacts []ContactSite
	// CoplanarA and CoplanarB map each facet of an operand to the facets of
	// the other that form an admitted coplanar pair with it
	// (agent-docs/coplanar-contact-design.md §3.2). Both are nil unless a pair
	// was admitted.
	CoplanarA, CoplanarB map[int][]int
	// Owned holds the exact key of every point the coplanar pass accepted, and
	// is nil unless some pair or edge was admitted.
	Owned map[string]struct{}
	// Legacy is the refusal the pass returns with the coplanar switch off for
	// the same operands, kept for the manifold audit (§5); nil when Owned is
	// empty.
	Legacy error
}

func pairCuts(ctx context.Context, ma, mb *BoolMesh, memo *ContactMemo, keyOf func(proof.Xpt) string) (pairCutsResult, error) {
	// Exact contacts, per facet of each operand. Facet boxes prune the pairs.
	cutsA := map[int][]Xseg{}
	cutsB := map[int][]Xseg{}
	var pointTouches []proof.Xpt
	var pointFacets [][2]int
	var pointCoplanar []bool
	var inPlane []PairContact
	// rims maps each rim vertex (a contact segment's endpoint, by exact key)
	// to the largest RimBound over the facet pairs whose segment ends there.
	rims := map[string]float64{}
	maxRim := 0.0
	minSin := math.Inf(1)
	deepTerm := 0.0
	if ma.Bulge == nil || mb.Bulge == nil {
		deepTerm = math.Inf(1)
	}
	var contactSites []ContactSite
	// touchedA and touchedB are the facets of each operand that meet the
	// other, which the held-mesh gate reads once classification is done.
	var touchedA, touchedB []int
	// coplanar collects the coplanar facet pairs the scan meets, up to
	// maxTouchContacts, so the refusal can say which planes are involved.
	var coplanar []TouchContact
	// The coplanar pass (agent-docs/coplanar-contact-design.md §3.2). refused
	// is set when a coplanar pair holds an inexact facet: that pair, and so the
	// pass, refuses with TouchCoplanar. The legacy lists are the contacts a
	// refusing pass collects for the admitted pairs, edges and points.
	cp := &coplanarPass{coplanarA: map[int][]int{}, coplanarB: map[int][]int{}, owned: map[string]struct{}{}}
	refused := false
	appendLive := func(side, facet int, seg Xseg) error { //nolint:unparam // the shape the bounded pass shares, whose appends can fail
		if side == 0 {
			cutsA[facet] = append(cutsA[facet], seg)
		} else {
			cutsB[facet] = append(cutsB[facet], seg)
		}
		return nil
	}
	work := 0
	boxTree := newBoxTree(mb.Boxes)
	candidates := make([]int, 0, 64)
	contacts := NewContactBatchExecutor(ctx, ma, mb, memo, ContactWorkers(ctx), func(pair ContactPair, c TriContact) error {
		i, j := pair.I, pair.J
		ta := TriCorners(ma, i)
		tb := TriCorners(mb, j)
		if c.Kind == ContactRegion {
			admitted := ma.isExact(i) && mb.isExact(j) || seatPair(ma, mb, i, j)
			if !admitted {
				refused = true
			}
			if len(coplanar) < maxTouchContacts {
				coplanar = append(coplanar, coplanarContactOf(ma, mb, i, j))
			}
			if refused && len(coplanar) >= maxTouchContacts {
				return ExpectedBoolean(BooleanExpectedContact, &TouchContactError{What: touchCoplanar, Contacts: coplanar})
			}
			if admitted {
				return cp.admit(ma, mb, i, j, keyOf, appendLive)
			}
			return nil
		}
		if c.Kind == ContactNone {
			return nil
		}
		// A contact along a wall's rim against a carrier-exact facet is decided by the seat (§16), not by how
		// finely the wall is held: the held-mesh gate does not read the wall's facet there.
		if !rimOnCarrier(mb, j, c.EdgeB, ma, i) {
			touchedB = append(touchedB, j)
		}
		if !rimOnCarrier(ma, i, c.EdgeA, mb, j) {
			touchedA = append(touchedA, i)
		}
		if c.Kind == ContactPoint {
			pointTouches = append(pointTouches, c.P0)
			pointFacets = append(pointFacets, [2]int{i, j})
			// A coplanar point touch of two exact facets is accepted unless a
			// crossing segment ends there, which the rim map settles below.
			pointCoplanar = append(pointCoplanar, c.Coplanar && ma.isExact(i) && mb.isExact(j))
			return nil
		}
		sin := 0.0
		if c.Sin2 != nil {
			sin = SinLowerBound(c.Sin2)
		}
		rim := rimBoundFromSin(ma.SlabBound[i], mb.SlabBound[j], sin, c.Sin2 != nil)
		maxRim = max(maxRim, rim)
		minSin = min(minSin, sin)
		if !math.IsInf(deepTerm, 1) {
			contactSites = append(contactSites, ContactSite{I: i, J: j, Sin: sin, Box: segmentSite(c.P0, c.P1)})
			bulge := max(ma.Bulge[i], mb.Bulge[j])
			if bulge > 0 {
				deepTerm = max(deepTerm, proofbound.DivUpper(proofbound.AbsSumUpper(bulge, bulge), sin))
			}
		}
		for _, p := range []proof.Xpt{c.P0, c.P1} {
			k := keyOf(p)
			if prev, ok := rims[k]; !ok || rim > prev {
				rims[k] = rim
			}
		}
		if c.EdgeA >= 0 || c.EdgeB >= 0 {
			// The segment runs ALONG a facet edge. Graze or crossing is not
			// decidable here — hold it for the mesh-level call below.
			inPlane = append(inPlane, PairContact{I: i, J: j, Ta: ta, Tb: tb, C: c})
			return nil
		}
		cutsA[i] = append(cutsA[i], Xseg{
			A: c.P0, B: c.P1,
			AOnEdge: c.P0OnA, BOnEdge: c.P1OnA,
			Partner: tb,
		})
		cutsB[j] = append(cutsB[j], Xseg{
			A: c.P0, B: c.P1,
			AOnEdge: c.P0OnB, BOnEdge: c.P1OnB,
			Partner: ta,
		})
		return nil
	})
	for i := range ma.Tris {
		candidates = candidates[:0]
		err := boxTree.visitOverlaps(ma.Boxes[i], ctx.Err, func(j int) error {
			candidates = append(candidates, j)
			return nil
		})
		if err != nil {
			return pairCutsResult{}, err
		}
		sort.Ints(candidates)
		for _, j := range candidates {
			work++
			if work%256 == 0 {
				if err := ctx.Err(); err != nil {
					return pairCutsResult{}, err
				}
			}
			if err := contacts.Add(i, j); err != nil {
				return pairCutsResult{}, err
			}
		}
	}
	if err := contacts.Done(); err != nil {
		return pairCutsResult{}, err
	}
	if refused {
		return pairCutsResult{}, ExpectedBoolean(BooleanExpectedContact, &TouchContactError{What: touchCoplanar, Contacts: coplanar})
	}
	// The held-mesh gate runs on the classification's own answer and before
	// any facet is cut (agent-docs/faceted-vertex-bounds-design.md §5).
	if err := RefuseCoarseHeldContact(ma, 0, touchedA); err != nil {
		return pairCutsResult{}, err
	}
	if err := RefuseCoarseHeldContact(mb, 1, touchedB); err != nil {
		return pairCutsResult{}, err
	}

	// The graze-or-crossing call, made once, OUTSIDE the pair loop: an in-plane
	// facet edge grazes the other operand exactly when the edge's two adjacent
	// facets lie strictly on ONE side of the other facet's plane — the operand's
	// boundary touches the plane and comes back, a tangency no side
	// classification can be proven for. Apexes that STRADDLE the plane are an
	// ordinary crossing: the boundary genuinely passes through, and the segment
	// is a real rim of the result.
	blockedA := map[[2]int]struct{}{}
	blockedB := map[[2]int]struct{}{}
	// grazes collects the facets of the first operand at every graze, so the
	// refusal can say where the operands touch.
	var grazes []TouchContact
	// realGraze is set by a graze no coplanar patch accounts for; only then
	// do the grazes refuse, and the refusal lists every contact in order.
	realGraze := false
	for i, p := range inPlane {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return pairCutsResult{}, err
			}
		}
		// skipCut is set by an edge that bounds an admitted coplanar patch: the
		// segment runs on the facet's own edge and the admitted pair already
		// cut the partner.
		skipCut, grazed := false, false
		if p.C.EdgeA >= 0 {
			key, side, adjacent, err := EdgeSide(ma, p.I, p.C.EdgeA, p.Tb)
			if err != nil {
				return pairCutsResult{}, err
			}
			if side == EdgeCoplanarBounded && (ma.isExact(p.I) && mb.isExact(p.J) ||
				rimEdge(ma, p.I, p.C.EdgeA) && slices.Contains(cp.coplanarA[adjacent], p.J)) {
				if err := cp.acceptEdge(p, adjacent, p.J, keyOf); err != nil {
					return pairCutsResult{}, err
				}
				skipCut, grazed = true, true
			} else if side != EdgeCrossing {
				if len(grazes) < maxTouchContacts {
					grazes = append(grazes, touchOf(ma, p.I, p.J))
				}
				realGraze = true
				continue
			} else {
				blockedA[key] = struct{}{}
			}
		}
		if p.C.EdgeB >= 0 {
			key, side, adjacent, err := EdgeSide(mb, p.J, p.C.EdgeB, p.Ta)
			if err != nil {
				return pairCutsResult{}, err
			}
			if side == EdgeCoplanarBounded && (ma.isExact(p.I) && mb.isExact(p.J) ||
				rimEdge(mb, p.J, p.C.EdgeB) && slices.Contains(cp.coplanarA[p.I], adjacent)) {
				if err := cp.acceptEdge(p, p.I, adjacent, keyOf); err != nil {
					return pairCutsResult{}, err
				}
				skipCut, grazed = true, true
			} else if side != EdgeCrossing {
				if len(grazes) < maxTouchContacts {
					grazes = append(grazes, touchOf(ma, p.I, p.J))
				}
				realGraze = true
				continue
			} else {
				blockedB[key] = struct{}{}
			}
		}
		if grazed && len(grazes) < maxTouchContacts {
			// The refusal the pair gives with no accepted edge records one
			// graze per contact, however many of its edges graze.
			grazes = append(grazes, touchOf(ma, p.I, p.J))
		}
		if skipCut {
			continue
		}
		// A crossing segment subdivides whichever facet does NOT already own it
		// as an edge — a segment lying along a facet's own boundary cuts nothing
		// off it. Its regions classify by exact parity, never by the partner
		// facet's plane: the other operand's boundary there is the DIHEDRAL
		// between two facets, and one plane of it decides nothing.
		if p.C.EdgeA < 0 {
			cutsA[p.I] = append(cutsA[p.I], Xseg{
				A: p.C.P0, B: p.C.P1,
				AOnEdge: p.C.P0OnA, BOnEdge: p.C.P1OnA,
				Partner: p.Tb, ViaParity: true,
			})
		}
		if p.C.EdgeB < 0 {
			cutsB[p.J] = append(cutsB[p.J], Xseg{
				A: p.C.P0, B: p.C.P1,
				AOnEdge: p.C.P0OnB, BOnEdge: p.C.P1OnB,
				Partner: p.Ta, ViaParity: true,
			})
		}
	}

	if realGraze {
		if len(coplanar) > 0 {
			// The pair always refused its coplanar contacts first, and the
			// retry reads that refusal.
			return pairCutsResult{}, ExpectedBoolean(BooleanExpectedContact, &TouchContactError{What: touchCoplanar, Contacts: coplanar})
		}
		return pairCutsResult{}, ExpectedBoolean(BooleanExpectedContact, &TouchContactError{What: touchGraze, Contacts: grazes})
	}
	cp.legacyGrazes = grazes

	// A point contact is legitimate only as the endpoint of some crossing
	// segment (a chain passing exactly through a vertex). An isolated one is a
	// tangency the boolean cannot classify: the operands pinch at a point, and
	// stitching it would emit a non-manifold result — refuse, never a wrong mesh.
	if len(pointTouches) > 0 {
		// every lists each point contact no crossing segment ends at, in order,
		// the accepted ones too, so a refusal reads as it does with none accepted.
		var every []TouchContact
		isolated := false
		// A point a coplanar pair of exact facets touches is owned whatever the
		// order its contacts arrive in.
		for k, p := range pointTouches {
			if pointCoplanar[k] {
				if _, ok := rims[keyOf(p)]; !ok {
					cp.owned[keyOf(p)] = struct{}{}
				}
			}
		}
		for k, p := range pointTouches {
			key := keyOf(p)
			if _, ok := rims[key]; ok {
				continue
			}
			if len(every) < maxTouchContacts {
				every = append(every, touchOf(ma, pointFacets[k][0], pointFacets[k][1]))
			}
			if seatPoint(ma, mb, pointFacets[k][0], pointFacets[k][1]) {
				_, owned := cp.owned[key]
				if pointCoplanar[k] || owned {
					cp.owned[key] = struct{}{}
					continue
				}
			}
			isolated = true
		}
		if isolated {
			if len(coplanar) > 0 {
				return pairCutsResult{}, ExpectedBoolean(BooleanExpectedContact, &TouchContactError{What: touchCoplanar, Contacts: coplanar})
			}
			return pairCutsResult{}, ExpectedBoolean(BooleanExpectedContact, &TouchContactError{What: touchPoint, Contacts: every})
		}
		cp.legacyPoints = every
	}

	res := pairCutsResult{CutsA: cutsA, CutsB: cutsB, BlockedA: blockedA, BlockedB: blockedB, Rims: rims,
		MaxRim: maxRim, MinSin: minSin, DeepTerm: deepTerm, Contacts: contactSites}
	if len(cp.coplanarA) > 0 {
		res.CoplanarA, res.CoplanarB = cp.coplanarA, cp.coplanarB
	}
	if len(cp.owned) > 0 {
		res.Owned = cp.owned
		res.Legacy = cp.legacy(coplanar)
	}
	return res, nil
}

// segmentSite is a float box that holds the exact contact segment p0–p1:
// each rounded endpoint is within half an ulp, which one ulp outward covers.
func segmentSite(p0, p1 proof.Xpt) [2]r3.Vec {
	a, b := p0.Vec(), p1.Vec()
	inf := math.Inf(1)
	return [2]r3.Vec{
		{X: math.Nextafter(math.Min(a.X, b.X), -inf), Y: math.Nextafter(math.Min(a.Y, b.Y), -inf), Z: math.Nextafter(math.Min(a.Z, b.Z), -inf)},
		{X: math.Nextafter(math.Max(a.X, b.X), inf), Y: math.Nextafter(math.Max(a.Y, b.Y), inf), Z: math.Nextafter(math.Max(a.Z, b.Z), inf)},
	}
}

// isExact reports whether facet t is exact (BoolMesh.Exact); a mesh that never
// had its facets read has none.
func (bm *BoolMesh) isExact(t int) bool { return bm.Exact != nil && bm.Exact[t] }

// coplanarPass is the state of the exact coplanar pass over one pairCuts call
// (agent-docs/coplanar-contact-design.md §3.2–§3.3).
type coplanarPass struct {
	// twin reads the facet across an edge, and charge meters the exact work of
	// the bounded replay pass; both are nil for the live pass, which reads the
	// operand's own owner table and meters nothing.
	twin                 func(m *BoolMesh, f, k int) (int, bool, error)
	charge               func(n uint64) error
	reserve              func(n uint64) error
	coplanarA, coplanarB map[int][]int
	owned                map[string]struct{}
	// legacyGrazes and legacyPoints are the contacts the switch-off pass would
	// have collected for what this pass accepted; the coplanar facet pairs
	// themselves are pairCuts' own list.
	legacyGrazes, legacyPoints []TouchContact
}

// admit records the admitted coplanar pair (i, j): it inserts each operand's
// patch-boundary edges, clipped to the other facet, as cuts, and owns the
// clipped end points.
func (cp *coplanarPass) admit(ma, mb *BoolMesh, i, j int, keyOf func(proof.Xpt) string, appendCut func(side, facet int, seg Xseg) error) error {
	cp.coplanarA[i] = append(cp.coplanarA[i], j)
	cp.coplanarB[j] = append(cp.coplanarB[j], i)
	if err := cp.clipEdges(mb, j, ma, i, keyOf, func(seg Xseg) error { return appendCut(0, i, seg) }); err != nil {
		return err
	}
	return cp.clipEdges(ma, i, mb, j, keyOf, func(seg Xseg) error { return appendCut(1, j, seg) })
}

// clipEdges clips the patch-boundary edges of facet src of from to the closed
// facet dst of into and appends each surviving piece to cuts[dst].
func (cp *coplanarPass) clipEdges(from *BoolMesh, src int, into *BoolMesh, dst int, keyOf func(proof.Xpt) string, add func(Xseg) error) error {
	xs, xd := XtriCorners(from, src), XtriCorners(into, dst)
	partner := TriCorners(from, src)
	for k := range 3 {
		boundary, err := cp.patchBoundary(from, src, k)
		if err != nil {
			return err
		}
		if !boundary {
			continue
		}
		if cp.charge != nil {
			if err := cp.charge(coplanarClipWork); err != nil {
				return err
			}
		}
		piece, ok, err := CoplanarClip([2]proof.Xpt{xs[k], xs[(k+1)%3]}, xd, into.Norms[dst])
		if err != nil {
			return err
		}
		if !ok {
			continue
		}
		if cp.reserve != nil {
			// A kept piece holds a cut and two owned keys.
			if err := cp.reserve(8); err != nil {
				return err
			}
		}
		if err := add(Xseg{
			A: piece.P0, B: piece.P1,
			AOnEdge: piece.P0OnA, BOnEdge: piece.P1OnA,
			Partner: partner, ViaParity: true,
		}); err != nil {
			return err
		}
		cp.owned[keyOf(piece.P0)] = struct{}{}
		cp.owned[keyOf(piece.P1)] = struct{}{}
	}
	return nil
}

// coplanarClipWork is the work a bounded pass charges for one exact clip of an
// edge to a triangle.
const coplanarClipWork = 512

// patchBoundary is patchBoundaryEdge through the pass's own twin lookup.
func (cp *coplanarPass) patchBoundary(m *BoolMesh, f, k int) (bool, error) {
	return patchBoundaryEdgeBy(m, f, k, cp.twin)
}

// patchBoundaryEdgeBy reports whether edge k of facet f bounds a planar patch of
// its operand: the facet across it is not coplanar with f. An edge between two
// coplanar facets is a triangulation diagonal and is never inserted
// (agent-docs/coplanar-contact-design.md §3.2).
func patchBoundaryEdgeBy(m *BoolMesh, f, k int, lookup func(m *BoolMesh, f, k int) (int, bool, error)) (bool, error) {
	var twin int
	var ok bool
	if lookup != nil {
		var err error
		twin, ok, err = lookup(m, f, k)
		if err != nil {
			return false, err
		}
	} else {
		twin, ok = m.TwinFacet(f, k)
	}
	if !ok {
		return false, fmt.Errorf(`%w: a coplanar facet edge has no twin facet`, decaderr.ErrBooleanFailed)
	}
	t, tt := m.Tris[f], m.Tris[twin]
	u, v := t[k], t[(k+1)%3]
	apex := m.Verts[tt[0]+tt[1]+tt[2]-u-v]
	return proof.OrientSign(m.Verts[t[0]], m.Verts[t[1]], m.Verts[t[2]], apex) != 0, nil
}

// acceptEdge accepts an in-plane edge that bounds a coplanar patch (§3.3). The
// facet of the first operand is a, of the second b; adjacent is the facet of
// the edge's own operand that lies in the plane, and the pair it forms with the
// partner facet must be one the pair pass admitted. It owns the contact segment's end points.
func (cp *coplanarPass) acceptEdge(p PairContact, a, b int, keyOf func(proof.Xpt) string) error {
	if cp.charge != nil {
		// Two exact keys and two owned entries.
		if err := cp.charge(128); err != nil {
			return err
		}
		if err := cp.reserve(4); err != nil {
			return err
		}
	}
	recorded := cp.coplanarA[a]
	if !slices.Contains(recorded, b) {
		return fmt.Errorf(`%w: an in-plane edge bounds a coplanar patch no admitted pair recorded`, decaderr.ErrBooleanFailed)
	}
	cp.owned[keyOf(p.C.P0)] = struct{}{}
	cp.owned[keyOf(p.C.P1)] = struct{}{}
	return nil
}

// legacy is the refusal the pass returns with the switch off for the contacts
// this one accepted, in that pass's order: coplanar facet pairs, then edge
// grazes, then isolated points (§5).
func (cp *coplanarPass) legacy(coplanar []TouchContact) error {
	switch {
	case len(coplanar) > 0:
		return ExpectedBoolean(BooleanExpectedContact, &TouchContactError{What: touchCoplanar, Contacts: coplanar})
	case len(cp.legacyGrazes) > 0:
		return ExpectedBoolean(BooleanExpectedContact, &TouchContactError{What: touchGraze, Contacts: cp.legacyGrazes})
	case len(cp.legacyPoints) > 0:
		return ExpectedBoolean(BooleanExpectedContact, &TouchContactError{What: touchPoint, Contacts: cp.legacyPoints})
	}
	return nil
}
