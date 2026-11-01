package meshbool

import (
	"context"
	"sort"

	"github.com/lestrrat-3d/decad/internal/proof"
)

type pairCutsResult struct {
	CutsA, CutsB       map[int][]Xseg
	BlockedA, BlockedB map[[2]int]struct{}
	Rims               map[string]float64
	MaxRim             float64
}

func pairCuts(ctx context.Context, ma, mb *BoolMesh, memo *ContactMemo, keyOf func(proof.Xpt) string) (pairCutsResult, error) {
	// Exact contacts, per facet of each operand. Facet boxes prune the pairs.
	cutsA := map[int][]Xseg{}
	cutsB := map[int][]Xseg{}
	var pointTouches []proof.Xpt
	var pointFacets [][2]int
	var segEnds []proof.Xpt
	var inPlane []PairContact
	// rims maps each rim vertex (a contact segment's endpoint, by exact key)
	// to the largest RimBound over the facet pairs whose segment ends there.
	rims := map[string]float64{}
	maxRim := 0.0
	// touchedA and touchedB are the facets of each operand that meet the
	// other, which the held-mesh gate reads once classification is done.
	var touchedA, touchedB []int
	// coplanar collects the coplanar facet pairs the scan meets, up to
	// maxTouchContacts, so the refusal can say which planes are involved.
	var coplanar []TouchContact
	work := 0
	boxTree := newBoxTree(mb.Boxes)
	candidates := make([]int, 0, 64)
	contacts := NewContactBatchExecutor(ctx, ma, mb, memo, ContactWorkers(ctx), func(pair ContactPair, c TriContact) error {
		i, j := pair.I, pair.J
		ta := TriCorners(ma, i)
		tb := TriCorners(mb, j)
		if c.Kind == ContactRegion {
			coplanar = append(coplanar, coplanarContactOf(ma, mb, i, j))
			if len(coplanar) >= maxTouchContacts {
				return ExpectedBoolean(BooleanExpectedContact, &TouchContactError{What: touchCoplanar, Contacts: coplanar})
			}
			return nil
		}
		if c.Kind == ContactNone {
			return nil
		}
		touchedA = append(touchedA, i)
		touchedB = append(touchedB, j)
		if c.Kind == ContactPoint {
			pointTouches = append(pointTouches, c.P0)
			pointFacets = append(pointFacets, [2]int{i, j})
			return nil
		}
		segEnds = append(segEnds, c.P0, c.P1)
		rim := RimBound(ma.SlabBound[i], mb.SlabBound[j], c.Sin2)
		maxRim = max(maxRim, rim)
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
	if len(coplanar) > 0 {
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
	for i, p := range inPlane {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return pairCutsResult{}, err
			}
		}
		if p.C.EdgeA >= 0 {
			key, crossing, err := EdgeCrosses(ma, p.I, p.C.EdgeA, p.Tb)
			if err != nil {
				return pairCutsResult{}, err
			}
			if !crossing {
				if len(grazes) < maxTouchContacts {
					grazes = append(grazes, touchOf(ma, p.I, p.J))
				}
				continue
			}
			blockedA[key] = struct{}{}
		}
		if p.C.EdgeB >= 0 {
			key, crossing, err := EdgeCrosses(mb, p.J, p.C.EdgeB, p.Ta)
			if err != nil {
				return pairCutsResult{}, err
			}
			if !crossing {
				if len(grazes) < maxTouchContacts {
					grazes = append(grazes, touchOf(ma, p.I, p.J))
				}
				continue
			}
			blockedB[key] = struct{}{}
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

	if len(grazes) > 0 {
		return pairCutsResult{}, ExpectedBoolean(BooleanExpectedContact, &TouchContactError{What: touchGraze, Contacts: grazes})
	}

	// A point contact is legitimate only as the endpoint of some crossing
	// segment (a chain passing exactly through a vertex). An isolated one is a
	// tangency the boolean cannot classify: the operands pinch at a point, and
	// stitching it would emit a non-manifold result — refuse, never a wrong mesh.
	if len(pointTouches) > 0 {
		ends := map[string]struct{}{}
		for _, p := range segEnds {
			ends[keyOf(p)] = struct{}{}
		}
		var isolated []TouchContact
		for k, p := range pointTouches {
			if _, ok := ends[keyOf(p)]; !ok && len(isolated) < maxTouchContacts {
				isolated = append(isolated, touchOf(ma, pointFacets[k][0], pointFacets[k][1]))
			}
		}
		if len(isolated) > 0 {
			return pairCutsResult{}, ExpectedBoolean(BooleanExpectedContact, &TouchContactError{What: touchPoint, Contacts: isolated})
		}
	}

	return pairCutsResult{cutsA, cutsB, blockedA, blockedB, rims, maxRim}, nil
}
