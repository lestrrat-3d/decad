package meshbool

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// FacesNearMiss reports whether two faces' facet sets come within slack of each
// other in a way this evaluator cannot decide (agent-docs/evaluator-design.md §9). When
// the facets stay farther than slack apart everywhere, no touch can be hiding and
// the pair is decided (near = false). When they come within slack — meeting or
// not — the pair is decided only if the facet sets provably INTERPENETRATE deeper
// than b = slack (ProvenDepthExceeds); a bare touch, a shallow meet at depth ≤ b,
// or a non-meeting near-miss stays undecidable (near = true). A coplanar
// face-on-face overlap is a tangency the exact mesh pass refuses as an
// unclassifiable contact (BooleanUnsupportedContact / ErrUnsupported), so it is
// left to that verdict rather than pre-empted here. That deferral is sound only
// while that refusal happens: agent-docs/interference-design.md §5.2 states what must
// settle such a pair before the refusal is removed.
func FacesNearMiss(ctx context.Context, bmA *BoolMesh, fis []int, bmB *BoolMesh, fjs []int, slack float64, memo *ContactMemo) (bool, error) {
	nc, deferred, err := GatherNearContacts(ctx, bmA, fis, bmB, fjs, slack, memo)
	if err != nil || deferred {
		return false, err
	}
	return facesNearMissFromContacts(ctx, bmA, bmB, nc, slack)
}

// facesNearMissFromContacts decides whether nc is a near miss at slack. The
// caller may reuse a broader contact set at a tighter depth threshold: an
// extra facet can only supply another exact witness, and every witness still
// has to prove penetration deeper than slack.
func facesNearMissFromContacts(ctx context.Context, bmA, bmB *BoolMesh, nc NearContacts, slack float64) (bool, error) {
	if len(nc.CloseA) == 0 {
		return false, nil // the facets stay clear of each other: nothing hides
	}
	// A restating operand's facets within the slack are where the pair meets
	// it, so the local chain-depth gate reads them before any depth question
	// is asked (agent-docs/faceted-vertex-bounds-design.md §5).
	if err := refuseCoarseNearContacts(bmA, bmB, nc); err != nil {
		return false, err
	}
	deep, err := ProvenDepthExceeds(ctx, bmA, nc.CloseA, bmB, nc.CloseB, nc.Spans, slack)
	if err != nil {
		return false, err
	}
	return !deep, nil
}

// ProvenDepthExceeds is the reject-only depth discriminator behind the meet
// admission (agent-docs/evaluator-design.md §9, agent-docs/tessellation-design.md §11 step
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
// It looks for a witness in three places: the fixed sample points of every
// contacting facet (DeepWitnessInside), then points on each contact segment's
// two facets walked inward from that segment (SpanWitness), then the points of
// each near facet nearest the sample points of a near facet of the other face
// (NearestWitness). All three only nominate candidates; DeepWitnessAt alone
// certifies one.
func ProvenDepthExceeds(ctx context.Context, bmA *BoolMesh, closeA []int, bmB *BoolMesh, closeB []int, spans []ContactSpan, b float64) (bool, error) {
	deep, err := DeepWitnessInside(ctx, bmA, closeA, bmB, b)
	if err != nil || deep {
		return deep, err
	}
	deep, err = DeepWitnessInside(ctx, bmB, closeB, bmA, b)
	if err != nil || deep {
		return deep, err
	}
	deep, err = SpanWitness(ctx, bmA, bmB, spans, b)
	if err != nil || deep {
		return deep, err
	}
	return NearestWitness(ctx, bmA, closeA, bmB, closeB, b)
}

// NearestWitness looks for a deep witness on each pair of near facets, one of
// each face, whose boxes come within b of each other: the points of facet i
// nearest the sample points of facet j, then the points of facet j nearest the
// sample points of facet i.
//
// The other searches miss a witness where a long facet meets a face that is
// short along it. A cylinder's chord facets run its whole length, so where such
// a wall passes deeply into a thin part of the other operand, every fixed
// sample of the facet lies outside it, and a walk from a contact segment toward
// the facet's far corners leaves the thin part before it gets deeper than b.
// The point of the facet nearest a sample of the other face's facet lies across
// the facet from the contact, inside the other face's extent.
//
// The nearest points are found in floats and only nominate: each candidate is
// rebuilt as an exact convex combination of the facet's exact corners, so it
// lies on the closed facet, and DeepWitnessAt certifies it. A rounded weight
// can only move a candidate along the facet. MaxDepthWitnessPairs caps the
// pairs, which bounds the work the refusal path spends; the cap can only
// refuse.
func NearestWitness(ctx context.Context, bmA *BoolMesh, closeA []int, bmB *BoolMesh, closeB []int, b float64) (bool, error) {
	pairs := 0
	for n, i := range closeA {
		if n >= MaxDepthWitnessFacets {
			break
		}
		for k, j := range closeB {
			if k >= MaxDepthWitnessFacets {
				break
			}
			if !BoxesWithin(bmA.Boxes[i], bmB.Boxes[j], b) {
				continue
			}
			if pairs >= MaxDepthWitnessPairs {
				return false, nil
			}
			pairs++
			if err := ctx.Err(); err != nil {
				return false, err
			}
			deep, err := nearestWitnessOn(ctx, bmA, i, bmB, j, b)
			if err != nil || deep {
				return deep, err
			}
			deep, err = nearestWitnessOn(ctx, bmB, j, bmA, i, b)
			if err != nil || deep {
				return deep, err
			}
		}
	}
	return false, nil
}

// nearestWitnessOn asks DeepWitnessAt about the points of facet fi of m
// nearest each sample point of facet fj of other.
func nearestWitnessOn(ctx context.Context, m *BoolMesh, fi int, other *BoolMesh, fj int, b float64) (bool, error) {
	t := TriCorners(m, fi)
	x := XtriCorners(m, fi)
	// Every candidate lies next to facet fj, so it is tried first.
	hint := fj
	for _, q := range floatSamplePoints(TriCorners(other, fj)) {
		inner, outer, ok := nearestFacetWeights(q, t)
		if !ok {
			continue
		}
		p := proof.Xlerp(proof.Xlerp(x[0], x[1], inner, nearestWeightDen), x[2], outer, nearestWeightDen)
		deep, err := deepWitnessAt(ctx, p, other, b, &hint)
		if err != nil || deep {
			return deep, err
		}
	}
	return false, nil
}

// floatSamplePoints is facetSamplePoints in floats: a facet's three corners,
// its three edge midpoints and its centroid.
func floatSamplePoints(t [3]r3.Vec) [7]r3.Vec {
	mid := func(p, q r3.Vec) r3.Vec { return p.Add(q).Scale(0.5) }
	return [7]r3.Vec{t[0], t[1], t[2], mid(t[0], t[1]), mid(t[1], t[2]), mid(t[2], t[0]),
		t[0].Add(t[1]).Add(t[2]).Scale(1.0 / 3)}
}

// nearestWeightBits is the precision of the dyadic weights nearestFacetWeights
// returns, and nearestWeightDen their common denominator.
const nearestWeightBits = 40

var nearestWeightDen = new(big.Int).Lsh(big.NewInt(1), nearestWeightBits)

// nearestFacetWeights returns dyadic weights, over nearestWeightDen, that place
// the point of triangle t nearest q: the point is lerp(lerp(t0, t1, inner),
// t2, outer). Both weights lie in [0, 1], so the point lies on the closed
// triangle whatever the rounding. It returns false when the floats give no
// weight, as for a degenerate triangle.
func nearestFacetWeights(q r3.Vec, t [3]r3.Vec) (*big.Int, *big.Int, bool) {
	v, w := nearestTriBarycentric(q, t)
	inner := 0.0
	if w < 1 {
		inner = v / (1 - w)
	}
	if !(inner >= 0 && inner <= 1 && w >= 0 && w <= 1) {
		if math.IsNaN(inner) || math.IsNaN(w) || math.IsInf(inner, 0) || math.IsInf(w, 0) {
			return nil, nil, false
		}
		inner = math.Min(math.Max(inner, 0), 1)
		w = math.Min(math.Max(w, 0), 1)
	}
	scale := math.Ldexp(1, nearestWeightBits)
	return big.NewInt(int64(math.Round(inner * scale))), big.NewInt(int64(math.Round(w * scale))), true
}

// nearestTriBarycentric returns (v, w) with t0 + v·(t1 − t0) + w·(t2 − t0)
// the point of triangle t nearest q, by the Voronoi regions of the triangle's
// corners, edges and face.
func nearestTriBarycentric(q r3.Vec, t [3]r3.Vec) (float64, float64) {
	a, b, c := t[0], t[1], t[2]
	ab, ac := b.Sub(a), c.Sub(a)
	ap := q.Sub(a)
	d1, d2 := ab.Dot(ap), ac.Dot(ap)
	if d1 <= 0 && d2 <= 0 {
		return 0, 0
	}
	bp := q.Sub(b)
	d3, d4 := ab.Dot(bp), ac.Dot(bp)
	if d3 >= 0 && d4 <= d3 {
		return 1, 0
	}
	vc := d1*d4 - d3*d2
	if vc <= 0 && d1 >= 0 && d3 <= 0 {
		return d1 / (d1 - d3), 0
	}
	cp := q.Sub(c)
	d5, d6 := ab.Dot(cp), ac.Dot(cp)
	if d6 >= 0 && d5 <= d6 {
		return 0, 1
	}
	vb := d5*d2 - d1*d6
	if vb <= 0 && d2 >= 0 && d6 <= 0 {
		return 0, d2 / (d2 - d6)
	}
	va := d3*d6 - d5*d4
	if va <= 0 && d4-d3 >= 0 && d5-d6 >= 0 {
		w := (d4 - d3) / ((d4 - d3) + (d5 - d6))
		return 1 - w, w
	}
	den := va + vb + vc
	return vb / den, vc / den
}

// DeepWitnessInside reports whether any held-facet sample point of m's
// contacting facets is PROVEN to lie strictly inside the other operand's solid
// deeper than b. The candidates are each facet's vertices, edge midpoints and
// centroid: a facet's deepest penetration need not fall on a mesh vertex (a rod
// pierces a plate through the interior of its wall facets), so the midpoints and
// centroid are sampled too. Reject-only: sampling and the facet cap can only
// miss a witness and refuse, never admit a shallow meet.
func DeepWitnessInside(ctx context.Context, m *BoolMesh, closeFacets []int, other *BoolMesh, b float64) (bool, error) {
	// Neighbouring sample points tend to sit within b of the same facet of
	// other, so the facet that dismissed one is tried first on the next.
	hint := -1
	for n, i := range closeFacets {
		if n >= MaxDepthWitnessFacets {
			break
		}
		if err := ctx.Err(); err != nil {
			return false, err
		}
		tri := m.Tris[i]
		for _, p := range facetSamplePoints(m.Xverts[tri[0]], m.Xverts[tri[1]], m.Xverts[tri[2]]) {
			deep, err := deepWitnessAt(ctx, p, other, b, &hint)
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
	mid := func(p, q proof.Xpt) proof.Xpt { return proof.Xlerp(p, q, one, two) }
	return []proof.Xpt{a, b, c, mid(a, b), mid(b, c), mid(c, a), XCentroid(a, b, c)}
}
