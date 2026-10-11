package decad

import (
	"fmt"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// loftCylinderConnectorContactBound bounds a contact on one retained root
// connector against the exposed source tooth/cylinder seam at the same
// height. The caller must hold an authenticated, unplaced join or bore record:
// its other teeth cannot cover this connector, and its bore stays inside the
// blank. Both exact contact endpoints must lie on one of the two segments of
// the held connector's horizontal slice.
//
// At this height, the source connector starts inside the exact blank and ends
// outside it. sourceBound covers the matched source-to-held facet departure.
// Every source circle crossing therefore maps to a held point
// with radius in [R-sourceBound,R+sourceBound]. The radial-monotone held slice
// makes that set one bounded strip. The maximum distance from either contact
// endpoint to the strip, plus sourceBound, covers the entire contact segment
// by convexity. It is a local bound, not the joined body's root-band bound.
func loftCylinderConnectorContactBound(lp loftPayload, root, connectorCell int,
	radius float64, level *big.Rat, p0, p1 proofarith.Xpt, sourceBound float64) (float64, error) {
	refuse := func(reason string) (float64, error) {
		return 0, fmt.Errorf("%w: %s", ErrUnsupported, reason)
	}
	if level == nil || radius <= 0 || proofbound.IsNonFinite(radius) ||
		sourceBound < 0 || proofbound.IsNonFinite(sourceBound) ||
		!lp.authenticatedReconstruction || lp.sourceIdentity == nil || lp.surfaceResult ||
		sourceBound < lp.proof.FacetDeparture || !loftCylinderAxialPlacement(lp.xform) {
		return refuse("connector contact has no matched Loft source bound")
	}
	n := len(lp.verts) / 2
	if len(lp.verts)%2 != 0 || n < 4 || root < 3 || root >= n-1 ||
		(connectorCell != 0 && connectorCell != root-1) ||
		len(lp.tris) != 4*n-4 || len(lp.segment) != 1 || len(lp.segment[0]) != n ||
		len(lp.profile0.Outer.Segments) != 6 || len(lp.profile1.Outer.Segments) != 6 ||
		!loftCylinderSectionKinds(lp) {
		return refuse("contact names no authenticated root connector")
	}
	z0, z1 := lp.verts[0].Z, lp.verts[n].Z
	if !proofbound.FiniteVec(r3.Vec{Z: z0}) || !proofbound.FiniteVec(r3.Vec{Z: z1}) || z1 <= z0 ||
		lp.frame0.Origin() != (r3.Vec{Z: z0}) || lp.frame1.Origin() != (r3.Vec{Z: z1}) ||
		level.Cmp(proofarith.FloatRat(z0)) <= 0 || level.Cmp(proofarith.FloatRat(z1)) >= 0 {
		return refuse("connector contact leaves the exact axial span")
	}
	r := proofarith.FloatRat(radius)
	delta := proofarith.FloatRat(sourceBound)
	if r.Cmp(delta) <= 0 {
		return refuse("connector source bound reaches the blank axis")
	}
	height := proofbound.RatFloatUp(new(big.Rat).Sub(proofarith.FloatRat(z1), proofarith.FloatRat(z0)))
	sourceCollar, ok := loftCylinderRootCollar(lp, radius, height)
	if !ok {
		return refuse("source root has no inside-circle collar")
	}
	heldCollar, ok := loftCylinderConnectorGate(lp, root, radius, height, sourceBound)
	if !ok {
		return refuse("held connector has no one-crossing proof")
	}
	belowTop := new(big.Rat).Sub(proofarith.FloatRat(z1), level)
	aboveBottom := new(big.Rat).Sub(level, proofarith.FloatRat(z0))
	collar := proofarith.FloatRat(max(sourceCollar, heldCollar))
	if belowTop.Cmp(collar) < 0 || aboveBottom.Cmp(collar) < 0 {
		return refuse("connector contact lies in an uncertified root collar")
	}
	near, far := 0, 1
	if connectorCell != 0 {
		near, far = root, root-1
	}
	for _, index := range []int{near, far, n + near, n + far} {
		if lp.verts[index].Z != z0 && lp.verts[index].Z != z1 {
			return refuse("connector vertex leaves its cap plane")
		}
	}
	if lp.verts[near].Z != z0 || lp.verts[far].Z != z0 ||
		lp.verts[n+near].Z != z1 || lp.verts[n+far].Z != z1 {
		return refuse("connector rulings do not share the cap planes")
	}
	var shared []int
	for _, v := range lp.tris[2*connectorCell] {
		for _, w := range lp.tris[2*connectorCell+1] {
			if v == w {
				shared = append(shared, v)
			}
		}
	}
	if len(shared) != 2 {
		return refuse("connector triangles have no common diagonal")
	}
	diagonal := [2]int{-1, -1}
	for _, v := range shared {
		switch v {
		case near, far:
			diagonal[0] = v
		case n + near, n + far:
			diagonal[1] = v
		default:
			return refuse("connector diagonal leaves its endpoint rulings")
		}
	}
	if diagonal[0] < 0 || diagonal[1] < 0 {
		return refuse("connector diagonal has no two cap endpoints")
	}
	t := new(big.Rat).Quo(new(big.Rat).Sub(level, proofarith.FloatRat(z0)),
		new(big.Rat).Sub(proofarith.FloatRat(z1), proofarith.FloatRat(z0)))
	a := loftCylinderContactSlice(lp.verts[near], lp.verts[n+near], t)
	d := loftCylinderContactSlice(lp.verts[diagonal[0]], lp.verts[diagonal[1]], t)
	f := loftCylinderContactSlice(lp.verts[far], lp.verts[n+far], t)
	contact0, ok0 := loftCylinderContactPoint(p0, level)
	contact1, ok1 := loftCylinderContactPoint(p1, level)
	if !ok0 || !ok1 {
		return refuse("exact contact leaves its cap-side level")
	}
	onFirst := loftCylinderContactOnSegment(contact0, a, d) &&
		loftCylinderContactOnSegment(contact1, a, d)
	onLast := loftCylinderContactOnSegment(contact0, d, f) &&
		loftCylinderContactOnSegment(contact1, d, f)
	if !onFirst && !onLast {
		return refuse("exact contact is not on one connector slice segment")
	}
	lower := new(big.Rat).Sub(r, delta)
	upper := new(big.Rat).Add(r, delta)
	lower2 := new(big.Rat).Mul(lower, lower)
	upper2 := new(big.Rat).Mul(upper, upper)
	nearContact, farContact := contact0, contact1
	if loftCylinderContactDot(nearContact, nearContact).Cmp(loftCylinderContactDot(farContact, farContact)) > 0 {
		nearContact, farContact = farContact, nearContact
	}
	// Beyond R+sourceBound the matched source wall stays outside the blank,
	// so its ordinary Loft bound suffices. Restrict the seam-distance charge
	// to the possibly hidden portion of this exact contact segment.
	if loftCylinderContactDot(nearContact, nearContact).Cmp(upper2) > 0 {
		return sourceBound, nil
	}
	if loftCylinderContactDot(farContact, farContact).Cmp(upper2) > 0 {
		farContact = loftCylinderContactClipFar(nearContact, farContact, upper2)
	}
	maximum := 0.0
	found := false
	for _, segment := range [][2]loftCylinderXY{{a, d}, {d, f}} {
		ends, inside := loftCylinderContactStrip(segment[0], segment[1], lower2, upper2)
		if !inside {
			continue
		}
		found = true
		for _, contact := range []loftCylinderXY{nearContact, farContact} {
			for _, end := range ends {
				gap := proofbound.RatSqrtUp(loftCylinderContactDistance2(contact, end))
				if proofbound.IsNonFinite(gap) {
					return refuse("contact-to-seam distance has no finite bound")
				}
				maximum = max(maximum, gap)
			}
		}
	}
	if !found {
		return refuse("connector has no buffered cylinder crossing")
	}
	bound := proofbound.AbsSumUpper(maximum, sourceBound)
	if proofbound.IsNonFinite(bound) {
		return refuse("source connector contact has no finite local bound")
	}
	return bound, nil
}

// near is no farther than the threshold and far is beyond it. The returned
// parameter is just outside the threshold, so it contains every possibly
// hidden point of the contact segment.
func loftCylinderContactClipFar(near, far loftCylinderXY, upper2 *big.Rat) loftCylinderXY {
	lo, hi := big.NewRat(0, 1), big.NewRat(1, 1)
	for range 52 {
		mid := new(big.Rat).Quo(new(big.Rat).Add(lo, hi), big.NewRat(2, 1))
		p := loftCylinderContactLerp(near, far, mid)
		if loftCylinderContactDot(p, p).Cmp(upper2) <= 0 {
			lo = mid
		} else {
			hi = mid
		}
	}
	return loftCylinderContactLerp(near, far, hi)
}

func loftCylinderContactSlice(a, b r3.Vec, t *big.Rat) loftCylinderXY {
	lerp := func(x, y float64) *big.Rat {
		return new(big.Rat).Add(proofarith.FloatRat(x), new(big.Rat).Mul(t,
			new(big.Rat).Sub(proofarith.FloatRat(y), proofarith.FloatRat(x))))
	}
	return loftCylinderXY{x: lerp(a.X, b.X), y: lerp(a.Y, b.Y)}
}

func loftCylinderContactPoint(p proofarith.Xpt, level *big.Rat) (loftCylinderXY, bool) {
	if p.X == nil || p.Y == nil || p.Z == nil || p.W == nil || p.W.Sign() <= 0 ||
		new(big.Rat).SetFrac(p.Z, p.W).Cmp(level) != 0 {
		return loftCylinderXY{}, false
	}
	return loftCylinderXY{x: new(big.Rat).SetFrac(p.X, p.W),
		y: new(big.Rat).SetFrac(p.Y, p.W)}, true
}

func loftCylinderContactSub(a, b loftCylinderXY) loftCylinderXY {
	return loftCylinderXY{x: new(big.Rat).Sub(a.x, b.x), y: new(big.Rat).Sub(a.y, b.y)}
}

func loftCylinderContactDot(a, b loftCylinderXY) *big.Rat {
	return new(big.Rat).Add(new(big.Rat).Mul(a.x, b.x), new(big.Rat).Mul(a.y, b.y))
}

func loftCylinderContactDistance2(a, b loftCylinderXY) *big.Rat {
	delta := loftCylinderContactSub(a, b)
	return loftCylinderContactDot(delta, delta)
}

func loftCylinderContactLerp(a, b loftCylinderXY, t *big.Rat) loftCylinderXY {
	v := loftCylinderContactSub(b, a)
	return loftCylinderXY{
		x: new(big.Rat).Add(a.x, new(big.Rat).Mul(t, v.x)),
		y: new(big.Rat).Add(a.y, new(big.Rat).Mul(t, v.y)),
	}
}

func loftCylinderContactOnSegment(p, a, b loftCylinderXY) bool {
	v, w := loftCylinderContactSub(b, a), loftCylinderContactSub(p, a)
	cross := new(big.Rat).Sub(new(big.Rat).Mul(v.x, w.y), new(big.Rat).Mul(v.y, w.x))
	if cross.Sign() != 0 {
		return false
	}
	position := loftCylinderContactDot(v, w)
	return position.Sign() >= 0 && position.Cmp(loftCylinderContactDot(v, v)) <= 0
}

// Each segment is radial-monotone under the connector gate. Bisection keeps
// the outside parameter endpoints, so the returned interval contains every
// point whose exact squared radius lies in [lower2,upper2].
func loftCylinderContactStrip(a, b loftCylinderXY, lower2, upper2 *big.Rat) ([2]loftCylinderXY, bool) {
	var empty [2]loftCylinderXY
	v := loftCylinderContactSub(b, a)
	if loftCylinderContactDot(a, v).Sign() < 0 || loftCylinderContactDot(b, v).Sign() < 0 ||
		loftCylinderContactDot(b, b).Cmp(lower2) < 0 ||
		loftCylinderContactDot(a, a).Cmp(upper2) > 0 {
		return empty, false
	}
	point := func(t *big.Rat) loftCylinderXY { return loftCylinderContactLerp(a, b, t) }
	first, last := big.NewRat(0, 1), big.NewRat(1, 1)
	if loftCylinderContactDot(a, a).Cmp(lower2) < 0 {
		lo, hi := big.NewRat(0, 1), big.NewRat(1, 1)
		for range 52 {
			mid := new(big.Rat).Quo(new(big.Rat).Add(lo, hi), big.NewRat(2, 1))
			p := point(mid)
			if loftCylinderContactDot(p, p).Cmp(lower2) < 0 {
				lo = mid
			} else {
				hi = mid
			}
		}
		first = lo
	}
	if loftCylinderContactDot(b, b).Cmp(upper2) > 0 {
		lo, hi := big.NewRat(0, 1), big.NewRat(1, 1)
		for range 52 {
			mid := new(big.Rat).Quo(new(big.Rat).Add(lo, hi), big.NewRat(2, 1))
			p := point(mid)
			if loftCylinderContactDot(p, p).Cmp(upper2) <= 0 {
				lo = mid
			} else {
				hi = mid
			}
		}
		last = hi
	}
	return [2]loftCylinderXY{point(first), point(last)}, true
}
