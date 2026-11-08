package planar

import (
	"cmp"
	"errors"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/decad/internal/proof"
)

// PlanarDeepVertex reports whether a vertex of either audited solid lies
// strictly inside the other solid, farther than margin from its boundary.
// Such a vertex proves the interiors overlap after moving each solid by any
// map that displaces its points by at most mA and mB with mA + mB <= margin:
// the ball of radius margin around the vertex lies in the other solid, so its
// moved image still holds the moved vertex in that solid's interior, and every
// neighbourhood of a boundary vertex meets its own solid's interior. The
// rotating sweep (agent-docs/multibody-dynamics-design.md §10.1) uses it to carry a
// float-pose overlap to the ideal path. margin must be nonnegative; poll is
// charged once per exact predicate group.
func PlanarDeepVertex(a, b *PlanarSolid, margin proof.Dyadic, poll func() error) (bool, error) {
	marginSquared := proof.DyMul(margin, margin)
	pa, pb := preparePlanar(a), preparePlanar(b)
	// The float boxes turn on deepInside's gap pre-test, the cast's axis
	// pre-test and the facet box pre-test (planar_prune.go); none changes an
	// answer or a poll.
	pa.floatBoxes()
	pb.floatBoxes()
	_, marginUp := proof.FloatBounds(marginSquared)
	k := &planarKernel{a: pa, b: pb, poll: poll}
	for _, side := range [][2]*planarPrep{{pa, pb}, {pb, pa}} {
		from, to := side[0], side[1]
		for v, point := range from.s.Verts {
			deep, err := k.deepInside(point, from.vertBox[v], to, marginSquared, marginUp)
			if err != nil || deep {
				return deep, err
			}
		}
	}
	return false, nil
}

// deepInside reports whether p lies strictly inside solid with every
// boundary triangle farther than sqrt(marginSquared) away. pb is p's outward
// float box and marginUp a float at or above marginSquared: a float lower
// bound on the box gap above marginUp skips the triangle as the exact box test
// would (planar_prune.go). Without the solid's float boxes every triangle
// takes the exact test.
func (k *planarKernel) deepInside(p proof.DyV3, pb floatBox, solid *planarPrep, marginSquared proof.Dyadic, marginUp float64) (bool, error) {
	boxed := solid.triBox != nil
	for t := range solid.s.Tris {
		if err := k.poll(); err != nil {
			return false, err
		}
		if boxed && gapSquaredBelowLimit(pb, solid.triBox[t], marginUp) > marginUp {
			continue
		}
		if proof.DyCmp(boxGapSquared(p, p, solid.triLo[t], solid.triHi[t]), marginSquared) > 0 {
			continue
		}
		d := pointTriangle(solid, t, p)
		if proof.DyCmp(d.num, proof.DyMul(marginSquared, d.den)) <= 0 {
			return false, nil
		}
	}
	result, err := k.cast(p, solid)
	return result == castInside, err
}

// pointTriangle is the exact squared distance from x to the closed triangle
// t: the plane distance when x projects inside it, else the nearest edge.
func pointTriangle(solid *planarPrep, t int, x proof.DyV3) frac {
	tri := solid.s.Tris[t]
	normal := solid.normal[t]
	inside := true
	for i := range 3 {
		if edgeSide(normal, solid.s.Verts[tri[i]], solid.s.Verts[tri[(i+1)%3]], dyPoint(x)) < 0 {
			inside = false
			break
		}
	}
	if inside {
		height := proof.DvDot(normal, proof.DvSub(x, solid.s.Verts[tri[0]]))
		return frac{num: proof.DyMul(height, height), den: proof.DvDot(normal, normal)}
	}
	best := pointSegment(x, solid.s.Verts[tri[0]], solid.s.Verts[tri[1]])
	for i := 1; i < 3; i++ {
		if d := pointSegment(x, solid.s.Verts[tri[i]], solid.s.Verts[tri[(i+1)%3]]); fracCmp(d, best) < 0 {
			best = d
		}
	}
	return best
}

// shallowTrials bounds the translations PlanarShallowOverlap tries, each a
// full ClassifyPlanar of the pair.
const shallowTrials = 3

// PlanarShallowOverlap reports whether a translation of a, no longer than
// margin, leaves the two audited solids touching or separated, when result is
// their Overlapping relation. Such a translation v bounds the overlap's depth:
// a point x inside both interiors has x + v outside b's interior, since a + v
// and b share no interior point, so the segment from x to x + v leaves b and
// x lies within |v| of b's boundary; and x − v lies outside a's interior for
// the same reason, so x lies within |v| of a's boundary too. A false result
// proves nothing.
//
// The tried directions are the outward normals of the crossing facets, b's
// as they are and a's reversed, most frequent first: the ways a backs out of
// the facets the overlap crosses. Floats only pick a direction; the offset is
// the exact lift of its float components, admitted when its exact length is at
// most margin. A trial that reaches ClassifyPlanar's work limit ends the
// search. margin must be nonnegative; poll is charged as ClassifyPlanar
// charges it.
func PlanarShallowOverlap(a, b *PlanarSolid, result PlanarResult, margin proof.Dyadic,
	poll func() error) (bool, error) {
	if result.Relation != pair.Overlapping || margin.Sign() <= 0 {
		return false, nil
	}
	length, _ := proof.FloatBounds(margin)
	// The float scale stays below margin by far more than the rounding of a
	// unit direction and its product with length.
	length *= 1 - 0x1p-20
	marginSquared := proof.DyMul(margin, margin)
	directions := shallowDirections(result.preparedFor(a), result.preparedFor(b), result.Crossings)
	for _, direction := range directions[:min(len(directions), shallowTrials)] {
		var offset proof.DyV3
		admitted := true
		for axis := range 3 {
			component, ok := proof.DyOf(direction[axis] * length)
			if !ok {
				admitted = false
				break
			}
			offset[axis] = component
		}
		if !admitted || proof.DvIsZero(offset) || proof.DyCmp(proof.DvDot(offset, offset), marginSquared) > 0 {
			continue
		}
		moved := translatedPlanar(a, offset)
		trial, err := classifyPlanar(&moved, b, PlanarHint{}, true, poll)
		if errors.Is(err, ErrWorkLimit) {
			return false, nil
		}
		if err != nil {
			return false, err
		}
		if trial.Relation == pair.Separated || trial.Relation == pair.Touching {
			return true, nil
		}
	}
	return false, nil
}

// shallowDirections returns the float unit directions that back a out of the
// crossing facets, most frequent first and in first-seen order on a tie: each
// facet of b a crossing names gives its outward normal, and each facet of a
// gives its inward one. Directions within float rounding of one another count
// as one.
func shallowDirections(pa, pb *planarPrep, crossings []PlanarCrossing) [][3]float64 {
	type tally struct {
		direction [3]float64
		count     int
		first     int
	}
	seen := make(map[[3]int64]*tally)
	var order []*tally
	add := func(normal proof.DyV3, sign float64) {
		var direction [3]float64
		for axis := range 3 {
			c := normal[axis]
			f, _ := c.Float64()
			direction[axis] = sign * f
		}
		norm := math.Hypot(math.Hypot(direction[0], direction[1]), direction[2])
		if norm == 0 || math.IsInf(norm, 0) || math.IsNaN(norm) {
			return
		}
		var key [3]int64
		for axis := range 3 {
			direction[axis] /= norm
			key[axis] = int64(math.Round(direction[axis] * 0x1p30))
		}
		if entry, ok := seen[key]; ok {
			entry.count++
			return
		}
		entry := &tally{direction: direction, count: 1, first: len(order)}
		seen[key] = entry
		order = append(order, entry)
	}
	for _, crossing := range crossings {
		for _, side := range [2]struct {
			part CrossingPart
			prep *planarPrep
			sign float64
		}{{crossing.A, pa, -1}, {crossing.B, pb, 1}} {
			add(side.prep.normal[side.part.Facets[0]], side.sign)
			if side.part.Facets[1] != side.part.Facets[0] {
				add(side.prep.normal[side.part.Facets[1]], side.sign)
			}
		}
	}
	slices.SortStableFunc(order, func(x, y *tally) int {
		if c := cmp.Compare(y.count, x.count); c != 0 {
			return c
		}
		return cmp.Compare(x.first, y.first)
	})
	out := make([][3]float64, len(order))
	for i, entry := range order {
		out[i] = entry.direction
	}
	return out
}

// translatedPlanar returns s moved by offset. A translation keeps every
// triangle normal and the signed volume, so the moved solid passes the audit s
// passed, and it reuses s's topology and normals when s carries them.
func translatedPlanar(s *PlanarSolid, offset proof.DyV3) PlanarSolid {
	moved := PlanarSolid{Verts: make([]proof.DyV3, len(s.Verts)), Tris: s.Tris, Faces: s.Faces}
	for i, v := range s.Verts {
		moved.Verts[i] = proof.DvAdd(v, offset)
	}
	if d := s.pose; d != nil && d.serves(s) {
		moved.pose = newPlanarPose(&moved, d.topo, d.normal)
	}
	return moved
}
