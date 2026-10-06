package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/tessellation"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file builds docs/sweep-design.md §16's mitred polyline sweep: the
// §16.3 construction over exact rationals, Table SM rows SM5 through SM8, the
// single rounding into the held vertex table, Table BM's topology, and
// §16.6's four readings, all taken over the rational vertices.
//
// Every section is the image of the one before it under its span's wall
// lines, and every wall line of one span passes through that span's apex (or
// is parallel to the span when its ratio is one). So every wall quad is
// exactly planar, and the map from one section to the next is a projective
// map on a convex region holding the whole section (SM6 states each of its
// conditions as a linear inequality over the vertex, so it holds on the hull
// once it holds at every vertex). That map carries a simple polygon with
// holes to a simple polygon with holes and carries a triangulation of one to
// a triangulation of the other, which is why capEnd reuses capStart's
// triangulation index for index rather than triangulating a second time.

// mitredSection is one loop-major section polygon: vertex v of loop i sits at
// index loopIdx[i][j].
type mitredSection []sweepRatVec

// mitredConstruction is §16.3's exact record before placement: every section
// polygon, the path points and the cap triangulation of P_0.
type mitredConstruction struct {
	sections []mitredSection
	loopIdx  [][]int
	capTris  [][3]int
	anchor   sweepRatVec
}

// mitredSpanError names the span, loop and vertex (or segment) a Table SM row
// refused on, so the caller knows which part of the path or section to
// repair (§16.4's SM6 note).
func mitredSpanError(sentinel error, span, loop, index int, what string) error {
	return fmt.Errorf(`%w: mitred sweep span %d, loop %d, %s`, sentinel, span, loop, fmt.Sprintf(what, index))
}

// constructMitredSweep runs §16.3 over the payload's record: it lifts the
// recorded profile, states every join plane, maps each section onto the next
// through its span's wall lines, and refuses on SM5, SM6 and SM7 per span in
// path order. Nothing is placed or rounded here.
func constructMitredSweep(ctx context.Context, mp mitredSweepPayload) (mitredConstruction, error) {
	pts2, loopIdx, err := mitredSweepLoops(mp.profile)
	if err != nil {
		return mitredConstruction{}, err
	}
	capTris, err := triangulate2DContext(ctx, pts2, loopIdx)
	if err != nil {
		return mitredConstruction{}, wrapLoftTriangulationError(err)
	}

	records := mp.path.records
	n := len(records)
	if len(mp.factors) != n {
		return mitredConstruction{}, fmt.Errorf(`%w: the mitred sweep records %d factors for %d spans`, ErrDegenerate, len(mp.factors), n)
	}
	points := make([]sweepRatVec, n+1)
	dirs := make([]sweepRatVec, n)
	lambdas := make([]*big.Rat, n)
	points[0] = sweepRatVecOf(records[0].start)
	for k, record := range records {
		points[k+1] = sweepRatVecOf(record.end)
		dirs[k] = sweepRatSub(points[k+1], points[k])
		lambda, err := mitredSpanLengthLower(record.start, record.end)
		if err != nil {
			return mitredConstruction{}, fmt.Errorf(`mitred sweep span %d: %w`, k, err)
		}
		lambdas[k] = lambda
	}

	// The section planes of the table in §16.3: Π_0's normal is the profile
	// plane's own, every join plane's is the length-weighted sum of its two
	// span directions, and Π_N's is the last span's direction.
	normals := make([]sweepRatVec, n+1)
	normals[0] = sweepRatFromDyadic(proofarith.DvCross(proofarith.DyVec(mp.plane.U), proofarith.DyVec(mp.plane.V)))
	normals[n] = dirs[n-1]

	section := make(mitredSection, len(pts2))
	for v, p := range pts2 {
		section[v] = mitredLift(mp.plane, p)
	}
	sections := []mitredSection{section}

	one := big.NewRat(1, 1)
	for k := range n {
		if err := ctx.Err(); err != nil {
			return mitredConstruction{}, err
		}
		if k+1 < n {
			// SM5: two exactly reversed consecutive spans have no join plane.
			if sweepRatIsZero(sweepRatCross(dirs[k], dirs[k+1])) && sweepRatDot(dirs[k], dirs[k+1]).Sign() < 0 {
				return mitredConstruction{}, fmt.Errorf(`%w: mitred sweep spans %d and %d run exactly back along each other and have no join plane`, ErrDegenerate, k, k+1)
			}
			normals[k+1] = sweepRatAdd(sweepRatScale(dirs[k], lambdas[k+1]), sweepRatScale(dirs[k+1], lambdas[k]))
		}
		// SM6's span-level arm: the span must leave its start plane forward
		// and reach its end plane forward. Every wall line's direction
		// differs from d_k by a vector in the start plane, so the first sign
		// is every wall line's own; the second puts the span's apex beyond
		// the end plane, which is what makes the per-vertex apex test below
		// the only way a wall line can meet that plane coming back.
		if sweepRatDot(normals[k], dirs[k]).Sign() <= 0 {
			return mitredConstruction{}, fmt.Errorf(`%w: mitred sweep span %d does not leave its start section plane forward`, ErrDegenerate, k)
		}
		if sweepRatDot(normals[k+1], dirs[k]).Sign() <= 0 {
			return mitredConstruction{}, fmt.Errorf(`%w: mitred sweep span %d does not reach its end section plane forward`, ErrDegenerate, k)
		}

		ratio := new(big.Rat).Quo(mitredFactor(mp.factors, k+1), mitredFactor(mp.factors, k))
		ratioLess := ratio.Cmp(one) < 0
		shrink := new(big.Rat).Sub(one, ratio)
		grow := new(big.Rat).Neg(shrink)
		end := normals[k+1]
		next := make(mitredSection, len(section))
		for i, idx := range loopIdx {
			for j, v := range idx {
				p := section[v]
				w := sweepRatAdd(dirs[k], sweepRatScale(sweepRatSub(p, points[k]), grow))
				nw := sweepRatDot(end, w)
				if nw.Sign() == 0 {
					return mitredConstruction{}, mitredSpanError(ErrDegenerate, k, i, j,
						"vertex %d: its wall line is parallel to the end section plane")
				}
				s := sweepRatDot(end, sweepRatSub(points[k+1], p))
				s.Quo(s, nw)
				if s.Sign() <= 0 {
					return mitredConstruction{}, mitredSpanError(ErrDegenerate, k, i, j,
						"vertex %d: its wall line meets the end section plane at or behind its start")
				}
				if ratioLess && new(big.Rat).Mul(s, shrink).Cmp(one) >= 0 {
					return mitredConstruction{}, mitredSpanError(ErrDegenerate, k, i, j,
						"vertex %d: its wall line meets the end section plane at or past the span's apex; lengthen the span, shrink the section or weaken the taper")
				}
				next[v] = sweepRatAdd(p, sweepRatScale(w, s))
			}
		}
		if err := mitredRequireWalls(k, loopIdx, section, next); err != nil {
			return mitredConstruction{}, err
		}
		section = next
		sections = append(sections, section)
	}
	return mitredConstruction{sections: sections, loopIdx: loopIdx, capTris: capTris, anchor: points[0]}, nil
}

// mitredSpanLengthLower is λ_j of §16.3: the lower endpoint of the certified
// enclosure of the span's length, read at float64 precision as the largest
// float whose square does not exceed the exact squared length
// (proofarith.DySqrtDown). The precision is fixed by that type, so the join
// plane never depends on a platform's sqrt or on FMA contraction.
func mitredSpanLengthLower(start, end r3.Vec) (*big.Rat, error) {
	d := proofarith.DvSub(proofarith.DyVec(end), proofarith.DyVec(start))
	lambda := proofarith.DySqrtDown(proofarith.DvDot(d, d))
	if !(lambda > 0) || math.IsInf(lambda, 0) {
		return nil, fmt.Errorf(`%w: the span's length has no positive float lower bound`, ErrUnsupported)
	}
	return proofarith.FloatRat(lambda), nil
}

// mitredFactor is f_k with f_0 = 1.
func mitredFactor(factors []float64, k int) *big.Rat {
	if k == 0 {
		return big.NewRat(1, 1)
	}
	return proofarith.FloatRat(factors[k-1])
}

// mitredLift is the recorded point (u, v) lifted to O + u·U + v·V over the
// plane record's floats, held as the exact rational that sum is.
func mitredLift(plane PlaneRecord, p Point2) sweepRatVec {
	u, v := proofarith.FloatRat(p.U), proofarith.FloatRat(p.V)
	return sweepRatAdd(sweepRatVecOf(plane.Origin), sweepRatScale(sweepRatVecOf(plane.U), u), sweepRatScale(sweepRatVecOf(plane.V), v))
}

// mitredRequireWalls is SM7 over one span: every wall quad p, q, q', p' has a
// nonzero exact area vector (q' − p) × (p' − q), and no two vertices of the
// span's end section coincide.
func mitredRequireWalls(k int, loopIdx [][]int, from, to mitredSection) error {
	for i, idx := range loopIdx {
		m := len(idx)
		for j := range m {
			p, q := from[idx[j]], from[idx[(j+1)%m]]
			pn, qn := to[idx[j]], to[idx[(j+1)%m]]
			if sweepRatIsZero(sweepRatCross(sweepRatSub(qn, p), sweepRatSub(pn, q))) {
				return mitredSpanError(ErrDegenerate, k, i, j, "segment %d: its wall quad has zero area")
			}
		}
	}
	seen := make(map[string]int, len(to))
	for v, p := range to {
		key := p[0].RatString() + "," + p[1].RatString() + "," + p[2].RatString()
		if _, dup := seen[key]; dup {
			return fmt.Errorf(`%w: mitred sweep span %d's end section has two coincident vertices`, ErrDegenerate, k)
		}
		seen[key] = v
	}
	return nil
}

// mitredPlace applies the accumulated placement to a rational point exactly:
// a Transform's basis and translation entries are floats, hence rationals.
func mitredPlace(xform r3.Transform, p sweepRatVec) sweepRatVec {
	if xform == r3.Identity() {
		return p
	}
	basis := xform.Basis()
	return sweepRatAdd(
		sweepRatScale(sweepRatVecOf(basis.EX), p[0]),
		sweepRatScale(sweepRatVecOf(basis.EY), p[1]),
		sweepRatScale(sweepRatVecOf(basis.EZ), p[2]),
		sweepRatVecOf(xform.Translation()),
	)
}

// mitredRound rounds one rational coordinate to its nearest float and
// returns the exact gap.
func mitredRound(r *big.Rat) (float64, *big.Rat, bool) {
	f, _ := r.Float64()
	if math.IsInf(f, 0) || math.IsNaN(f) {
		return 0, nil, false
	}
	gap := new(big.Rat).Sub(r, proofarith.FloatRat(f))
	return f, gap.Abs(gap), true
}

// evalMitredSweep builds the body for one payload record: §16.3's
// construction, the exact placement, the single rounding, the assembled
// triangle set and its orientation, SM8's crossing audit, Table BM's
// topology and §16.6's readings. The first build and every placement run it.
func evalMitredSweep(ctx context.Context, d *Document, ref producerID, mp mitredSweepPayload) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	c, err := constructMitredSweep(ctx, mp)
	if err != nil {
		return nil, err
	}

	stride := len(c.sections[0])
	exact := make([]sweepRatVec, 0, stride*len(c.sections))
	for _, section := range c.sections {
		for _, p := range section {
			exact = append(exact, mitredPlace(mp.xform, p))
		}
	}
	anchor := mitredPlace(mp.xform, c.anchor)

	// Rounding, once: every vertex to its nearest float per coordinate, with
	// that vertex's own 3D gap read exactly from its largest coordinate gap and
	// rounded up (docs/faceted-vertex-bounds-design.md §2.1), and delta the
	// largest of them.
	verts := make([]r3.Vec, len(exact))
	vertexBound := make([]float64, len(exact))
	delta := 0.0
	for v, p := range exact {
		var coords [3]float64
		worst := new(big.Rat)
		for axis := range 3 {
			f, gap, ok := mitredRound(p[axis])
			if !ok {
				return nil, fmt.Errorf(`%w: a mitred sweep vertex runs past the representable float64 range`, ErrUnsupported)
			}
			coords[axis] = f
			if gap.Cmp(worst) > 0 {
				worst = gap
			}
		}
		verts[v] = r3.NewVec(coords[0], coords[1], coords[2])
		if worst.Sign() != 0 {
			w, _ := worst.Float64()
			vertexBound[v] = proofbound.Radius3D(proofbound.ProvenUpRound(w))
			delta = max(delta, vertexBound[v])
		}
	}

	a := assembleMitredSweep(c, stride)
	vol6, moments := mitredVolumeMoments(exact, a.tris, anchor)
	switch vol6.Sign() {
	case 0:
		return nil, fmt.Errorf(`%w: the mitred sweep encloses no volume`, ErrDegenerate)
	case -1:
		// §16.3's orientation rule: one exact sign for the whole shell.
		a.reversed = true
		for t, tri := range a.tris {
			a.tris[t] = [3]int{tri[0], tri[2], tri[1]}
		}
		vol6.Neg(vol6)
		for axis := range moments {
			moments[axis].Neg(moments[axis])
		}
	}
	for t, tri := range a.tris {
		if tessellation.TriangleCollapsed(verts, tri) {
			return nil, fmt.Errorf(`%w: rounding the mitred sweep's vertices collapsed its triangle %d (%s)`, ErrUnsupported, t, a.roles[a.triFace[t]])
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := tessellation.LoftCrossingAudit(proofbound.NewWorkBudget(ctx), verts, a.tris); err != nil {
		var contact *tessellation.LoftContactError
		if errors.As(err, &contact) {
			return nil, fmt.Errorf(`%w: mitred sweep faces %s and %s %s`, ErrDegenerate,
				a.roles[a.triFace[contact.I]], a.roles[a.triFace[contact.J]], contact.Reason)
		}
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	areas, err := mitredTriangleAreas(ctx, exact, a.tris)
	if err != nil {
		return nil, err
	}
	body := &Body{doc: d, origin: FeatureRef{producer: ref, Role: roleBody}, solid: true, kind: BodySolid}
	faces, err := buildMitredTopology(ctx, body, ref, a, exact, verts, areas, delta)
	if err != nil {
		return nil, err
	}
	if err := attachFaceLoopsContext(ctx, faces); err != nil {
		return nil, err
	}
	body.lumps = []*Lump{{shells: []*Shell{{faces: faces}}}}

	body.volume = mitredVolume(vol6)
	if body.centroid, err = mitredCentroid(anchor, vol6, moments); err != nil {
		return nil, err
	}
	body.bounds = mitredBounds(exact)
	areaLo, areaHi := new(big.Rat), new(big.Rat)
	for _, enclosure := range areas {
		areaLo.Add(areaLo, enclosure[0])
		areaHi.Add(areaHi, enclosure[1])
	}
	value, bound := mitredEnclosure(areaLo, areaHi)
	body.area = Measurement{
		Value:     units.SquareMillimeters(value),
		Exactness: exactnessOf(bound),
		Bound:     units.SquareMillimeters(bound),
	}
	if err := validateLoftBodyMeasurements(body); err != nil {
		return nil, err
	}
	if !finiteMeasurementValues(bound) {
		return nil, fmt.Errorf(`%w: the mitred sweep's area has no finite bound`, ErrUnsupported)
	}

	mp.exact, mp.verts, mp.tris, mp.triFace, mp.faceRoles, mp.delta = exact, verts, a.tris, a.triFace, a.roles, delta
	mp.vertexBound = vertexBound
	body.payload = mp
	return body, nil
}
