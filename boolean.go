package decad

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the public boolean surface of core §8 over the exact-predicate
// mesh boolean of docs/evaluator-design.md §9: Union, Cut and Intersect
// tessellate both operands at an evaluator-internal chord tolerance derived
// from the pair's own diameter, intersect the meshes with adaptive-exact
// predicates, and produce a Faceted body whose measurements carry proven
// composed bounds. The signatures follow core §3 invariant #4: no operand is
// mutated, no target-out parameter exists — each call retires its operands
// from their document and registers the result, reached through the operands'
// own owning document.

// boolChordFactor derives the evaluator-internal chord tolerance from the
// operand pair's diameter (§9: the booleans expose no tolerance parameter —
// the tolerance's whole effect surfaces as the result's proven Bound). At
// diameter × 2e-5 the §2 worked example — a Ø20×10 mm cylinder — carries a
// volume bound several times inside the default 1e-3 relative gate, while the
// chord counts stay far under the tessellator's cap.
const boolChordFactor = 2e-5

// Union returns the body enclosing the volume of a or b, retiring both
// operands from their document (core §8). The result is a Faceted body: its
// faces are grouped by the operands' source faces, so provenance
// (FaceCreatedBy) survives, and its measurements are Approximate with proven
// bounds (docs/evaluator-design.md §9) — except that an all-planar pair whose
// contact points round exactly keeps an Exact VOLUME (the volume integral is
// computed in exact arithmetic); the surface area always carries at least an
// ulp-scale float-summation bound, so it reads Approximate with a bound tiny
// against any real tolerance. Operands from different
// documents are ErrForeignBody; retired operands ErrRetiredBody. A
// curved-surface tangent contact whose chord facets never meet is
// ErrUnsupported — whether the true surfaces touch would be decided by
// where the chords happened to fall. An exact coplanar / face-on-face or
// isolated-point contact whose facets meet degenerately (a face-on-face
// overlap, a grazing edge) is ErrDegenerate. A result with no volume is
// ErrBooleanFailed.
// Both operands are tessellated first, so a revolve operand — which has no
// tessellator — is ErrUnsupported.
func Union(a, b *Body) (*Body, error) {
	return performBoolean(OpUnion, a, b)
}

// Cut returns target minus tool, retiring both operands from their document
// (core §8). The recorded step's Inputs order is [target, tool] — the two
// roles are asymmetric. A cut that removes everything is ErrBooleanFailed;
// the other gates match Union's.
func Cut(target, tool *Body) (*Body, error) {
	return performBoolean(OpCut, target, tool)
}

// Intersect returns the volume common to a and b, retiring both operands
// from their document (core §8). Disjoint operands share nothing, so the
// empty result is ErrBooleanFailed; the other gates match Union's.
func Intersect(a, b *Body) (*Body, error) {
	return performBoolean(OpIntersect, a, b)
}

type booleanExpectedKind int

const (
	booleanExpectedEmpty booleanExpectedKind = iota
	booleanExpectedContact
	booleanExpectedUnsupported
	booleanExpectedCoarseTessellation
)

// booleanExpectedError identifies an ordinary geometric non-result for the
// read-only evaluator. Public booleans still expose the wrapped sentinel;
// Verify recognizes the private type and leaves the pair undecided. Invariant
// failures remain ordinary errors and return from Verify.
type booleanExpectedError struct {
	kind booleanExpectedKind
	err  error
}

func (e *booleanExpectedError) Error() string { return e.err.Error() }
func (e *booleanExpectedError) Unwrap() error { return e.err }

func expectedBoolean(kind booleanExpectedKind, err error) error {
	var expected *booleanExpectedError
	if errors.As(err, &expected) {
		return err
	}
	return &booleanExpectedError{kind: kind, err: err}
}

func asExpectedBoolean(err error) (*booleanExpectedError, bool) {
	var expected *booleanExpectedError
	return expected, errors.As(err, &expected)
}

// booleanEvaluation is the geometry-only result shared by public booleans and
// interference verification. It contains no document reference or recipe
// state and cannot make the transient result live.
type booleanEvaluation struct {
	payload facetedPayload
	volume  Measurement
}

// performBoolean gates the operands, runs the read-only geometry evaluator,
// then builds and commits the public result atomically. A failure before the
// commit leaves the recipe, live-body set, and operands unchanged.
func performBoolean(op OpKind, a, b *Body) (*Body, error) {
	if a == nil || a.doc == nil {
		return nil, fmt.Errorf(`%w: the first operand belongs to no document`, ErrDegenerate)
	}
	d := a.doc
	if err := d.requireLive(a); err != nil {
		return nil, err
	}
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if a == b {
		return nil, fmt.Errorf(`%w: a boolean needs two distinct bodies`, ErrDegenerate)
	}
	eval, err := evaluateBoolean(context.Background(), op, a, b)
	if err != nil {
		return nil, err
	}

	step := Step{
		Op:     op,
		Inputs: []StepRef{a.originStep(), b.originStep()},
	}
	ref := d.nextStepRef()
	body, err := buildFacetedBody(d, ref, eval.payload)
	if err != nil {
		return nil, err
	}
	d.commit(step, body, a, b)
	return body, nil
}

// evaluateBoolean runs the complete geometry pipeline without writing the
// document. It deliberately does not gate liveness or mint a StepRef: the
// public wrapper owns those actions, while Verify already walks live bodies.
func evaluateBoolean(ctx context.Context, op OpKind, a, b *Body) (booleanEvaluation, error) {
	if err := ctx.Err(); err != nil {
		return booleanEvaluation{}, err
	}

	tolMM, dPair, err := pairChordTolerance(a, b)
	if err != nil {
		return booleanEvaluation{}, err
	}
	if err := ctx.Err(); err != nil {
		return booleanEvaluation{}, err
	}
	ma, err := tessellateContext(ctx, a, units.Millimeters(tolMM))
	if err != nil {
		var coarse *tessellationExpectedError
		if errors.As(err, &coarse) {
			err = expectedBoolean(booleanExpectedCoarseTessellation, err)
		} else if errors.Is(err, ErrUnsupported) {
			err = expectedBoolean(booleanExpectedUnsupported, err)
		}
		return booleanEvaluation{}, err
	}
	if err := ctx.Err(); err != nil {
		return booleanEvaluation{}, err
	}
	mb, err := tessellateContext(ctx, b, units.Millimeters(tolMM))
	if err != nil {
		var coarse *tessellationExpectedError
		if errors.As(err, &coarse) {
			err = expectedBoolean(booleanExpectedCoarseTessellation, err)
		} else if errors.Is(err, ErrUnsupported) {
			err = expectedBoolean(booleanExpectedUnsupported, err)
		}
		return booleanEvaluation{}, err
	}
	if err := ctx.Err(); err != nil {
		return booleanEvaluation{}, err
	}

	// Global source-face ids: the operands' faces in order, so the payload
	// records provenance without holding live pointers.
	var groups []facetGroup
	faceID := map[*Face]int{}
	for i, f := range append(a.Faces(), b.Faces()...) {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return booleanEvaluation{}, err
			}
		}
		faceID[f] = len(groups)
		groups = append(groups, facetGroup{
			origins: f.Origins(),
			planar:  f.isPlanar(),
		})
	}
	srcA, err := sourceIDs(ctx, ma, faceID)
	if err != nil {
		return booleanEvaluation{}, err
	}
	srcB, err := sourceIDs(ctx, mb, faceID)
	if err != nil {
		return booleanEvaluation{}, err
	}
	if err := ctx.Err(); err != nil {
		return booleanEvaluation{}, err
	}

	bmA, err := prepBoolMeshContext(ctx, ma, srcA)
	if err != nil {
		if errors.Is(err, ErrUnsupported) {
			err = expectedBoolean(booleanExpectedUnsupported, err)
		}
		return booleanEvaluation{}, err
	}
	bmB, err := prepBoolMeshContext(ctx, mb, srcB)
	if err != nil {
		if errors.Is(err, ErrUnsupported) {
			err = expectedBoolean(booleanExpectedUnsupported, err)
		}
		return booleanEvaluation{}, err
	}
	if err := ctx.Err(); err != nil {
		return booleanEvaluation{}, err
	}
	if err := refuseUndecidableProximity(ctx, ma, mb, bmA, bmB); err != nil {
		if errors.Is(err, ErrUnsupported) {
			err = expectedBoolean(booleanExpectedUnsupported, err)
		}
		return booleanEvaluation{}, err
	}
	if err := ctx.Err(); err != nil {
		return booleanEvaluation{}, err
	}
	kept, sinMin, err := meshBoolean(ctx, op, bmA, bmB)
	if err != nil {
		return booleanEvaluation{}, err
	}
	if len(kept) == 0 {
		return booleanEvaluation{}, expectedBoolean(booleanExpectedEmpty,
			fmt.Errorf(`%w: the %s result is empty`, ErrBooleanFailed, op))
	}
	if err := ctx.Err(); err != nil {
		return booleanEvaluation{}, err
	}
	stitched, err := stitchFacetsContext(ctx, kept)
	if err != nil {
		if errors.Is(err, ErrUnsupported) {
			err = expectedBoolean(booleanExpectedUnsupported, err)
		}
		return booleanEvaluation{}, err
	}
	if err := ctx.Err(); err != nil {
		return booleanEvaluation{}, err
	}

	// Bound composition (§9, the verification-design shapes; the helpers own
	// every mechanism — bounds.go). The volume error obeys the
	// symmetric-difference identity |1_{A∘B} − 1_{A'∘B'}| ≤ |1_A − 1_A'| +
	// |1_B − 1_B'| for all three ops, so it is the sum of the operands' own
	// symmetric-difference bounds — each δ · (that operand's held area) — plus
	// what the final float rounding can move. The BOUNDARY bound is a different
	// question: a vertex the boolean itself creates sits at the crossing of two
	// chord PLANES, so it is displaced by the trim-amplified (δA + δB)/sin θ,
	// not by δ.
	rim, err := rimDelta(ma.bound, mb.bound, sinMin, dPair)
	if err != nil {
		if errors.Is(err, ErrUnsupported) {
			err = expectedBoolean(booleanExpectedUnsupported, err)
		}
		return booleanEvaluation{}, err
	}
	symA := operandSymDiff(a, ma)
	symB := operandSymDiff(b, mb)
	// The final rounding's own volume error is what its vertex displacement
	// sweeps out over the surface it acted on — the stitched surface BEFORE the
	// weld dropped any collapsed facet from it (bounds.go, sweptVolumeAllow).
	// The held mesh's area is the WRONG yardstick here: a dropped facet is
	// missing from it, and its swept volume would go uncharged. The area the
	// weld dropped is likewise missing from every area the result reports, so
	// it joins the operands' own chord deficit in areaSlack.
	roundVol := sweptVolumeAllow(stitched.round, stitched.preArea)
	payload := facetedPayload{
		verts:      stitched.verts,
		tris:       stitched.tris,
		src:        stitched.src,
		groups:     groups,
		meshBound:  rim + stitched.round,
		volSymDiff: symA + symB + roundVol,
		areaSlack:  ma.areaSlack + mb.areaSlack + stitched.dropArea,
		dPair:      dPair,
		xform:      r3.Identity(),
	}
	if err := ctx.Err(); err != nil {
		return booleanEvaluation{}, err
	}
	if _, err := auditFacetedMesh(ctx, payload.verts, payload.tris); err != nil {
		return booleanEvaluation{}, err
	}
	volume, _, err := meshVolumeMeasurement(ctx, payload.verts, payload.tris, payload.volSymDiff)
	if err != nil {
		return booleanEvaluation{}, err
	}
	if err := ctx.Err(); err != nil {
		return booleanEvaluation{}, err
	}
	return booleanEvaluation{payload: payload, volume: volume}, nil
}

// sourceIDs maps a tessellation's per-facet source faces to the global
// group ids.
func sourceIDs(ctx context.Context, m *Mesh, faceID map[*Face]int) ([]int, error) {
	out := make([]int, len(m.source))
	for i, f := range m.source {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		id, ok := faceID[f]
		if !ok {
			return nil, fmt.Errorf(`%w: a facet's source is not an operand face`, ErrBooleanFailed)
		}
		out[i] = id
	}
	return out, nil
}

// operandSymDiff bounds the volume of the symmetric difference between the
// operand's held tessellation and the body it stands for. A tessellated
// analytic body deviates within each facet's own chord sliver, so δ times
// the held area bounds it; a Faceted operand carries its own composed bound.
func operandSymDiff(b *Body, m *Mesh) float64 {
	if fp, ok := b.payload.(facetedPayload); ok {
		return fp.volSymDiff
	}
	return m.bound * meshAreaUpper(m.vertices, m.triangles)
}

// refuseUndecidableProximity is the tangency gate. The mesh boolean decides
// every contact on the CHORDS, and a chord polygon lies strictly inside the
// curved surface it approximates — so a true tangency between two operands can
// vanish from the tessellation entirely, and the boolean would return a clean
// two-lump body for two solids that genuinely touch. Worse, whether it vanishes
// at all depends on where the chord samples happen to fall.
//
// The gate is reject-only, and it proves nothing it does not have. If the true
// surfaces of two faces touch, that point lies within δA of face A's facets and
// within δB of face B's, so the two facet sets come within δA + δB of each
// other. Contrapose it: a face pair whose facets stay FARTHER apart than
// δA + δB cannot hide a touch. That leaves two cases — a face pair whose facets
// already MEET, where the contact is real and the exact predicates own it, and
// a face pair that comes within δA + δB without meeting, which is exactly the
// undecidable one. The latter is refused (ErrUnsupported). Deciding it for real
// is the analytic clearance kernel's job (docs/clearance-design.md), not a
// chord's; loud beats silently wrong.
//
// The question is asked per analytic FACE pair, not per facet pair: a facet of
// one face can pass arbitrarily close to a facet of the other while the FACES
// plainly cross (a cylinder wall threading a cap's own triangulation diagonal),
// and that is no tangency at all.
//
// It may refuse a valid model whose operands genuinely pass within a chord
// tolerance of each other. That is the accepted price; the alternative is a
// verdict decided by chord placement.
func refuseUndecidableProximity(ctx context.Context, ma, mb *Mesh, bmA, bmB *boolMesh) error {
	// One counter spans the grouping and the pair scan it feeds: the grouping
	// walks every facet and every new source face's edges, which is work the
	// §7.2 interval covers just as the pair scan is.
	budget := newWorkBudget(ctx)
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
			if err := budget.step(); err != nil {
				return err
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
			near, err := facesNearMiss(ctx, bmA, ga.facets, bmB, gb.facets, slack*(1+1e-9))
			if err != nil {
				return err
			}
			if near {
				return fmt.Errorf(`%w: the operands' true surfaces come within the chord tolerance without their tessellations meeting, so whether they touch is decided by where the chords fall — this evaluator refuses the question rather than answer it wrong`, ErrUnsupported)
			}
		}
	}
	return nil
}

// facesNearMiss reports whether two faces' facet sets come within slack of each
// other WITHOUT meeting. A pair that meets is decided (the contact is exact);
// only a pair that comes close and misses is the undecidable one.
func facesNearMiss(ctx context.Context, bmA *boolMesh, fis []int, bmB *boolMesh, fjs []int, slack float64) (bool, error) {
	near := false
	work := 0
	for _, i := range fis {
		for _, j := range fjs {
			work++
			if work%256 == 0 {
				if err := ctx.Err(); err != nil {
					return false, err
				}
			}
			if !boxesWithin(bmA.boxes[i], bmB.boxes[j], slack) {
				continue
			}
			ta := triCorners(bmA, i)
			tb := triCorners(bmB, j)
			c, err := triTriClassify(ta, tb, xtriCorners(bmA, i), xtriCorners(bmB, j), bmA.norms[i], bmB.norms[j])
			if err != nil {
				return false, err
			}
			if c.kind != contactNone {
				return false, nil // the faces meet: nothing here is undecided
			}
			if triTriDistance(ta, tb) <= slack {
				near = true
			}
		}
	}
	return near, nil
}

// faceFacets is one analytic face of an operand: its facets, and the chord
// displacement between them and the true face.
type faceFacets struct {
	delta  float64
	facets []int
}

// facesOfMesh groups a tessellation's facets by their source face, in first-
// appearance order, and charges each face its own chord displacement: how far
// the TRUE face may lie from the facets that stand for it. A planar face with
// straight edges triangulates exactly, and a Faceted face IS its polygons (core
// §6.1) — both are held with zero error. Anything else — a cylinder wall, or a
// planar cap whose rim is a circle the chords inscribe — deviates by up to the
// mesh's own proven bound.
func facesOfMesh(budget *workBudget, m *Mesh) ([]faceFacets, error) {
	index := map[*Face]int{}
	var out []faceFacets
	for i, f := range m.source {
		if err := budget.step(); err != nil {
			return nil, err
		}
		k, ok := index[f]
		if !ok {
			k = len(out)
			index[f] = k
			delta, err := faceChordDelta(budget, f, m.bound)
			if err != nil {
				return nil, err
			}
			out = append(out, faceFacets{delta: delta})
		}
		out[k].facets = append(out[k].facets, i)
	}
	return out, nil
}

func faceChordDelta(budget *workBudget, f *Face, meshBound float64) (float64, error) {
	switch f.Surface().Kind() {
	case KindFaceted:
		return 0, nil
	case KindPlane:
		for _, e := range f.Edges() {
			if err := budget.step(); err != nil {
				return 0, err
			}
			if _, ok := e.Curve().(Line3); !ok {
				return meshBound, nil
			}
		}
		return 0, nil
	}
	return meshBound, nil
}

// boxesWithin reports whether the two boxes come within slack of each other.
func boxesWithin(a, b [2]r3.Vec, slack float64) bool {
	return a[0].X-slack <= b[1].X && b[0].X-slack <= a[1].X &&
		a[0].Y-slack <= b[1].Y && b[0].Y-slack <= a[1].Y &&
		a[0].Z-slack <= b[1].Z && b[0].Z-slack <= a[1].Z
}

// triTriDistance is the distance between two DISJOINT triangles: the smallest
// of the nine edge-edge distances and the six vertex-to-triangle distances,
// which is where the minimum of two disjoint convex sets is always attained.
// The result is nudged DOWN so its own float rounding can only widen the
// refusal, never narrow it.
func triTriDistance(ta, tb [3]r3.Vec) float64 {
	best := math.Inf(1)
	for i := range 3 {
		for j := range 3 {
			best = math.Min(best, segSegDist3(ta[i], ta[(i+1)%3], tb[j], tb[(j+1)%3]))
		}
	}
	for i := range 3 {
		best = math.Min(best, pointTriDistance(ta[i], tb))
		best = math.Min(best, pointTriDistance(tb[i], ta))
	}
	if best <= 0 || isNonFinite(best) {
		return 0
	}
	return best * (1 - 1e-12)
}

// segSegDist3 is the distance between two closed 3D segments.
func segSegDist3(p0, p1, q0, q1 r3.Vec) float64 {
	u := p1.Sub(p0)
	v := q1.Sub(q0)
	w := p0.Sub(q0)
	a, b, c := u.Dot(u), u.Dot(v), v.Dot(v)
	d, e := u.Dot(w), v.Dot(w)
	den := a*c - b*b
	var s, t float64
	if den <= 0 {
		// Parallel (or a collapsed segment): pin s and solve for t.
		s, t = 0, clamp01(e, c)
	} else {
		s = clamp01(b*e-c*d, den)
		t = clamp01(a*e-b*d, den)
	}
	// Re-clamp against the other segment's ends, the standard two-pass fix.
	if tn := e + s*b; c > 0 {
		t = clamp01(tn, c)
	}
	if sn := -d + t*b; a > 0 {
		s = clamp01(sn, a)
	}
	return p0.Add(u.Scale(s)).Sub(q0.Add(v.Scale(t))).Len()
}

func clamp01(num, den float64) float64 {
	if den <= 0 {
		return 0
	}
	return math.Max(0, math.Min(1, num/den))
}

// pointTriDistance is the distance from a point to a closed triangle: the
// perpendicular foot when it lands inside, otherwise the nearest edge.
func pointTriDistance(p r3.Vec, t [3]r3.Vec) float64 {
	n := t[1].Sub(t[0]).Cross(t[2].Sub(t[0]))
	if nn := n.Dot(n); nn > 0 {
		h := n.Dot(p.Sub(t[0])) / nn
		foot := p.Sub(n.Scale(h))
		if pointInTriangle(foot, t, n) {
			return p.Sub(foot).Len()
		}
	}
	best := math.Inf(1)
	for i := range 3 {
		best = math.Min(best, segSegDist3(p, p, t[i], t[(i+1)%3]))
	}
	return best
}

// pointInTriangle reports whether a point already on the triangle's plane lies
// inside it, by the three edge cross products against the facet normal.
func pointInTriangle(p r3.Vec, t [3]r3.Vec, n r3.Vec) bool {
	for i := range 3 {
		if t[(i+1)%3].Sub(t[i]).Cross(p.Sub(t[i])).Dot(n) < 0 {
			return false
		}
	}
	return true
}

// pairChordTolerance derives the evaluator-internal chord tolerance and the
// pair diameter from the operands' own bounds boxes (inflated by their
// proven bounds). All-planar operands chord nothing regardless, so the
// tolerance only shapes curved boundaries.
func pairChordTolerance(a, b *Body) (float64, float64, error) {
	boxA, err := a.Bounds()
	if err != nil {
		return 0, 0, err
	}
	boxB, err := b.Bounds()
	if err != nil {
		return 0, 0, err
	}
	infA := boxA.Bound.Base()
	infB := boxB.Bound.Base()
	lo := r3.Vec{
		X: math.Min(boxA.Min.X-infA, boxB.Min.X-infB),
		Y: math.Min(boxA.Min.Y-infA, boxB.Min.Y-infB),
		Z: math.Min(boxA.Min.Z-infA, boxB.Min.Z-infB),
	}
	hi := r3.Vec{
		X: math.Max(boxA.Max.X+infA, boxB.Max.X+infB),
		Y: math.Max(boxA.Max.Y+infA, boxB.Max.Y+infB),
		Z: math.Max(boxA.Max.Z+infA, boxB.Max.Z+infB),
	}
	diag := hi.Sub(lo).Len()
	if diag <= 0 || isNonFinite(diag) {
		return 0, 0, fmt.Errorf(`%w: the operand pair has no extent to derive a chord tolerance from`, ErrDegenerate)
	}
	return diag * boolChordFactor, diag, nil
}
