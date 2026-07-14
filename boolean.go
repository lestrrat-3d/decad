package decad

import (
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
// documents are ErrForeignBody; retired operands ErrRetiredBody; a tangent
// contact the exact predicates cannot classify (face-on-face or grazing
// contact) is ErrDegenerate; a result with no volume is ErrBooleanFailed.
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

// performBoolean is the shared pipeline: gate the operands, tessellate at
// the pair-derived tolerance, run the exact mesh boolean, build the Faceted
// body, and commit the step atomically (a failure leaves recipe and document
// untouched, and the operands live).
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

	tolMM, dPair, err := pairChordTolerance(a, b)
	if err != nil {
		return nil, err
	}
	ma, err := a.Tessellate(units.Millimeters(tolMM))
	if err != nil {
		return nil, err
	}
	mb, err := b.Tessellate(units.Millimeters(tolMM))
	if err != nil {
		return nil, err
	}

	// Global source-face ids: the operands' faces in order, so the payload
	// records provenance without holding live pointers.
	var groups []facetGroup
	faceID := map[*Face]int{}
	for _, f := range append(a.Faces(), b.Faces()...) {
		faceID[f] = len(groups)
		groups = append(groups, facetGroup{
			origins: f.Origins(),
			planar:  f.Surface().Kind() == KindPlane,
		})
	}
	srcA, err := sourceIDs(ma, faceID)
	if err != nil {
		return nil, err
	}
	srcB, err := sourceIDs(mb, faceID)
	if err != nil {
		return nil, err
	}

	bmA, err := prepBoolMesh(ma, srcA)
	if err != nil {
		return nil, err
	}
	bmB, err := prepBoolMesh(mb, srcB)
	if err != nil {
		return nil, err
	}
	kept, err := meshBoolean(op, bmA, bmB)
	if err != nil {
		return nil, err
	}
	if len(kept) == 0 {
		return nil, fmt.Errorf(`%w: the %s result is empty`, ErrBooleanFailed, op)
	}
	stitched, err := stitchFacets(kept)
	if err != nil {
		return nil, err
	}

	// Bound composition (§9, the verification-design shapes). The volume
	// error obeys the symmetric-difference identity |1_{A∘B} − 1_{A'∘B'}| ≤
	// |1_A − 1_A'| + |1_B − 1_B'| for all three ops, so it is the sum of the
	// operands' own symmetric-difference bounds — each δ · (that operand's
	// held area) — plus what the final float rounding can move.
	symA := operandSymDiff(a, ma)
	symB := operandSymDiff(b, mb)
	roundArea := stitched.round * meshAreaUpper(stitched.verts, stitched.tris)
	payload := facetedPayload{
		verts:      stitched.verts,
		tris:       stitched.tris,
		src:        stitched.src,
		groups:     groups,
		meshBound:  ma.bound + mb.bound + stitched.round,
		volSymDiff: symA + symB + roundArea,
		areaSlack:  ma.areaSlack + mb.areaSlack,
		dPair:      dPair,
		xform:      r3.Identity(),
	}

	step := Step{
		Op:     op,
		Inputs: []StepRef{a.originStep(), b.originStep()},
	}
	ref := d.nextStepRef()
	body, err := buildFacetedBody(d, ref, payload)
	if err != nil {
		return nil, err
	}
	d.commit(step, body, a, b)
	return body, nil
}

// sourceIDs maps a tessellation's per-facet source faces to the global
// group ids.
func sourceIDs(m *Mesh, faceID map[*Face]int) ([]int, error) {
	out := make([]int, len(m.source))
	for i, f := range m.source {
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
