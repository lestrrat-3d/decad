package booleaneval

import (
	"context"
	"errors"
	"math"

	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/gaptol"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// This file is the shifted retry of agent-docs/gap-tolerance-design.md §5.2: a
// mesh Boolean whose operands meet in a contact the exact predicates cannot
// classify is tried again with the second operand moved by up to the document's
// gap tolerance, which turns a face-on-face or edge-on-face contact into a
// crossing or a gap. The moved operand is an approximation of the true one that
// is wider by the shift, so the ordinary composition of operand bounds charges
// the shift, trim amplification included.

// errShiftDeclined reports that a shifted attempt was not made or could not
// stand: the shifted mesh folds, or the pair uses a path this retry does not
// touch. The caller keeps the refusal that started the retry.
var errShiftDeclined = errors.New("booleaneval: shifted retry declined")

// shiftSteps are the fractions of the reach a shifted retry tries, smallest
// first, so the result departs from the true Boolean by as little as it can.
var shiftSteps = [...]float64{1.0 / 16, 1.0 / 4, 1}

// shiftWeights are the relative sizes of the generic part of a shift. They are
// unequal and irrational-looking so the shift is parallel to none of the planes
// a modeller commonly meets, which a shift along one axis would leave coplanar
// with the walls of two bodies that share a footprint.
var shiftWeights = [...]r3.Vec{
	{X: 1, Y: 0.6180339887, Z: 0.3819660113},
	{X: 0.3819660113, Y: 1, Z: 0.6180339887},
	{X: 0.6180339887, Y: 0.3819660113, Z: 1},
}

const (
	// shiftGeneric is the weight of the generic part against the unit direction
	// the contacts ask for.
	shiftGeneric = 0.2
	// shiftSignMargin and shiftOpenMargin are the least component of a unit
	// shift along a contact's normal in the wanted sense, and in either sense:
	// a shift closer to parallel to a contact's plane opens it too little to be
	// told from the plane's own rounding.
	shiftSignMargin = 0.05
	shiftOpenMargin = 0.02
)

// shiftDirections returns the unit directions to try for the contacts that
// refused the Boolean, or none when no shift opens them the way the operation
// needs. Every contact must open, whichever way it opens. The way matters where
// it changes the result. A Union wants its operands to overlap, so the result is
// one body, and a Decisive contact (a face-on-face touch, an edge grazing along
// a facet, an isolated point touch) must open inward along the first operand's
// outward normal. Cut and Intersect want the operands apart, so a Decisive
// contact must open outward along it: a touching Intersect is empty and a
// touching Cut leaves its target whole. When no contact is Decisive, the
// operands are flush on one side, and Cut and Intersect still need every
// contact to open outward, so a tool flush with its target's wall pokes through
// it instead of leaving a skin. Union wants nothing more of a flush contact
// than that it open.
func shiftDirections(op meshbool.OperationKind, contacts []meshbool.TouchContact) []r3.Vec {
	sigma := 1.0
	if op == meshbool.OpUnion {
		sigma = -1
	}
	wanted := wantedContacts(op, contacts)
	var sum r3.Vec
	for _, c := range wanted {
		sum = sum.Add(c.NormalA)
	}
	base, hasBase := sum.Scale(sigma).Normalize()
	var out []r3.Vec
	for _, w := range shiftWeights {
		u := w
		if hasBase {
			u = base.Add(w.Scale(shiftGeneric))
		}
		u, ok := u.Normalize()
		if !ok {
			continue
		}
		if shiftOpens(u, sigma, wanted, contacts) {
			out = append(out, u)
		}
	}
	return out
}

// wantedContacts are the contacts whose opening direction matters to op: the
// face-on-face touches when there are any, and otherwise every contact for Cut
// and Intersect and none for Union.
func wantedContacts(op meshbool.OperationKind, contacts []meshbool.TouchContact) []meshbool.TouchContact {
	var touches []meshbool.TouchContact
	for _, c := range contacts {
		if c.Decisive {
			touches = append(touches, c)
		}
	}
	switch {
	case len(touches) > 0:
		return touches
	case op == meshbool.OpUnion:
		return nil
	default:
		return contacts
	}
}

// shiftOpens reports whether moving the second operand along u opens every
// contact by at least shiftOpenMargin, and every wanted contact in the sense
// sigma names.
func shiftOpens(u r3.Vec, sigma float64, wanted, contacts []meshbool.TouchContact) bool {
	for _, c := range contacts {
		if math.Abs(u.Dot(c.NormalA)) < shiftOpenMargin {
			return false
		}
	}
	for _, c := range wanted {
		if sigma*u.Dot(c.NormalA) < shiftSignMargin {
			return false
		}
	}
	return true
}

// shiftOperandMesh returns two copies of m moved by shift. The decision copy
// keeps m's bounds: the moved operand is the solid the Boolean decides about,
// so the contact gate reads it exactly as it reads an unmoved one. The charged
// copy describes the moved mesh as an approximation of the true operand, which
// the composition of the result's bounds reads: every vertex bound, face bound
// and the mesh bound grow by the displacement, the symmetric-difference volume
// grows by what the move sweeps over the operand's boundary, and the area
// allowance grows by the rounding of the new coordinates. m is not modified.
// It returns errShiftDeclined when the moved mesh folds through itself.
func shiftOperandMesh(ctx context.Context, m *topo.Mesh, shift r3.Vec) (decision, charged *topo.Mesh, err error) {
	verts := make([]r3.Vec, len(m.Vertices))
	maxAbs := 0.0
	for i, v := range m.Vertices {
		verts[i] = v.Add(shift)
		maxAbs = math.Max(maxAbs, math.Max(math.Abs(verts[i].X), math.Max(math.Abs(verts[i].Y), math.Abs(verts[i].Z))))
	}
	if !proofbound.FiniteVec(shift) {
		return nil, nil, errShiftDeclined
	}
	// Each moved coordinate is the float sum of an exact coordinate and a
	// shift component, so it is within an ulp of the exact sum.
	round := 2 * proofbound.UlpOf(maxAbs)
	perCoord := proofbound.AbsSumUpper(math.Max(math.Abs(shift.X), math.Max(math.Abs(shift.Y), math.Abs(shift.Z))), round)
	d := proofbound.Radius3D(perCoord)
	if proofbound.IsNonFinite(d) {
		return nil, nil, errShiftDeclined
	}

	h := meshbool.HeldRounding{
		Verts: verts, Tris: m.Triangles,
		Exact: make([]proof.Xpt, len(verts)), Moved: make([]bool, len(verts)), Movable: make([]bool, len(verts)),
	}
	for i, v := range verts {
		h.Exact[i] = proof.XptOf(v)
		h.Moved[i] = true
	}
	folds, err := meshbool.HeldFolds(ctx, h)
	if err != nil {
		return nil, nil, err
	}
	if len(folds) > 0 {
		return nil, nil, errShiftDeclined
	}

	beta, err := topobuild.VertexBounds(m)
	if err != nil {
		return nil, nil, err
	}
	for i := range beta {
		beta[i] = proofbound.AbsSumUpper(beta[i], d)
	}
	faceBound := make(map[*topo.Face]float64, len(m.FaceBound))
	for f, b := range m.FaceBound {
		faceBound[f] = proofbound.AbsSumUpper(b, d)
	}
	swept, err := proofbound.PerturbedAreaUpperContext(ctx, verts, m.Triangles, d)
	if err != nil {
		return nil, nil, err
	}
	perimeter := 0.0
	for _, t := range m.Triangles {
		a, b, c := verts[t[0]], verts[t[1]], verts[t[2]]
		perimeter += b.Sub(a).Len() + c.Sub(b).Len() + a.Sub(c).Len()
	}
	perimeter = proofbound.UpRound(perimeter + proofbound.SumSlop(3*len(m.Triangles), perimeter))
	areaRound := proofbound.ProductUpper(proofbound.Radius3D(round), perimeter)

	moved := *m
	moved.Vertices = verts
	moved.Owner = nil

	next := moved
	next.Bound = proofbound.AbsSumUpper(m.Bound, d)
	next.CoordBound = proofbound.AbsSumUpper(m.CoordBound, d)
	next.VertexBound = beta
	next.FaceBound = faceBound
	next.AreaSlack = proofbound.AbsSumUpper(m.AreaSlack, areaRound)
	next.VolSymDiff = proofbound.AbsSumUpper(m.VolSymDiff, proofbound.SweptVolumeAllow(d, swept))
	if proofbound.IsNonFinite(next.AreaSlack) || proofbound.IsNonFinite(next.VolSymDiff) {
		return nil, nil, errShiftDeclined
	}
	return &moved, &next, nil
}

// retryShifted runs attempt with the second operand moved by up to the
// document's gap tolerance, after attempt(zero) refused with contacts the exact
// predicates cannot classify. It returns the first outcome that is not such a
// refusal, and the original refusal when no shift gives a different outcome.
func retryShifted(ctx context.Context, refusal error, op meshbool.OperationKind,
	attempt func(r3.Vec) (Evaluation, error)) (Evaluation, error) {
	var coplanar *meshbool.TouchContactError
	if !errors.As(refusal, &coplanar) {
		return Evaluation{}, refusal
	}
	reach := gaptol.Reach(ctx)
	if !(reach > 0) {
		return Evaluation{}, refusal
	}
	dirs := shiftDirections(op, coplanar.Contacts)
	for _, step := range shiftSteps {
		for _, dir := range dirs {
			if err := ctx.Err(); err != nil {
				return Evaluation{}, err
			}
			ev, err := attempt(dir.Scale(step * reach))
			if errors.Is(err, errShiftDeclined) {
				continue
			}
			var again *meshbool.TouchContactError
			if errors.As(err, &again) {
				continue
			}
			return ev, err
		}
	}
	return Evaluation{}, refusal
}
