package booleaneval

import (
	"cmp"
	"context"
	"errors"
	"fmt"
	"math"
	"slices"
	"strings"

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
// first, so the result departs from the true Boolean by as little as it can. A
// contact with chord slack needs more than its slack, so a step is the larger of
// the fraction and the least shift that reaches the slack (shiftNeed).
var shiftSteps = [...]float64{1.0 / 64, 1.0 / 16, 1.0 / 4, 1}

// shiftMargin is how far past a contact's slack the least shift reaches.
const shiftMargin = 1.05

// shiftNeed is the length of the shortest shift along u that reaches the slack
// of every contact: the slack over the component of u along its normal, with the
// margin. A contact with no slack needs nothing.
func shiftNeed(u r3.Vec, contacts []meshbool.TouchContact) float64 {
	need := 0.0
	for _, c := range contacts {
		if c.Slack > 0 {
			need = math.Max(need, shiftMargin*c.Slack/math.Abs(u.Dot(c.NormalA)))
		}
	}
	return need
}

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
	base, hasBase := weightedBase(sigma, wanted, contacts)
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

// weightedBase is the direction that reaches every contact's slack with the
// shortest shift, or close to it. Each contact pulls with a weight in proportion
// to its slack, so a contact with little slack does not set the length of the
// shift, and a contact with none pulls with almost nothing. A wanted contact
// pulls in the sense sigma names. Any other contact pulls in the sense the pull
// so far already leans, since moving the second operand either way decides it.
func weightedBase(sigma float64, wanted []bool, contacts []meshbool.TouchContact) (r3.Vec, bool) {
	weight := func(c meshbool.TouchContact) float64 { return math.Max(c.Slack, 1e-6) }
	var pull r3.Vec
	var rest []meshbool.TouchContact
	for i, c := range contacts {
		if wanted[i] {
			pull = pull.Add(c.NormalA.Scale(sigma * weight(c)))
		} else {
			rest = append(rest, c)
		}
	}
	slices.SortStableFunc(rest, func(a, b meshbool.TouchContact) int { return cmp.Compare(weight(b), weight(a)) })
	for _, c := range rest {
		sense := 1.0
		if pull.Dot(c.NormalA) < 0 {
			sense = -1
		}
		pull = pull.Add(c.NormalA.Scale(sense * weight(c)))
	}
	return pull.Normalize()
}

// wantedContacts marks the contacts whose opening direction matters to op: the
// Decisive ones when there are any, and otherwise every contact for Cut and
// Intersect and none for Union.
func wantedContacts(op meshbool.OperationKind, contacts []meshbool.TouchContact) []bool {
	mask := make([]bool, len(contacts))
	decisive := false
	for i, c := range contacts {
		if c.Decisive {
			mask[i], decisive = true, true
		}
	}
	if !decisive && op != meshbool.OpUnion {
		for i := range mask {
			mask[i] = true
		}
	}
	return mask
}

// shiftOpens reports whether moving the second operand along u opens every
// contact by at least shiftOpenMargin, and every wanted contact in the sense
// sigma names.
func shiftOpens(u r3.Vec, sigma float64, wanted []bool, contacts []meshbool.TouchContact) bool {
	for i, c := range contacts {
		along := u.Dot(c.NormalA)
		if math.Abs(along) < shiftOpenMargin {
			return false
		}
		if wanted[i] && sigma*along < shiftSignMargin {
			return false
		}
	}
	return true
}

// pieces partitions a mesh into its connected pieces, the sets of facets that
// share vertices, so a tool made of several separate bodies, such as the teeth of
// a gear, can have each piece moved on its own.
type pieces struct {
	ofVertex []int
	ofFacet  []int
	count    int
}

func piecesOf(m *topo.Mesh) pieces {
	parent := make([]int, len(m.Vertices))
	for i := range parent {
		parent[i] = i
	}
	find := func(x int) int {
		for parent[x] != x {
			parent[x] = parent[parent[x]]
			x = parent[x]
		}
		return x
	}
	for _, t := range m.Triangles {
		for _, v := range t[1:] {
			if a, b := find(t[0]), find(v); a != b {
				parent[max(a, b)] = min(a, b)
			}
		}
	}
	id := map[int]int{}
	pc := pieces{ofVertex: make([]int, len(m.Vertices)), ofFacet: make([]int, len(m.Triangles))}
	for i := range m.Vertices {
		root := find(i)
		c, ok := id[root]
		if !ok {
			c = len(id)
			id[root] = c
		}
		pc.ofVertex[i] = c
	}
	for f, t := range m.Triangles {
		pc.ofFacet[f] = pc.ofVertex[t[0]]
	}
	pc.count = len(id)
	return pc
}

// shiftOperandMesh returns two copies of m with each piece moved by its own
// shift, a piece with none staying where it is. The decision copy keeps m's
// bounds: the moved operand is the solid the Boolean decides about, so the
// contact gate reads it exactly as it reads an unmoved one. The charged copy
// describes the moved mesh as an approximation of the true operand, which the
// composition of the result's bounds reads: the vertex and face bounds of a
// moved piece grow by that piece's displacement, the mesh bound by the largest,
// the symmetric-difference volume by what each move sweeps over its piece's
// boundary, and the area allowance by the rounding of the new coordinates. m is
// not modified. It returns errShiftDeclined when the moved mesh folds through
// itself.
func shiftOperandMesh(ctx context.Context, m *topo.Mesh, pc pieces, shifts map[int]r3.Vec) (decision, charged *topo.Mesh, displacementBound float64, err error) {
	verts := make([]r3.Vec, len(m.Vertices))
	moved := make([]bool, len(m.Vertices))
	maxAbs := 0.0
	for i, v := range m.Vertices {
		sh := shifts[pc.ofVertex[i]]
		if !proofbound.FiniteVec(sh) {
			return nil, nil, 0, errShiftDeclined
		}
		verts[i] = v.Add(sh)
		moved[i] = sh != (r3.Vec{})
		maxAbs = math.Max(maxAbs, math.Max(math.Abs(verts[i].X), math.Max(math.Abs(verts[i].Y), math.Abs(verts[i].Z))))
	}
	// Each moved coordinate is the float sum of an exact coordinate and a
	// shift component, so it is within an ulp of the exact sum.
	round := 2 * proofbound.UlpOf(maxAbs)
	displacement := make(map[int]float64, len(shifts))
	worst := 0.0
	for c, sh := range shifts {
		d := proofbound.AbsSumUpper(proofbound.DvLenUpper(proofbound.HeldDelta(sh, r3.Vec{})), proofbound.Radius3D(round))
		if proofbound.IsNonFinite(d) {
			return nil, nil, 0, errShiftDeclined
		}
		displacement[c] = d
		worst = max(worst, d)
	}

	h := meshbool.HeldRounding{
		Verts: verts, Tris: m.Triangles,
		Exact: make([]proof.Xpt, len(verts)), Moved: moved, Movable: make([]bool, len(verts)),
	}
	for i, v := range verts {
		h.Exact[i] = proof.XptOf(v)
	}
	folds, err := meshbool.HeldFolds(ctx, h)
	if err != nil {
		return nil, nil, 0, err
	}
	if len(folds) > 0 {
		return nil, nil, 0, errShiftDeclined
	}

	beta, err := topobuild.VertexBounds(m)
	if err != nil {
		return nil, nil, 0, err
	}
	for i := range beta {
		beta[i] = proofbound.AbsSumUpper(beta[i], displacement[pc.ofVertex[i]])
	}
	faceMove := make(map[*topo.Face]float64, len(m.FaceBound))
	trisOf := make(map[int][][3]int, len(shifts))
	for f, t := range m.Triangles {
		c := pc.ofFacet[f]
		d, ok := displacement[c]
		if !ok {
			continue
		}
		trisOf[c] = append(trisOf[c], t)
		if f < len(m.Source) && m.Source[f] != nil {
			faceMove[m.Source[f]] = max(faceMove[m.Source[f]], d)
		}
	}
	faceBound := make(map[*topo.Face]float64, len(m.FaceBound))
	for f, b := range m.FaceBound {
		faceBound[f] = proofbound.AbsSumUpper(b, faceMove[f])
	}
	volume, perimeter := 0.0, 0.0
	for c, tris := range trisOf {
		swept, err := proofbound.PerturbedAreaUpperContext(ctx, verts, tris, displacement[c])
		if err != nil {
			return nil, nil, 0, err
		}
		volume = proofbound.AbsSumUpper(volume, proofbound.SweptVolumeAllow(displacement[c], swept))
		for _, t := range tris {
			a, b, c := verts[t[0]], verts[t[1]], verts[t[2]]
			perimeter += b.Sub(a).Len() + c.Sub(b).Len() + a.Sub(c).Len()
		}
	}
	perimeter = proofbound.UpRound(perimeter + proofbound.SumSlop(3*len(m.Triangles), perimeter))
	areaRound := proofbound.ProductUpper(proofbound.Radius3D(round), perimeter)

	decided := *m
	decided.Vertices = verts
	decided.Owner = nil

	next := decided
	next.Bound = proofbound.AbsSumUpper(m.Bound, worst)
	next.CoordBound = proofbound.AbsSumUpper(m.CoordBound, worst)
	next.VertexBound = beta
	next.FaceBound = faceBound
	next.AreaSlack = proofbound.AbsSumUpper(m.AreaSlack, areaRound)
	next.VolSymDiff = proofbound.AbsSumUpper(m.VolSymDiff, volume)
	if proofbound.IsNonFinite(next.AreaSlack) || proofbound.IsNonFinite(next.VolSymDiff) {
		return nil, nil, 0, errShiftDeclined
	}
	return &decided, &next, worst, nil
}

// retryShifted runs attempt with the pieces of the second operand moved by up to
// the document's gap tolerance, after attempt(nil) refused with contacts the
// exact predicates or the proximity gate cannot classify. Each piece that a
// contact names moves on its own, along the direction that opens its contacts
// and by the shortest length that reaches their slack, so the pieces of a tool
// such as the teeth of a gear are each charged once, never one through another.
// It returns the first outcome that is not such a refusal, and the original
// refusal when no shift gives a different outcome. A shifted attempt can meet a
// contact the unmoved pair did not reach, such as the coplanar caps of a tooth
// the proximity gate stopped first, so the contacts of every refusal are
// gathered and the directions are chosen again for all of them, up to
// shiftRounds times.
func retryShifted(ctx context.Context, refusal error, op meshbool.OperationKind, pc pieces,
	attempt func(map[int]r3.Vec) (Evaluation, error)) (Evaluation, error) {
	var touch *meshbool.TouchContactError
	if !errors.As(refusal, &touch) {
		return Evaluation{}, refusal
	}
	reach := gaptol.Reach(ctx)
	if !(reach > 0) {
		return Evaluation{}, refusal
	}
	byPiece := groupContacts(pc, nil, touch.Contacts)
	for range shiftRounds {
		dirs := make(map[int][]r3.Vec, len(byPiece))
		order := make([]int, 0, len(byPiece))
		for c, contacts := range byPiece {
			dirs[c] = shiftDirections(op, contacts)
			if len(dirs[c]) == 0 {
				return Evaluation{}, refusal
			}
			order = append(order, c)
		}
		slices.Sort(order)
		tried := map[string]struct{}{}
		grown := false
	steps:
		for _, step := range shiftSteps {
			for variant := range shiftWeights {
				if err := ctx.Err(); err != nil {
					return Evaluation{}, err
				}
				shifts := make(map[int]r3.Vec, len(order))
				fits := true
				var key strings.Builder
				for _, c := range order {
					dir := dirs[c][variant%len(dirs[c])]
					length := math.Max(step*reach, shiftNeed(dir, byPiece[c]))
					if length > reach {
						fits = false
						break
					}
					shifts[c] = dir.Scale(length)
					fmt.Fprintf(&key, "%d:%v;", c, shifts[c])
				}
				if !fits {
					continue
				}
				if _, dup := tried[key.String()]; dup {
					continue
				}
				tried[key.String()] = struct{}{}
				ev, err := attempt(shifts)
				if errors.Is(err, errShiftDeclined) {
					continue
				}
				var again *meshbool.TouchContactError
				if errors.As(err, &again) {
					// A contact this shift met and the last did not changes
					// which directions and lengths open them all, so the
					// search starts over with it before any longer shift.
					if more := groupContacts(pc, byPiece, again.Contacts); contactCount(more) > contactCount(byPiece) {
						byPiece, grown = more, true
						break steps
					}
					continue
				}
				return ev, err
			}
		}
		if !grown {
			break
		}
	}
	return Evaluation{}, refusal
}

// contactCount is the number of contacts held across all pieces.
func contactCount(byPiece map[int][]meshbool.TouchContact) int {
	n := 0
	for _, contacts := range byPiece {
		n += len(contacts)
	}
	return n
}

// groupContacts adds contacts to a copy of held, each to the pieces of the second
// operand its facets lie in, and to every piece when it names none. A contact a
// piece already holds, to a thousandth of a unit of its normal, is not added
// again.
func groupContacts(pc pieces, held map[int][]meshbool.TouchContact, contacts []meshbool.TouchContact) map[int][]meshbool.TouchContact {
	out := make(map[int][]meshbool.TouchContact, len(held))
	for c, cs := range held {
		out[c] = slices.Clone(cs)
	}
	for _, contact := range contacts {
		targets := map[int]struct{}{}
		for _, f := range contact.FacetsB {
			if f >= 0 && f < len(pc.ofFacet) {
				targets[pc.ofFacet[f]] = struct{}{}
			}
		}
		if len(targets) == 0 {
			for c := range pc.count {
				targets[c] = struct{}{}
			}
		}
		for c := range targets {
			out[c] = mergeContacts(out[c], []meshbool.TouchContact{contact})
		}
	}
	return out
}

// shiftRounds is how many times retryShifted widens the contacts it opens.
const shiftRounds = 4

// mergeContacts returns contacts followed by each contact of more whose normal
// and decisiveness it does not already hold, to a thousandth of a unit.
func mergeContacts(contacts, more []meshbool.TouchContact) []meshbool.TouchContact {
	type key struct {
		x, y, z  int64
		decisive bool
	}
	keyOf := func(c meshbool.TouchContact) key {
		return key{int64(math.Round(c.NormalA.X * 1e3)), int64(math.Round(c.NormalA.Y * 1e3)),
			int64(math.Round(c.NormalA.Z * 1e3)), c.Decisive}
	}
	seen := make(map[key]struct{}, len(contacts))
	for _, c := range contacts {
		seen[keyOf(c)] = struct{}{}
	}
	out := slices.Clone(contacts)
	for _, c := range more {
		if _, dup := seen[keyOf(c)]; dup {
			continue
		}
		seen[keyOf(c)] = struct{}{}
		out = append(out, c)
	}
	return out
}
