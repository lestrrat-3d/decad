package meshbool

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
)

type BooleanExpectedKind int

const (
	BooleanExpectedEmpty BooleanExpectedKind = iota
	// BooleanExpectedContact is a TRUE contact/proximity refusal of the boolean
	// geometry itself: an unclassifiable tangent contact (ErrUnclassifiableContact
	// / MeshBoolean) or an undecidable near-miss (refuseUndecidableProximity). It
	// maps to the public BooleanUnsupportedContact.
	BooleanExpectedContact
	// BooleanExpectedUnsupported is an in-pipeline reach of the boolean geometry
	// on operands that DID tessellate — a collapsed or welded-away facet
	// (prepBoolMesh / stitchFacets) or a trim amplification that outgrew the pair
	// diameter (RimBound, refused by the root). Like a contact refusal the model is real and the limit
	// is the evaluator's reach, so it too maps to BooleanUnsupportedContact.
	BooleanExpectedUnsupported
	// BooleanExpectedStaging is a capability/staging limit of one operand: a
	// mesh the operand cannot give (a tessellation ErrUnsupported reached
	// before any contact is examined), or a restating operand whose facets
	// where the pair meets hold bounds coarser than the pair tolerance
	// (RefuseCoarseHeldContact, which runs on the contact classification but
	// before any facet is cut). The limit is the operand's held geometry, not
	// the contact's shape, so it is NOT a contact refusal — it passes through
	// the public boundary as a plain ErrUnsupported, never a BooleanError.
	BooleanExpectedStaging
	BooleanExpectedCoarseTessellation
	// BooleanExpectedVolumeProof is BooleanExpectedStaging's sibling for the one
	// staging cause that is NOT a tessellation limit: the operand meshes, but its
	// mesh carries no proof of the volume it and the body it stands for differ by
	// (docs/tessellation-design.md §11), so no boolean may compose it. It maps to
	// the same public error as BooleanExpectedStaging; it exists so Verify can say
	// which of the two happened.
	BooleanExpectedVolumeProof
)

// BooleanExpectedError identifies an ordinary geometric non-result for the
// read-only evaluator. Public booleans still expose the wrapped sentinel;
// Verify recognizes the private type and leaves the pair undecided. Invariant
// failures remain ordinary errors and return from Verify.
type BooleanExpectedError struct {
	Kind    BooleanExpectedKind
	Operand int
	Err     error
}

func (e *BooleanExpectedError) Error() string { return e.Err.Error() }

func (e *BooleanExpectedError) Unwrap() error { return e.Err }

func ExpectedBoolean(kind BooleanExpectedKind, err error) error {
	return ExpectedBooleanForOperand(kind, -1, err)
}

// ExpectedBooleanForOperand retains which operand hit a pre-contact staging
// limit. Other expected outcomes concern the pair or the result and use -1.
func ExpectedBooleanForOperand(kind BooleanExpectedKind, operand int, err error) error {
	var expected *BooleanExpectedError
	if errors.As(err, &expected) {
		return err
	}
	return &BooleanExpectedError{Kind: kind, Operand: operand, Err: err}
}

// CoarseHeldContactError is RefuseCoarseHeldContact's refusal: operand
// Operand's facets where the pair meets carry a bound of Bound, coarser than
// the pair's chord tolerance Tol. The root restates it in the boolean's own
// terms (boolean.go, booleanOperandStaging).
type CoarseHeldContactError struct {
	Operand    int
	Bound, Tol float64
}

func (e *CoarseHeldContactError) Error() string {
	return fmt.Sprintf(`%v: operand %d's held facets where the pair meets carry a bound of %g mm, coarser than the pair's chord tolerance %g mm`,
		decaderr.ErrUnsupported, e.Operand, e.Bound, e.Tol)
}

func (e *CoarseHeldContactError) Unwrap() error { return decaderr.ErrUnsupported }

// RefuseCoarseHeldContact is docs/faceted-vertex-bounds-design.md §5's local
// chain-depth gate: it refuses when any of the listed facets of a gated
// operand (m.Gate > 0) carries a facet bound above m.Gate, the pair's chord
// tolerance, and quotes the largest such bound. An ungated operand passes.
// Reject-only: it compares two proven numbers and admits nothing; a facet it
// passes enters the composition with its own bound.
func RefuseCoarseHeldContact(m *BoolMesh, operand int, facets []int) error {
	if m.Gate <= 0 {
		return nil
	}
	worst := 0.0
	for _, f := range facets {
		worst = max(worst, m.FacetBound[f])
	}
	if worst <= m.Gate {
		return nil
	}
	return ExpectedBooleanForOperand(BooleanExpectedStaging, operand,
		&CoarseHeldContactError{Operand: operand, Bound: worst, Tol: m.Gate})
}

// NearContacts is what GatherNearContacts collects for one face pair: the
// facets of each side that meet or come within slack of the other side, and
// every contact segment among those meets.
type NearContacts struct {
	CloseA, CloseB []int
	Spans          []ContactSpan
}

// GatherNearContacts classifies every facet pair of two faces whose boxes come
// within slack and collects the pairs that meet or stay within slack. deferred
// reports a coplanar face-on-face overlap, which facesNearMiss leaves to the
// mesh pass's own refusal.
func GatherNearContacts(ctx context.Context, bmA *BoolMesh, fis []int, bmB *BoolMesh, fjs []int, slack float64, memo *ContactMemo) (NearContacts, bool, error) {
	var nc NearContacts
	seenA := map[int]bool{}
	seenB := map[int]bool{}
	work := 0
	contacts := NewContactBatchExecutor(ctx, bmA, bmB, memo, ContactWorkers(ctx), func(pair ContactPair, c TriContact) error {
		i, j := pair.I, pair.J
		if c.Kind == ContactRegion {
			return ErrContactBatchStop
		}
		if c.Kind == ContactNone && TriTriDistance(TriCorners(bmA, i), TriCorners(bmB, j)) > slack {
			return nil
		}
		if c.Kind == ContactSegment {
			nc.Spans = append(nc.Spans, ContactSpan{I: i, J: j, P0: c.P0, P1: c.P1})
		}
		if !seenA[i] {
			seenA[i] = true
			nc.CloseA = append(nc.CloseA, i)
		}
		if !seenB[j] {
			seenB[j] = true
			nc.CloseB = append(nc.CloseB, j)
		}
		return nil
	})
	for _, i := range fis {
		for _, j := range fjs {
			work++
			if work%256 == 0 {
				if err := ctx.Err(); err != nil {
					return NearContacts{}, false, err
				}
			}
			if !BoxesWithin(bmA.Boxes[i], bmB.Boxes[j], slack) {
				continue
			}
			if err := contacts.Add(i, j); err != nil {
				if err == ErrContactBatchStop {
					return NearContacts{}, true, nil
				}
				return NearContacts{}, false, err
			}
		}
	}
	if err := contacts.Done(); err != nil {
		if err == ErrContactBatchStop {
			return NearContacts{}, true, nil
		}
		return NearContacts{}, false, err
	}
	return nc, false, nil
}

// ContactSpan is one exact contact segment of a face pair the gate examines:
// facet i of operand A and facet j of operand B meet along p0–p1, and every
// point of that segment lies on both closed facets (TriTriClassify).
type ContactSpan struct {
	I, J   int
	P0, P1 proof.Xpt
}

// MaxDepthWitnessFacets caps how many contacting facets deepWitnessInside
// probes at their fixed sample points on each side of a face pair, and
// MaxDepthWitnessSpans caps how many contact segments SpanWitness walks from.
// The fixed samples sit wherever the tessellation put the facet's corners, so a
// LONG facet can cross the other operand deeply while all seven of its samples
// lie outside it (two prism walls crossing near the middle of their length);
// the segment walk finds that crossing's witness at the segment itself. The
// caps only bound the work the reject-only refusal path spends confirming no
// witness exists: capping over-refuses at worst, which is sound.
const (
	MaxDepthWitnessFacets = 96
	MaxDepthWitnessSpans  = 96
	// SpanWalkSteps is how many halving steps SpanWitness takes from a contact
	// segment's midpoint toward one facet corner: the step lengths run from
	// half the corner's distance down to 2^-SpanWalkSteps of it.
	SpanWalkSteps = 24
)

// SpanWitness walks each contact segment's two facets for a deep witness.
// Where facet i of A crosses facet j of B, the points of facet i next to their
// shared segment lie on the inner side of facet j's plane on one side of the
// segment, and so inside B whenever the crossing is real and B's other facets
// stay farther than b away. The walk starts at the segment's exact midpoint,
// which lies on both facets, and steps toward each corner of facet i that lies
// STRICTLY on the inner side of facet j's plane (its outward normal is the CCW
// winding normal), halving the step each time. Every candidate is the exact
// point mid + (c − mid)/2^k, which lies on facet i because the facet is convex.
// The same walk then runs on facet j against A. Points toward a corner on the
// outer side lie outside the other solid near the segment, so the walk skips
// that corner rather than pay an exact parity test per step; a skipped
// candidate can only refuse. The halving
// stops once the step falls below b, because the midpoint lies on the other
// operand's boundary and no point within b of it can be deeper than b.
// Reject-only: the walk only nominates; DeepWitnessAt certifies each witness.
func SpanWitness(ctx context.Context, bmA, bmB *BoolMesh, spans []ContactSpan, b float64) (bool, error) {
	if len(spans) == 0 {
		return false, nil
	}
	allA, allB := AllFacets(bmA), AllFacets(bmB)
	one, two := big.NewInt(1), big.NewInt(2)
	for n, s := range spans {
		if n >= MaxDepthWitnessSpans {
			break
		}
		if err := ctx.Err(); err != nil {
			return false, err
		}
		mid := proof.Xlerp(s.P0, s.P1, one, two)
		deep, err := WalkFacetFromSpan(ctx, mid, bmA, s.I, bmB, s.J, allB, b)
		if err != nil || deep {
			return deep, err
		}
		deep, err = WalkFacetFromSpan(ctx, mid, bmB, s.J, bmA, s.I, allA, b)
		if err != nil || deep {
			return deep, err
		}
	}
	return false, nil
}

// WalkFacetFromSpan is one side of SpanWitness: from mid, a point on facet fi
// of m and on facet fj of other, it steps toward every corner of facet fi that
// lies strictly on the inner side of facet fj's plane and asks DeepWitnessAt
// about each step.
func WalkFacetFromSpan(ctx context.Context, mid proof.Xpt, m *BoolMesh, fi int, other *BoolMesh, fj int, all []int, b float64) (bool, error) {
	ot := other.Tris[fj]
	origin, normal := other.Xverts[ot[0]], other.Norms[fj]
	midF := mid.Vec()
	one := big.NewInt(1)
	for _, vi := range m.Tris[fi] {
		c := m.Xverts[vi]
		if proof.XdotSign(normal, proof.Xsub(c, origin)) >= 0 {
			continue
		}
		// The float length only decides when to stop halving. Stopping early
		// drops candidates, which can only refuse, so its rounding is harmless.
		reach := m.Verts[vi].Sub(midF).Len()
		den := big.NewInt(1)
		for range SpanWalkSteps {
			den = new(big.Int).Lsh(den, 1)
			reach /= 2
			if reach*(1+1e-6) <= b {
				break
			}
			p := proof.Xlerp(mid, c, one, den)
			deep, err := DeepWitnessAt(ctx, p, other, all, b)
			if err != nil || deep {
				return deep, err
			}
		}
	}
	return false, nil
}

// DeepWitnessAt reports whether the exact point p is PROVEN to lie strictly
// inside other's solid, deeper than b: its certified LOWER-bound distance to the
// other mesh's boundary exceeds b, and the exact ray-parity predicate places it
// strictly inside (never on a boundary). This is the one certification every
// witness search goes through. The cheap float depth runs before the costly
// exact parity: a point no deeper than b is no witness however it classifies,
// so the order changes which points are tested, never which become witnesses.
func DeepWitnessAt(ctx context.Context, p proof.Xpt, other *BoolMesh, all []int, b float64) (bool, error) {
	if CertifiedInteriorDepth(p, other) <= b {
		return false, nil
	}
	inside, onBoundary, err := MeshParityPreparedContext(ctx, p, other.Parity, all)
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return false, ctxErr
		}
		// The only other error is an all-axes-ambiguous parity ray: this point
		// cannot be PROVEN inside, so it is no witness — reject-only, never a
		// failure of the whole boolean.
		return false, nil
	}
	return inside && !onBoundary, nil
}

// CertifiedInteriorDepth is a certified LOWER bound (mm) on the distance from the
// exact interior point p to the boundary of the other operand's mesh: the float
// point-to-facet distances, taken at their minimum and nudged DOWN for their own
// float error, then reduced by an upper bound on p's exact→float rounding. It is
// only ever read as "> b", so under-reporting is safe (it over-refuses, never
// over-admits).
func CertifiedInteriorDepth(p proof.Xpt, other *BoolMesh) float64 {
	pf := p.Vec()
	best := math.Inf(1)
	for i := range other.Tris {
		// The facet lies inside its own containing box, so the point's distance
		// to that box never exceeds its distance to the facet. A box already at or
		// beyond the running minimum therefore cannot hold a nearer facet — skip
		// it. This prunes on distance alone and cannot lower the minimum below the
		// unpruned value; any float-boundary case where a skipped box rounds a hair
		// past the true nearest is already covered by the 1e-12 down-nudge below,
		// so the certified LOWER bound is preserved.
		if PointBoxDist(pf, other.Boxes[i]) >= best {
			continue
		}
		best = math.Min(best, PointTriDistance(pf, TriCorners(other, i)))
	}
	if best <= 0 || proofbound.IsNonFinite(best) {
		return 0
	}
	// best is the float distance from the ROUNDED point pf; the true distance
	// from the exact point p is at least (best, nudged down for the distance
	// evaluation's own float error) minus how far pf sits from p.
	return best*(1-1e-12) - PointRoundBound(p, pf)
}

// PointRoundBound upper-bounds the 3D displacement between the exact point p and
// its float rounding pf: the largest per-coordinate rational gap, up-rounded and
// read as a 3D distance (internal/proofbound/bounds.go, proofbound.Radius3D).
func PointRoundBound(p proof.Xpt, pf r3.Vec) float64 {
	px, py, pz := proof.XhpRat(proof.Xhp(p))
	worst := new(big.Rat)
	for _, pair := range [][2]*big.Rat{{px, polynomial.MustRatOf(pf.X)}, {py, polynomial.MustRatOf(pf.Y)}, {pz, polynomial.MustRatOf(pf.Z)}} {
		d := new(big.Rat).Sub(pair[0], pair[1])
		d.Abs(d)
		if d.Cmp(worst) > 0 {
			worst = d
		}
	}
	if worst.Sign() == 0 {
		return 0
	}
	w, _ := worst.Float64()
	return proofbound.Radius3D(proofbound.ProvenUpRound(w))
}

// BoxesWithin reports whether the two boxes come within slack of each other.
func BoxesWithin(a, b [2]r3.Vec, slack float64) bool {
	return a[0].X-slack <= b[1].X && b[0].X-slack <= a[1].X &&
		a[0].Y-slack <= b[1].Y && b[0].Y-slack <= a[1].Y &&
		a[0].Z-slack <= b[1].Z && b[0].Z-slack <= a[1].Z
}

// TriTriDistance is the distance between two DISJOINT triangles: the smallest
// of the nine edge-edge distances and the six vertex-to-triangle distances,
// which is where the minimum of two disjoint convex sets is always attained.
// The result is nudged DOWN so its own float rounding can only widen the
// refusal, never narrow it.
//
// Disjointness is the CALLER's obligation, discharged by the exact classifier
// before the call, never by this routine: an intersecting pair attains its
// minimum where the two triangles' interiors meet, which this candidate set
// does not contain, so the value returned for such a pair is neither the
// distance nor a lower bound on it and must never gate an admission.
func TriTriDistance(ta, tb [3]r3.Vec) float64 {
	best := math.Inf(1)
	for i := range 3 {
		for j := range 3 {
			best = math.Min(best, SegSegDist3(ta[i], ta[(i+1)%3], tb[j], tb[(j+1)%3]))
		}
	}
	for i := range 3 {
		best = math.Min(best, PointTriDistance(ta[i], tb))
		best = math.Min(best, PointTriDistance(tb[i], ta))
	}
	if best <= 0 || proofbound.IsNonFinite(best) {
		return 0
	}
	return best * (1 - 1e-12)
}

// SegSegDist3 is the distance between two closed 3D segments.
func SegSegDist3(p0, p1, q0, q1 r3.Vec) float64 {
	u := p1.Sub(p0)
	v := q1.Sub(q0)
	w := p0.Sub(q0)
	a, b, c := u.Dot(u), u.Dot(v), v.Dot(v)
	d, e := u.Dot(w), v.Dot(w)
	den := a*c - b*b
	var s, t float64
	if den <= 0 {
		// Parallel (or a collapsed segment): pin s and solve for t.
		s, t = 0, Clamp01(e, c)
	} else {
		s = Clamp01(b*e-c*d, den)
		t = Clamp01(a*e-b*d, den)
	}
	// Re-clamp against the other segment's ends, the standard two-pass fix.
	if tn := e + s*b; c > 0 {
		t = Clamp01(tn, c)
	}
	if sn := -d + t*b; a > 0 {
		s = Clamp01(sn, a)
	}
	return p0.Add(u.Scale(s)).Sub(q0.Add(v.Scale(t))).Len()
}

func Clamp01(num, den float64) float64 {
	if den <= 0 {
		return 0
	}
	return math.Max(0, math.Min(1, num/den))
}

// PointBoxDist is the Euclidean distance from a point to an axis-aligned box
// (zero when the point is inside it). It lower-bounds the distance to anything
// the box contains, so it is a sound distance prune for a nearest-facet scan.
func PointBoxDist(p r3.Vec, box [2]r3.Vec) float64 {
	axis := func(v, lo, hi float64) float64 {
		switch {
		case v < lo:
			return lo - v
		case v > hi:
			return v - hi
		default:
			return 0
		}
	}
	dx := axis(p.X, box[0].X, box[1].X)
	dy := axis(p.Y, box[0].Y, box[1].Y)
	dz := axis(p.Z, box[0].Z, box[1].Z)
	return math.Sqrt(dx*dx + dy*dy + dz*dz)
}

// PointTriDistance is the distance from a point to a closed triangle: the
// perpendicular foot when it lands inside, otherwise the nearest edge.
func PointTriDistance(p r3.Vec, t [3]r3.Vec) float64 {
	n := t[1].Sub(t[0]).Cross(t[2].Sub(t[0]))
	if nn := n.Dot(n); nn > 0 {
		h := n.Dot(p.Sub(t[0])) / nn
		foot := p.Sub(n.Scale(h))
		if PointInTriangle(foot, t, n) {
			return p.Sub(foot).Len()
		}
	}
	best := math.Inf(1)
	for i := range 3 {
		best = math.Min(best, SegSegDist3(p, p, t[i], t[(i+1)%3]))
	}
	return best
}

// PointInTriangle reports whether a point already on the triangle's plane lies
// inside it, by the three edge cross products against the facet normal.
func PointInTriangle(p r3.Vec, t [3]r3.Vec, n r3.Vec) bool {
	for i := range 3 {
		if t[(i+1)%3].Sub(t[i]).Cross(p.Sub(t[i])).Dot(n) < 0 {
			return false
		}
	}
	return true
}
