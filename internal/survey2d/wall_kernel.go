package survey2d

import (
	"errors"
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// The 2D kernel evaluates the analytic wall survey
// (docs/verification-design.md §6, docs/evaluator-design.md §10): the
// spanning-ball reading of a prism or revolved body reduces exactly to
// empty-disk analysis in a 2D section — the profile plane for a prism, the
// meridian plane through the ball's center for a solid of revolution — so the
// survey enumerates a closed-form candidate set of critical inscribed disks
// and validates each against the whole boundary. The candidate set is
// complete for the attained infimum/supremum over line/arc boundaries: a
// critical disk has three contacts (an Apollonius solution), two contacts at
// an antipodal configuration (a pair critical), two contacts exactly at the
// allowance boundary (an angle-limit disk — the closed-under-limits family of
// §6), or a whole-arc contact (a concentric disk). Corner pinches — dihedrals
// within the allowance, which span at every size a ball is starved to — are
// the caller's to report; they read zero and need no disk.
//
// Every candidate carries a PROVEN bound beside its radius (DiskCand.rBound):
// each family derives it from its own closed form over the bounded-scalar
// arithmetic moments.go already owns (proofbound.BoundedAdd/proofbound.BoundedSub/proofbound.BoundedMul/
// proofbound.BoundedQuotient/proofbound.BoundedSqrt), reading every element COORDINATE (line
// endpoints and normals, arc centers, junction vertices) as an EXACT leaf —
// the same convention prismPayload's own directional readings use — so for
// those the bound speaks only for the arithmetic THIS kernel performs, never for
// the record's own numbers or for the draft allowance angle itself. Every step
// of that arithmetic is charged, coefficient construction and Cramer
// determinants included: reading a coordinate as exact never licenses reading
// a PRODUCT of two of them as exact. A division's bound comes from
// proofbound.BoundedQuotient's own denominator-and-numerator composition, so a small
// denominator amplifies it without a separate case; nothing here loosens a
// candidate SET, a containment scan, or a tolerance — the bound only ever
// widens the interval the winning candidate publishes.
//
// The kernel takes exactly TWO inputs that are not exact leaves, and both
// arrive already bounded rather than being trusted:
//
//   - SurveyElem.rrBound, an arc element's own radius error. A caller that
//     states a radius outright, and a recorded CircleSeg, both give zero; a
//     walk built from a recorded ArcSeg holds a math.Hypot radius and gives
//     extrude.go's arcWalkRadiusBound. Every candidate radius derived from an
//     element radius reads it through SurveyElem.rrBS, never as a leaf.
//   - WallKernel's wedgeS, the partial-revolve cap half-angle's SINE, which
//     its caller proves from a certified trig interval.
//
// Every candidate derived from either composes its bound like any other, so an
// input the caller could not enclose never reaches a published reading.
//
// A candidate's own bound is on its RADIUS, while the kernel's answer is a
// spanning DIAMETER (WallSurveyOut.span = 2r), so the bound is doubled with
// the value it accompanies. runBudget performs that conversion at the single
// point where a candidate enters the answer, so every field of
// WallSurveyOut that carries a bound already speaks in the answer's own
// units and no consumer restates the factor.

// DiskCand is a candidate inscribed disk. rBound is the PROVEN bound its own
// generator derived — zero only where that generator's arithmetic is exact
// (a literal coordinate difference, never a division or a square root of a
// non-perfect value).
type DiskCand struct{ X, Y, R, RBound float64 }

// WallKernel is one 2D section survey: elements (material on the left),
// junction vertices, the draft allowance, and — for a partial revolve — the
// wedge the sweep caps cut, encoded by wedgeS = sin(min(Δφ/2, π/2)): a ball
// of radius r centered at height y fits the wedge iff y·wedgeS ≥ r.
//
// wedgeS is one of the two kernel inputs that are NOT exact leaves (the kernel
// comment names both; the other is an element's own radius): it is a sine, so
// its caller (survey.go's revolveWall) proves it from a certified trig
// interval and hands it over as a proofbound.BoundedScalar. Every wedge-derived radius
// composes that bound through the same bounded arithmetic the rest of the kernel
// uses, so a candidate the wedge produced publishes an interval that contains
// the sine's own error.
type WallKernel struct {
	Elems       []SurveyElem
	ContainOnly []SurveyElem // boundary for containment only (on-axis chords)
	Verts       [][2]float64
	Alpha       float64
	// draftTrig holds certified ±(π−alpha) values in candidate order.
	DraftTrig  [2]struct{ sin, cos proofbound.BoundedScalar }
	WedgeS     proofbound.BoundedScalar // value 0 = no wedge
	WedgeSpans bool                     // two wedge-cap contacts count as a spanning pair
	FitMax     float64                  // spanning disks wider than this cannot lift to 3D
	Scale, Tol float64
	SubTolFar  bool         // a sub-tolerance candidate away from every junction was dropped
	Boundary   []SurveyElem // elems + containOnly, built lazily for contains
}

// NewWallKernel sizes the tolerances from the geometry with the default draft allowance.
func NewWallKernel(elems []SurveyElem, verts [][2]float64, fitMax float64) *WallKernel {
	k, _ := NewWallKernelBudget(nil, elems, nil, verts, 15*math.Pi/180, proofbound.ExactScalar(0), false, fitMax)
	return k
}

// NewWallKernelBudget sizes the tolerances from the geometry while charging
// the boundary scan to the caller's shared budget.
func NewWallKernelBudget(budget *proofbound.WorkBudget, elems, containOnly []SurveyElem, verts [][2]float64, alpha float64, wedgeS proofbound.BoundedScalar, wedgeSpans bool, fitMax float64) (*WallKernel, error) {
	if err := WallBudgetErr(budget); err != nil {
		return nil, err
	}
	scale := 1.0
	grow := func(vs ...float64) {
		for _, v := range vs {
			if a := math.Abs(v); a > scale {
				scale = a
			}
		}
	}
	for _, boundary := range [][]SurveyElem{elems, containOnly} {
		for _, e := range boundary {
			if err := WallBudgetStep(budget); err != nil {
				return nil, err
			}
			if e.Kind == SurveyLine {
				grow(e.Ax, e.Ay, e.Bx, e.By)
				continue
			}
			grow(e.Qx-e.Rr, e.Qx+e.Rr, e.Qy-e.Rr, e.Qy+e.Rr)
		}
	}
	aStar := math.Pi - alpha
	positiveSin, positiveCos := RadianTrigBounds(aStar)
	negativeSin, negativeCos := RadianTrigBounds(-aStar)
	return &WallKernel{
		Elems:       elems,
		ContainOnly: containOnly,
		Verts:       verts,
		Alpha:       alpha,
		DraftTrig: [2]struct{ sin, cos proofbound.BoundedScalar }{
			{sin: positiveSin, cos: positiveCos},
			{sin: negativeSin, cos: negativeCos},
		},
		WedgeS:     wedgeS,
		WedgeSpans: wedgeSpans,
		FitMax:     fitMax,
		Scale:      scale,
		Tol:        1e-9 * scale,
	}, nil
}

// WallSurveyOut is the kernel's answer over its candidate set. Both of its
// numbers are extrema over a population of candidates that each carry their
// OWN bound, so each is published as survey.go's §9.2 ExtremeAggregate proves
// it — the aggregate interval's midpoint beside its half-width — and never as
// the figures of whichever candidate won a comparison of held values. Two
// candidates whose held values order one way can have true values that order
// the other, and the aggregate is what stops the reading from following the
// held order off the truth.
//
// span and spanBound speak in the spanning DIAMETER, not in the candidate
// radius each generator actually bounded: runBudget doubles a candidate's
// value and its bound together as it feeds the aggregate, at the single point
// where a candidate enters the answer, so no consumer restates the factor.
type WallSurveyOut struct {
	Ok            bool    // false: the survey could not decide (never a silent pass)
	HasSpan       bool    // some empty spanning disk exists
	Span          float64 // the smallest spanning DIAMETER, as the aggregate proves it
	SpanBound     float64 // that aggregate interval's half-width, also on the DIAMETER
	Inradius      float64 // the largest empty disk radius found (the 2D inradius)
	InradiusBound float64 // that aggregate interval's half-width
	SubTolFar     bool    // an off-junction sub-tolerance candidate was dropped
}

var ErrWallSurveyUndecided = errors.New("wall survey undecided")

// WallBudgetStep counts one unit of wall work when Verify supplied a budget.
// The context-free shell-admission caller uses nil.
func WallBudgetStep(budget *proofbound.WorkBudget) error {
	if budget == nil {
		return nil
	}
	return budget.Step()
}

func WallBudgetErr(budget *proofbound.WorkBudget) error {
	if budget == nil {
		return nil
	}
	return budget.Err()
}

// run preserves the context-free kernel API used by shell admission.
func (k *WallKernel) Run() WallSurveyOut {
	out, _ := k.RunBudget(nil)
	return out
}

// runBudget streams and validates candidates under one shared Verify budget,
// folding each admitted candidate into the two §9.2 aggregates the kernel
// publishes — the least spanning diameter and the greatest empty radius.
// Neither extremum is taken by comparing held values: a candidate's own bound
// is its own, so the candidate that holds the smaller number need not be the
// one whose truth is smaller, and reducing by held order publishes an interval
// a rival's truth can sit outside of. The aggregates reach to whichever
// candidate's interval reaches furthest, which is also what keeps an exact
// reading exact: an inexact rival that does not reach the extremum moves
// neither end.
//
// A candidate whose own generator could not derive a finite bound never
// widens the published interval silently: it makes the whole survey
// undecided instead (the same refusal prismWall already takes for a
// displaced section), since a reading with no proven bound is not one this
// evaluator may stand behind. The aggregate itself carries that refusal, so a
// candidate that loses the extremum cannot smuggle an unbounded figure past
// it either.
func (k *WallKernel) RunBudget(budget *proofbound.WorkBudget) (WallSurveyOut, error) {
	out := WallSurveyOut{Ok: true, Span: math.Inf(1)}
	spans, radii := MinAggregate(), MaxAggregate()
	err := k.Generate(budget, func(c DiskCand) error {
		spanning, empty, ok, err := k.Validate(c, budget)
		if err != nil {
			return err
		}
		if !ok {
			return ErrWallSurveyUndecided
		}
		if !empty {
			return nil
		}
		radii.Take(c.R, c.RBound)
		if !spanning {
			return nil
		}
		// The candidate's generator bounded its RADIUS; the answer is the
		// DIAMETER 2r, so the bound doubles with the value. Multiplying a
		// float64 by two only scales the exponent, so the doubled bound is
		// exact and needs no outward rounding — the sole failure is an
		// overflow to +Inf, which the aggregate's own finiteness refusal
		// catches with every other underivable bound.
		//
		// The fit gate is read on the candidate's own proven interval, not on
		// its held diameter, and it is reject-only like validate's own guards:
		// a disk PROVEN wider than the sweep cannot lift to 3D and is dropped,
		// while one whose interval still reaches under the height is kept and
		// carries its doubt into the reading. Deciding a straddle the other way
		// would drop a wall the body may really have, which is the one error a
		// wall reading must never make.
		diam := proofbound.BoundedMul(proofbound.ExactScalar(2), proofbound.MeasuredScalar(c.R, c.RBound))
		if proofbound.AdmitAbove(diam, k.FitMax+k.Tol) == proofbound.SurvAdmit {
			return nil
		}
		out.HasSpan = true
		spans.Take(diam.Value, diam.Bound)
		return nil
	})
	if errors.Is(err, ErrWallSurveyUndecided) {
		return WallSurveyOut{}, nil
	}
	if err != nil {
		return WallSurveyOut{}, err
	}
	if out.HasSpan {
		span, bound, ok := spans.Resolve()
		if !ok {
			return WallSurveyOut{}, nil
		}
		out.Span, out.SpanBound = span, bound
	}
	if !radii.Empty() {
		inradius, bound, ok := radii.Resolve()
		if !ok {
			return WallSurveyOut{}, nil
		}
		out.Inradius, out.InradiusBound = inradius, bound
	} else if radii.Unbounded {
		return WallSurveyOut{}, nil
	}
	out.SubTolFar = k.SubTolFar
	return out, nil
}

// validate checks one candidate disk against the whole boundary: it must fit
// the wedge (if any), sit inside the material, and clear every element; its
// contact set decides whether it spans — two contact directions within the
// allowance of antipodal (inclusive), a single whole-arc contact wide
// enough, or (when the wedge's two cap contacts span) an active wedge.
//
// Every one of those readings is taken on the candidate's OWN proven interval
// rather than on its held radius, and each REJECTS only what that interval
// proves: an element proven to cut into the disk blocks it, an element proven
// clear of the rim is not a contact, a contact pair proven short of the
// allowance does not span. A straddle resolves the same way every generator's
// does — toward keeping the candidate and counting the contact — and the
// candidate's own interval carries the doubt into the reading.
//
// Resolving that way, rather than into the undecided channel, is a decision
// the numbers force and it is worth stating why. These guards compare against
// slacks the survey DECLARES — k.tol and contactTol() are 1e-9 and 8e-9 of the
// section's own scale — but the intervals read against them are NOT ulp-scale:
// a T3 angle-limit candidate composes the draft allowance's certified trig
// enclosure through a division, and on a 100 mm section its radius bound comes
// out near 1e-6 mm, two decades ABOVE k.tol. Straddles here are the ordinary
// case, not an exotic one, so an undecided cell would read Suspect on
// commonplace bodies — the wrong answer, since the doubt is a micron of
// arithmetic and the reading already has an interval to put it in.
//
// Keeping a straddling candidate is sound because the straddle bounds how far
// the true geometry can be from it. Where an element cannot be proven to cut
// into the disk, its distance d satisfies d >= r − b − k.tol, so an empty disk
// of radius min_j d_j really does sit at this centre, within the candidate's
// own published interval up to the declared slack — the same slack the held
// comparison already conceded. Where an element cannot be proven clear of the
// rim, a disk inside that interval really does touch it. What the interval
// cannot absorb is a bound that is not a number at all: runBudget's aggregate
// refuses a candidate whose generator could derive no finite bound, and that
// is the reading's undecided channel.
func (k *WallKernel) Validate(c DiskCand, budget *proofbound.WorkBudget) (bool, bool, bool, error) {
	if err := WallBudgetStep(budget); err != nil {
		return false, false, false, err
	}
	if math.IsNaN(c.X) || math.IsNaN(c.Y) || math.IsNaN(c.R) || math.IsInf(c.R, 0) {
		return false, false, true, nil
	}
	// The candidate floor: a disk this small is indistinguishable from the
	// numeric noise of a degenerate solve. Dropping it is safe ONLY where a
	// junction owns the spot — the junction-pinch rule reports the true
	// dihedral there exactly. Away from every junction the same tiny disk
	// can be a REAL near-tangent web (two boundary elements almost touching
	// mid-span), and treating it as absent would bless a body whose wall is
	// thinner than the kernel can resolve: that is an undecided survey,
	// never a silent pass.
	if c.R <= 4*k.Tol {
		reach := c.R + 8*k.Tol
		for _, v := range k.Verts {
			if err := WallBudgetStep(budget); err != nil {
				return false, false, false, err
			}
			if math.Hypot(c.X-v[0], c.Y-v[1]) <= reach {
				return false, false, true, nil
			}
		}
		// Not junction-owned: remember it. The caller decides — an exact
		// zero elsewhere still stands (nothing sits below zero), but a
		// positive reading or an absence claim would rest on a candidate
		// the kernel could not resolve, and reads undecided instead.
		k.SubTolFar = true
		return false, false, true, nil
	}
	wedgeActive := false
	if k.HasWedge() {
		// A fit/contact test against k.tol, not a published reading: the sine's
		// own bound reaches the answer through the candidate radii the wedge
		// generates. The FIT half still refuses only what the sine's own
		// interval proves does not fit, so a disk the wedge might admit is
		// never discarded on a held product. The CONTACT half below stays a
		// held comparison: its slack is 8·k.tol = 8e-9 of the section's own
		// scale while the product's bound is that scale times the sine's
		// last-ulp figure, so the interval cannot reach across the slack and
		// the straddle is unreachable.
		room := proofbound.BoundedMul(proofbound.ExactScalar(c.Y), k.WedgeS)
		if proofbound.AdmitBelow(proofbound.BoundedSub(room, k.ClearRadius(c)), 0) == proofbound.SurvAdmit {
			return false, false, true, nil
		}
		// The contact half reads the same two intervals: the wedge counts as a
		// contact unless the room it leaves is PROVEN past the slack.
		wedgeActive = proofbound.AdmitAbove(proofbound.BoundedSub(room, k.ContactRadius(c)), 0) != proofbound.SurvAdmit
	}
	var contacts []DirArc
	for _, e := range k.Elems {
		if err := WallBudgetStep(budget); err != nil {
			return false, false, false, err
		}
		d, dir, dirOK := e.Nearest(c.X, c.Y, SurvTiny*k.Scale)
		// Emptiness: drop the candidate only where the element is PROVEN to cut
		// into the disk.
		if proofbound.AdmitBelow(proofbound.BoundedSub(proofbound.ExactScalar(d), k.ClearRadius(c)), 0) == proofbound.SurvAdmit {
			return false, false, true, nil
		}
		// Contact: count the element unless it is PROVEN clear of the rim.
		if proofbound.AdmitAbove(proofbound.BoundedSub(proofbound.ExactScalar(d), k.ContactRadius(c)), 0) == proofbound.SurvAdmit {
			continue
		}
		if !dirOK {
			// A contact whose direction is undefined: the disk is
			// degenerate against this element; do not decide on it.
			return false, false, true, nil
		}
		contacts = append(contacts, dir)
	}
	inside, ok, err := k.Contains(c.X, c.Y, budget)
	if err != nil {
		return false, false, false, err
	}
	if !ok {
		return false, false, false, nil
	}
	if !inside {
		return false, false, true, nil
	}
	spanning := k.WedgeSpans && wedgeActive
	// The spanning threshold composes Go's float64 math.Pi, so it is not the
	// exact π − α − SurvAngTol it denotes and carries PiRoundGuard; the angle
	// beside it is this kernel's own atan2 composition and carries
	// AngleReadGuard. Both are decades under SurvAngTol, the inclusive slack
	// the survey DECLARES for a wall drafted at exactly the allowance, so the
	// declared line — not the arithmetic — is what decides that wall, and a
	// straddle needs a dihedral within a femtoradian of the declared line.
	need := proofbound.BoundedSub(
		proofbound.BoundedSub(proofbound.MeasuredScalar(math.Pi, PiRoundGuard), proofbound.ExactScalar(k.Alpha)),
		proofbound.ExactScalar(SurvAngTol),
	)
	for i := 0; i < len(contacts) && !spanning; i++ {
		for j := i; j < len(contacts); j++ {
			if err := WallBudgetStep(budget); err != nil {
				return false, false, false, err
			}
			ang := proofbound.MeasuredScalar(MaxAngleBetween(contacts[i], contacts[j]), AngleReadGuard)
			// A pair spans unless its own interval is PROVEN short of the
			// threshold, the same reject-only posture the two guards above
			// take.
			if proofbound.AdmitBelow(proofbound.BoundedSub(ang, need), 0) != proofbound.SurvAdmit {
				spanning = true
				break
			}
		}
	}
	return spanning, true, true, nil
}

// clearRadius and contactRadius are the two bands validate reads an element's
// distance against, each carrying the candidate's own proven radius bound: an
// element nearer than clearRadius cuts into the disk, one no further than
// contactRadius touches it. The slacks themselves are the survey's declared
// lines, not measurements, so they enter as exact leaves.
func (k *WallKernel) ClearRadius(c DiskCand) proofbound.BoundedScalar {
	return proofbound.BoundedSub(proofbound.MeasuredScalar(c.R, c.RBound), proofbound.ExactScalar(k.Tol))
}

func (k *WallKernel) ContactRadius(c DiskCand) proofbound.BoundedScalar {
	return proofbound.BoundedAdd(proofbound.MeasuredScalar(c.R, c.RBound), proofbound.ExactScalar(k.ContactTol()))
}

// contactTol is the slack for counting an element as a contact.
func (k *WallKernel) ContactTol() float64 { return 8 * k.Tol }

// AngleReadGuard is the survey's DECLARED conservative allowance on one
// contact-angle reading: MaxAngleBetween composes a handful of math.Atan2
// evaluations, differences and reductions over values no larger than 2π, and
// this kernel assumes no ulp contract from any of them (the same posture
// RadianTrigBounds takes toward math.Sin). Ten femtoradians is decades above
// the last ulp of 2π and five decades below SurvAngTol, the inclusive slack
// the spanning comparison is actually about, so declaring it costs the
// comparison nothing while keeping the reading from asserting an accuracy the
// library never promised.
const AngleReadGuard = 1e-14

// hasWedge reports whether this kernel carries a partial revolve's cap wedge at
// all. It is a STRUCTURAL question, not a numeric one: a full turn's caller
// states proofbound.ExactScalar(0), a proven zero, and a partial sweep's caller
// (survey.go's revolveWall) proves its own half-angle sine positive before
// handing it over — so the reading is never taken on a sine that cannot be
// told from zero, and the proofbound.SurvAdmit answer below is the only one a built
// kernel reaches.
func (k *WallKernel) HasWedge() bool {
	return proofbound.AdmitAbove(k.WedgeS, 0) == proofbound.SurvAdmit
}

// contains is the material-membership test: crossing parity of a ray against
// every boundary element, retried across directions when a crossing is
// ambiguous (near an endpoint, near tangency, or grazing the start). All
// directions ambiguous → undecided, never guessed.
func (k *WallKernel) Contains(px, py float64, budget *proofbound.WorkBudget) (bool, bool, error) {
	if k.Boundary == nil {
		boundary := make([]SurveyElem, 0, len(k.Elems)+len(k.ContainOnly))
		for _, elems := range [][]SurveyElem{k.Elems, k.ContainOnly} {
			for _, e := range elems {
				if err := WallBudgetStep(budget); err != nil {
					return false, false, err
				}
				boundary = append(boundary, e)
			}
		}
		k.Boundary = boundary
	}
	all := k.Boundary
	for i := range 16 {
		if err := WallBudgetStep(budget); err != nil {
			return false, false, err
		}
		th := 0.5 + float64(i)*2.399963229728653 // golden-angle sequence
		dx, dy := math.Cos(th), math.Sin(th)
		crossings, ok := 0, true
		for _, e := range all {
			if err := WallBudgetStep(budget); err != nil {
				return false, false, err
			}
			n, good := RayCrossings(e, px, py, dx, dy, k.Tol)
			if !good {
				ok = false
				break
			}
			crossings += n
		}
		if ok {
			return crossings%2 == 1, true, nil
		}
	}
	return false, false, nil
}

// RayCrossings counts proper crossings of the ray p + t·d (t > 0) with one
// element; good is false when a crossing is too ambiguous to count.
func RayCrossings(e SurveyElem, px, py, dx, dy, tol float64) (int, bool) {
	if e.Kind == SurveyLine {
		ex, ey := e.Bx-e.Ax, e.By-e.Ay
		seg := math.Hypot(ex, ey)
		det := dx*(-ey) - (-ex)*dy
		if math.Abs(det) <= 1e-12*math.Max(1, seg) {
			// Parallel: an overlap along the ray line is ambiguous; a clean
			// miss (the segment's line offset from the ray's) is a zero.
			if math.Abs((e.Ax-px)*dy-(e.Ay-py)*dx) <= tol {
				return 0, false
			}
			return 0, true
		}
		rx, ry := e.Ax-px, e.Ay-py
		t := (rx*(-ey) + ex*ry) / det
		u := (dx*ry - dy*rx) / det
		uTol := tol / seg
		if u < -uTol || u > 1+uTol {
			return 0, true
		}
		if t <= tol {
			if t > -tol {
				return 0, false // the boundary passes through the ray start
			}
			return 0, true
		}
		if u <= uTol || u >= 1-uTol {
			return 0, false // the ray grazes a segment endpoint
		}
		return 1, true
	}
	// Arc: |p + t·d − q|² = R².
	fx, fy := px-e.Qx, py-e.Qy
	b := fx*dx + fy*dy
	cc := fx*fx + fy*fy - e.Rr*e.Rr
	disc := b*b - cc
	if disc <= 0 {
		if disc > -tol*e.Rr {
			// Grazing tangency: ambiguous only when it happens ahead of us.
			if -b > 0 {
				return 0, false
			}
		}
		return 0, true
	}
	s := math.Sqrt(disc)
	n := 0
	for _, t := range []float64{-b - s, -b + s} {
		if t <= tol {
			if t > -tol {
				return 0, false
			}
			continue
		}
		x, y := px+t*dx, py+t*dy
		th := math.Atan2(y-e.Qy, x-e.Qx)
		if e.Closed {
			n++
			continue
		}
		lo, hi := e.ArcRange()
		off := Mod2pi(th - lo)
		ext := hi - lo
		angTol := tol / e.Rr
		if off <= angTol || math.Abs(off-ext) <= angTol || math.Abs(off-2*math.Pi) <= angTol {
			return 0, false
		}
		if off < ext {
			n++
		}
	}
	if math.Abs(s) <= tol {
		return 0, false
	}
	return n, true
}

// generate emits the closed-form candidate set: pair criticals (T2),
// angle-limit disks (T3), Apollonius triples (T4), concentric whole-arc
// disks, and — under a wedge — the wedge-tangent minima. Candidates are
// generated liberally; validate is what admits them. Each candidate is sent
// directly to visit so the cubic triple set is never materialized.
func (k *WallKernel) Generate(budget *proofbound.WorkBudget, visit func(DiskCand) error) error {
	var visitErr error
	add := func(x, y, r, rBound float64) {
		if visitErr == nil {
			visitErr = visit(DiskCand{X: x, Y: y, R: r, RBound: rBound})
		}
	}

	// Concentric whole-arc disks: the element's own radius, under the
	// element's own bound on it — zero for a stated or recorded radius, the
	// hypot bracket for an ArcSeg-derived one.
	for _, e := range k.Elems {
		if err := WallBudgetStep(budget); err != nil {
			return err
		}
		if e.Kind == SurveyArc && e.MatInside {
			add(e.Qx, e.Qy, e.Rr, e.RrBound)
			if visitErr != nil {
				return visitErr
			}
		}
	}

	// Pair criticals and angle limits.
	for i := range k.Elems {
		for j := i + 1; j < len(k.Elems); j++ {
			if err := WallBudgetStep(budget); err != nil {
				return err
			}
			k.PairCands(k.Elems[i], k.Elems[j], add)
			if visitErr != nil {
				return visitErr
			}
		}
	}
	for _, e := range k.Elems {
		for _, v := range k.Verts {
			if err := WallBudgetStep(budget); err != nil {
				return err
			}
			k.VertexElemCands(v, e, add)
			if visitErr != nil {
				return visitErr
			}
		}
	}
	for i := range k.Verts {
		for j := i + 1; j < len(k.Verts); j++ {
			if err := WallBudgetStep(budget); err != nil {
				return err
			}
			k.VertexVertexCands(k.Verts[i], k.Verts[j], add)
			if visitErr != nil {
				return visitErr
			}
		}
	}

	// Wedge-tangent minima (partial revolve only).
	if k.HasWedge() {
		if err := k.WedgeCands(budget, visit); err != nil {
			return err
		}
	}

	// Apollonius triples, the wedge included as an item.
	eqs, err := k.TripleEquations(budget)
	if err != nil {
		return err
	}
	for i := range eqs {
		for j := i + 1; j < len(eqs); j++ {
			for l := j + 1; l < len(eqs); l++ {
				if err := WallBudgetStep(budget); err != nil {
					return err
				}
				SolveTriple([3]CircEq{eqs[i], eqs[j], eqs[l]}, k.Scale, add)
				if visitErr != nil {
					return visitErr
				}
			}
		}
	}
	return nil
}
