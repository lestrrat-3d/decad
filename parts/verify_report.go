package parts

import (
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
)

// pairKey names an ordered pair of bodies; a view stores both orders.
type pairKey struct{ a, b *decad.Body }

func keyOf(a, b *decad.Body) pairKey { return pairKey{a, b} }

// verifyView is a Verify report read as the relation of each pair of bodies.
type verifyView struct {
	overlap map[pairKey]struct{}
	gap     map[pairKey]decad.Measurement
}

func newVerifyView(r *decad.Report) verifyView {
	v := verifyView{overlap: map[pairKey]struct{}{}, gap: map[pairKey]decad.Measurement{}}
	for _, i := range r.Interferences {
		v.overlap[keyOf(i.A, i.B)], v.overlap[keyOf(i.B, i.A)] = struct{}{}, struct{}{}
	}
	for _, c := range r.Clearances {
		v.gap[keyOf(c.A, c.B)], v.gap[keyOf(c.B, c.A)] = c.Gap, c.Gap
	}
	return v
}

// relation is R1: Overlapping for an interference row, Touching for a clearance
// row whose gap is an Exact zero, Separated for another clearance row, and
// Undecided for a pair in neither list. The gap is returned for a clearance row.
func (v verifyView) relation(a, b *decad.Body) (decad.ContactRelation, *decad.Measurement) {
	k := keyOf(a, b)
	if _, ok := v.overlap[k]; ok {
		return decad.ContactOverlapping, nil
	}
	if g, ok := v.gap[k]; ok {
		if g.Exactness == decad.Exact && g.Value.Base() == 0 {
			return decad.ContactTouching, &g
		}
		return decad.ContactSeparated, &g
	}
	return decad.ContactUndecided, nil
}

// checkVerifyReport is R3: a report of another document, or one that does not
// name every body of the Set, is decad.ErrDegenerate.
func (s *Set) checkVerifyReport(r *decad.Report) error {
	named := map[*decad.Body]struct{}{}
	for _, b := range r.Bodies {
		if b.Body.Document() != s.doc {
			return fmt.Errorf("parts: %w: the Verify report is of another document", decad.ErrDegenerate)
		}
		named[b.Body] = struct{}{}
	}
	for _, p := range s.parts {
		if p.Body == nil {
			continue
		}
		if _, ok := named[p.Body]; !ok {
			return fmt.Errorf("parts: %w: the Verify report does not name the body of part %q", decad.ErrDegenerate, p.Name)
		}
	}
	return nil
}

// applyVerify is R1 and R2: it records each pair's relation, and lets the
// report settle what the structural rules left open or contradict what they
// confirmed. It never settles a Tie, which is not checked.
func (s *Set) applyVerify(rep *JointReport, c config) {
	v := newVerifyView(c.verify)
	for i := range rep.Joints {
		r := &rep.Joints[i]
		j := r.Joint
		a, b := s.parts[s.index[j.A]].Body, s.parts[s.index[j.B]].Body
		if a == nil || b == nil {
			continue
		}
		rel, gap := v.relation(a, b)
		r.Verified = &rel
		press := j.Kind == PinInBore && j.Clearance != (units.Value{}) && j.Clearance.Base() < 0
		switch {
		case rel == decad.ContactOverlapping && j.Kind != Tie && !press:
			r.Status, r.Cause = Contradicted, CauseInterference
			r.Reason = "the Verify report shows the parts overlapping"
		case j.Kind == Keepout && r.Status == Undecided && rel == decad.ContactSeparated &&
			gap.Value.Base()-gap.Bound.Base() >= j.MinGap.Base():
			r.Status, r.Cause, r.Reason = Confirmed, CauseNone, ""
		case j.Kind == Contact && r.Status == Undecided && (rel == decad.ContactTouching ||
			(rel == decad.ContactSeparated && gap.Value.Base()+gap.Bound.Base() <= c.contactTol)):
			r.Status, r.Cause, r.Reason = Confirmed, CauseNone, ""
		}
	}
	kept := rep.Unexplained[:0]
	for _, p := range rep.Unexplained {
		a, b := s.parts[s.index[p.A]].Body, s.parts[s.index[p.B]].Body
		rel, gap := v.relation(a, b)
		p.Verified = &rel
		if rel == decad.ContactSeparated && gap.Value.Base()-gap.Bound.Base() > c.contactTol {
			continue
		}
		kept = append(kept, p)
	}
	rep.Unexplained = kept
}
