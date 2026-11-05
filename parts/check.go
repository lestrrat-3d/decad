package parts

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/explodeplan"
	"github.com/lestrrat-3d/decad/internal/jointcheck"
	"github.com/lestrrat-3d/decad/internal/jointinfer"
	"github.com/lestrrat-3d/units"
)

// Check judges every declared joint against the faces and bounds of its two
// parts, and lists the pairs of parts whose boxes meet with no joint naming
// them (agent-docs/parts-design.md §7.2). It is a structural check: it never
// calls Document.Verify, and it does not prove that two parts' interiors are
// disjoint. A Contradicted joint is a finding, not an error.
//
// Until the contact witnesses land, Contact and Keepout joints are NotChecked,
// as a Tie is.
func (s *Set) Check(ctx context.Context, opts ...CheckOption) (*JointReport, error) {
	c, err := foldOptions(opts)
	if err != nil {
		return nil, err
	}
	if ctx == nil || s == nil || s.doc == nil {
		return nil, fmt.Errorf("parts: %w: nil context or set", decad.ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := s.live(); err != nil {
		return nil, err
	}
	fc := newFaceCache(true)
	faces := map[string]*bodyFaces{}
	rep := &JointReport{Request: CheckRequest{
		FitTolerance: units.Millimeters(c.fitTol), ContactTolerance: units.Millimeters(c.contactTol),
		AxisTolerance: units.Radians(c.axisTol), MaxFit: units.Millimeters(c.maxFit)}}
	for _, p := range s.parts {
		if p.Body == nil {
			continue
		}
		bf, err := fc.get(ctx, p.Body)
		if err != nil {
			return nil, fmt.Errorf("parts: part %q: %w", p.Name, err)
		}
		faces[p.Name] = bf
		rep.Request.Meshes = append(rep.Request.Meshes, PartMesh{Part: p.Name, Tolerance: units.Millimeters(bf.meshTol),
			Bound: units.Millimeters(bf.meshBound), Verified: bf.verified})
	}
	for _, j := range s.joints {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		rep.Joints = append(rep.Joints, s.checkJoint(j, faces[j.A], faces[j.B], c))
	}
	rep.Unexplained = s.unexplained(faces, c)
	return rep, nil
}

func (s *Set) checkJoint(j Joint, a, b *bodyFaces, c config) JointResult {
	r := JointResult{Joint: j}
	switch j.Kind {
	case PinInBore, PinInSlot:
		s.checkPin(&r, a, b, c)
	case FaceContact:
		checkFace(&r, a, b, c)
	default:
		r.Status = NotChecked
	}
	return r
}

func partBox(bf *bodyFaces) jointcheck.Box {
	return jointcheck.Box{Min: bf.box.Min, Max: bf.box.Max}
}

// boxGapLow is G1: the lower end of the gap between the two parts.
func boxGapLow(a, b *bodyFaces) float64 {
	return jointcheck.GapLow(partBox(a), partBox(b), a.box.Bound.Base()+b.box.Bound.Base())
}

// unreadNear reports whether a face that reads as neither a plane nor a
// cylinder lies near the other part (G3), or a body would not mesh.
func unreadNear(a, b *bodyFaces, maxFit float64) bool {
	if a.unmeshed || b.unmeshed {
		return true
	}
	near := func(faces []explodeplan.Box, other jointcheck.Box) bool {
		for _, f := range faces {
			if (jointcheck.Box{Min: f.Min, Max: f.Max}).Grow(maxFit).Meets(other) {
				return true
			}
		}
		return false
	}
	return near(a.unread, partBox(b)) || near(b.unread, partBox(a))
}

func measurement(v, bound float64) *decad.Measurement {
	ex := decad.Approximate
	if bound == 0 {
		ex = decad.Exact
	}
	return &decad.Measurement{Value: units.Millimeters(v), Exactness: ex, Bound: units.Millimeters(bound)}
}

func (r *JointResult) settlef(status JointStatus, cause Cause, format string, args ...any) {
	r.Status, r.Cause = status, cause
	r.Reason = fmt.Sprintf(format, args...)
}

// checkPin is J1 and J2.
func (s *Set) checkPin(r *JointResult, a, b *bodyFaces, c config) {
	j := r.Joint
	if low := boxGapLow(a, b); low > c.maxFit {
		r.settlef(Contradicted, CauseApart, "the parts' boxes are at least %.3g mm apart, more than the %.3g mm a joint may leave",
			low, c.maxFit)
		s.declaredAxis(r)
		return
	}
	params := jointinfer.Params{MaxFit: c.maxFit, FitTol: c.fitTol, AxisTol: c.axisTol}
	var (
		line      jointinfer.Line
		length    float64
		clearance float64
		bound     float64
		matched   bool
	)
	switch j.Kind {
	case PinInBore:
		if ms := jointinfer.MatchBore(a.cyls, b.cyls, params); len(ms) > 0 {
			m := ms[0]
			for _, i := range m.Pin {
				r.FacesA = append(r.FacesA, a.cylFaces[i])
			}
			for _, i := range m.Bore {
				r.FacesB = append(r.FacesB, b.cylFaces[i])
			}
			line, length, clearance, bound, matched = m.Axis, m.Length, m.Clearance, m.Bound, true
		}
	case PinInSlot:
		if ms := jointinfer.MatchSlot(a.cyls, b.planes, params); len(ms) > 0 {
			m := ms[0]
			r.FacesA = []*decad.Face{a.cylFaces[m.Pin]}
			r.FacesB = []*decad.Face{b.planeFaces[m.Walls[0]], b.planeFaces[m.Walls[1]]}
			line, length, clearance, bound, matched = m.Axis, m.Length, m.Clearance, m.Bound, true
		}
	}
	if !matched {
		s.declaredAxis(r)
		if unreadNear(a, b, c.maxFit) {
			r.settlef(Undecided, CauseUnreadFace, "a face near the pair reads as neither a plane nor a cylinder, or a body would not mesh")
			return
		}
		r.settlef(Contradicted, CauseNoMatch, "%s", noMatchReason(j.Kind, a, b))
		return
	}
	ax := Axis{Point: line.Point, Dir: line.Dir}
	r.Axis, r.AxisFrom = &ax, AxisSourceFaces
	r.Measured, r.Engaged = measurement(clearance, bound), measurement(length, bound)

	var contradicted, undecided bool
	note := func(v jointcheck.Verdict, cause Cause, format string, args ...any) {
		switch v {
		case jointcheck.Outside:
			if !contradicted {
				r.settlef(Contradicted, cause, format, args...)
			}
			contradicted = true
		case jointcheck.Straddle:
			if !contradicted && !undecided {
				r.settlef(Undecided, cause, format, args...)
			}
			undecided = true
		case jointcheck.Inside:
		}
	}
	// Engaged: the axial overlap must be positive at its lower end.
	if length-bound <= 0 {
		note(jointcheck.Straddle, CauseNoMatch, "the engagement of %.3g mm is within its bound of %.3g mm", length, bound)
	}
	if want := j.Clearance; want != (units.Value{}) {
		w := want.Base()
		note(jointcheck.Within(clearance, bound, w-c.fitTol, w+c.fitTol), CauseClearance,
			"measured clearance %.3g mm against the declared %.3g mm (tolerance %.3g mm)", clearance, w, c.fitTol)
	} else {
		note(jointcheck.Within(clearance, bound, 0, math.Inf(1)), CauseInterference,
			"the pin is %.3g mm larger than its bore and no press fit is declared", -clearance)
	}
	if j.Axis != nil {
		angle, dist := jointcheck.LineOffset(jointcheck.Line{Point: j.Axis.Point, Dir: j.Axis.Dir},
			jointcheck.Line{Point: line.Point, Dir: line.Dir})
		v := jointcheck.Within(angle, 0, 0, c.axisTol)
		if d := jointcheck.Within(dist, bound, 0, c.fitTol); d > v {
			v = d
		}
		note(v, CauseAxis, "the declared axis is %.3g degrees and %.3g mm from the matched one", angle*180/math.Pi, dist)
	}
	switch {
	case contradicted:
		r.Status = Contradicted
	case undecided:
		r.Status = Undecided
	default:
		r.Status, r.Cause, r.Reason = Confirmed, CauseNone, ""
	}
}

// declaredAxis reports the declared axis when the faces gave none.
func (s *Set) declaredAxis(r *JointResult) {
	if r.Joint.Axis != nil {
		ax := *r.Joint.Axis
		r.Axis, r.AxisFrom = &ax, AxisSourceDeclared
	}
}

// noMatchReason names the closest candidate faces, so a person can see why no
// match was made.
func noMatchReason(kind JointKind, a, b *bodyFaces) string {
	if kind == PinInSlot {
		return "no pin between parallel walls"
	}
	var pins, bores int
	best, bestD := "", math.Inf(1)
	for _, p := range a.cyls {
		if !p.Convex {
			continue
		}
		pins++
		for _, q := range b.cyls {
			if q.Convex {
				continue
			}
			bores++
			d := q.Origin.Sub(p.Origin).Cross(p.Axis).Len()
			if d < bestD {
				bestD = d
				best = fmt.Sprintf("closest pair: pin radius %.3g mm, bore radius %.3g mm, %.3g mm off axis", p.Radius, q.Radius, d)
			}
		}
	}
	switch {
	case pins == 0:
		return "no coaxial cylinder pair: the pin part has no convex cylinder face"
	case bores == 0:
		return "no coaxial cylinder pair: the bore part has no concave cylinder face"
	}
	return "no coaxial cylinder pair; " + best
}

// checkFace is J3.
func checkFace(r *JointResult, a, b *bodyFaces, c config) {
	if low := boxGapLow(a, b); low > c.maxFit {
		r.settlef(Contradicted, CauseApart, "the parts' boxes are at least %.3g mm apart, more than the %.3g mm a joint may leave",
			low, c.maxFit)
		return
	}
	params := jointinfer.Params{MaxFit: c.maxFit, FitTol: c.fitTol, AxisTol: c.axisTol}
	ms := jointinfer.MatchFace(a.planes, b.planes, params)
	var coincident, straddle bool
	nearest := math.Inf(1)
	for _, m := range ms {
		bound := a.planes[m.A].Bound + b.planes[m.B].Bound
		switch jointcheck.Within(m.Gap, bound, 0, c.fitTol) {
		case jointcheck.Outside:
			nearest = math.Min(nearest, m.Gap)
			continue
		case jointcheck.Straddle:
			straddle = true
			nearest = math.Min(nearest, m.Gap)
			continue
		case jointcheck.Inside:
		}
		coincident = true
		w, ok := jointcheck.OverlapWitness(a.planeTris[m.A], b.planeTris[m.B], m.Normal, a.meshBound+b.meshBound)
		if !ok {
			continue
		}
		r.FacesA, r.FacesB = []*decad.Face{a.planeFaces[m.A]}, []*decad.Face{b.planeFaces[m.B]}
		r.Measured, r.Witness = measurement(m.Gap, bound), &w
		r.Status, r.Cause, r.Reason = Confirmed, CauseNone, ""
		return
	}
	switch {
	case unreadNear(a, b, c.maxFit):
		r.settlef(Undecided, CauseUnreadFace, "a face near the pair reads as neither a plane nor a cylinder, or a body would not mesh")
	case coincident:
		r.settlef(Undecided, CauseNoWitness, "opposed planes coincide but no triangle shows shared area")
	case straddle:
		r.settlef(Undecided, CauseApart, "the planes' distance of %.3g mm straddles the %.3g mm fit tolerance", nearest, c.fitTol)
	case len(ms) > 0:
		r.settlef(Contradicted, CauseApart, "the nearest opposed planes are %.3g mm apart, more than the %.3g mm fit tolerance",
			nearest, c.fitTol)
	default:
		r.settlef(Contradicted, CauseNoMatch, "no opposed planar faces of the two parts overlap")
	}
}

// unexplained is U1: the pairs of parts with bodies whose boxes, grown by the
// contact tolerance and their bounds, meet, and that no joint names.
func (s *Set) unexplained(faces map[string]*bodyFaces, c config) []PairFinding {
	named := map[[2]string]struct{}{}
	for _, j := range s.joints {
		named[[2]string{j.A, j.B}] = struct{}{}
		named[[2]string{j.B, j.A}] = struct{}{}
	}
	var out []PairFinding
	for i, p := range s.parts {
		a, ok := faces[p.Name]
		if !ok {
			continue
		}
		for _, q := range s.parts[i+1:] {
			b, ok := faces[q.Name]
			if !ok {
				continue
			}
			if _, joined := named[[2]string{p.Name, q.Name}]; joined {
				continue
			}
			ba, bb := a.box.Bound.Base(), b.box.Bound.Base()
			o, meets := partBox(a).Grow(c.contactTol + ba).Overlap(partBox(b).Grow(c.contactTol + bb))
			if !meets {
				continue
			}
			ex := decad.Approximate
			if ba+bb == 0 && a.box.Exactness == decad.Exact && b.box.Exactness == decad.Exact {
				ex = decad.Exact
			}
			out = append(out, PairFinding{A: p.Name, B: q.Name, Overlap: decad.Box{Min: o.Min, Max: o.Max,
				Exactness: ex, Bound: units.Millimeters(ba + bb)}})
		}
	}
	return out
}
