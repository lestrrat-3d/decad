package parts

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/jointinfer"
	"github.com/lestrrat-3d/units"
)

// InferredJoint is a joint the geometry suggests. Infer and JointReport.Inferred
// return them; they are never added to the Set.
type InferredJoint struct {
	Joint    Joint // Kind PinInBore or PinInSlot, Motion Fixed, Axis and Clearance as matched
	Measured decad.Measurement
	FacesA   []*decad.Face
	FacesB   []*decad.Face
}

// Infer lists the pin-in-bore and pin-in-slot joints the faces suggest between
// the parts (agent-docs/parts-design.md §7.3). For every pair of parts with
// bodies whose boxes, grown by MaxFit, meet, it runs the matching of §7.1 in both
// directions and returns each match, longest engagement first within a pair,
// with the clearance it measured. A match is a convex cylinder face of one part
// in a coaxial concave one of the other, or between two parallel planar faces; a
// face that reads as neither a plane nor a cylinder never matches. Joint names
// are "A~B", with "#2", "#3" for further matches of one pair. Infer adds nothing
// to the Set. Coplanar face contacts are not inferred.
func (s *Set) Infer(ctx context.Context, opts ...CheckOption) ([]InferredJoint, error) {
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
	fc := newFaceCache(false)
	faces := map[string]*bodyFaces{}
	for _, p := range s.parts {
		if p.Body == nil {
			continue
		}
		bf, err := fc.get(ctx, p.Body)
		if err != nil {
			return nil, fmt.Errorf("parts: part %q: %w", p.Name, err)
		}
		faces[p.Name] = bf
	}
	return s.infer(faces, c), nil
}

// infer runs the matching over every candidate pair, in Set order.
func (s *Set) infer(faces map[string]*bodyFaces, c config) []InferredJoint {
	params := jointinfer.Params{MaxFit: c.maxFit, FitTol: c.fitTol, AxisTol: c.axisTol}
	var out []InferredJoint
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
			if !partBox(a).Grow(c.maxFit).Meets(partBox(b)) { // M1
				continue
			}
			var pair []InferredJoint
			for _, dir := range []struct {
				pin, recv   string
				pinF, recvF *bodyFaces
			}{{p.Name, q.Name, a, b}, {q.Name, p.Name, b, a}} {
				pair = append(pair, inferBores(dir.pin, dir.recv, dir.pinF, dir.recvF, params)...)
				pair = append(pair, inferSlots(dir.pin, dir.recv, dir.pinF, dir.recvF, params)...)
			}
			for k := range pair {
				if k > 0 {
					pair[k].Joint.Name = fmt.Sprintf("%s#%d", pair[0].Joint.Name, k+1)
				}
			}
			out = append(out, pair...)
		}
	}
	return out
}

func inferBores(pin, recv string, pf, rf *bodyFaces, params jointinfer.Params) []InferredJoint {
	var out []InferredJoint
	for _, m := range jointinfer.MatchBore(pf.cyls, rf.cyls, params) {
		ij := InferredJoint{Joint: Joint{Name: pin + "~" + recv, Kind: PinInBore, A: pin, B: recv,
			Axis: &Axis{Point: m.Axis.Point, Dir: m.Axis.Dir}, Clearance: units.Millimeters(m.Clearance)},
			Measured: *measurement(m.Clearance, m.Bound)}
		for _, i := range m.Pin {
			ij.FacesA = append(ij.FacesA, pf.cylFaces[i])
		}
		for _, i := range m.Bore {
			ij.FacesB = append(ij.FacesB, rf.cylFaces[i])
		}
		out = append(out, ij)
	}
	return out
}

func inferSlots(pin, recv string, pf, rf *bodyFaces, params jointinfer.Params) []InferredJoint {
	var out []InferredJoint
	for _, m := range jointinfer.MatchSlot(pf.cyls, rf.planes, params) {
		out = append(out, InferredJoint{Joint: Joint{Name: pin + "~" + recv, Kind: PinInSlot, A: pin, B: recv,
			Axis: &Axis{Point: m.Axis.Point, Dir: m.Axis.Dir}, Clearance: units.Millimeters(m.Clearance)},
			Measured: *measurement(m.Clearance, m.Bound), FacesA: []*decad.Face{pf.cylFaces[m.Pin]},
			FacesB: []*decad.Face{rf.planeFaces[m.Walls[0]], rf.planeFaces[m.Walls[1]]}})
	}
	return out
}
