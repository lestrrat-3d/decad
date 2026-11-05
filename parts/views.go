package parts

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Views is what a renderer needs to draw each part alone on a part sheet.
type Views struct{ Parts []PartView } // in Set order, body-less parts omitted

// PartView is one part posed for drawing.
type PartView struct {
	Part      string
	Placement r3.Transform // assembled coordinates to view coordinates
	Bounds    decad.Box    // the body's box in view coordinates
	Anchor    r3.Vec       // label anchor: the top centre of Bounds
	Ports     []Port       // where each joint attaches, in view coordinates
}

// Port is where one joint attaches to a part.
type Port struct {
	Joint string
	Other string // the part at the joint's other end
	Point r3.Vec // the middle of the engagement, or the contact face's centre, or a box fallback
	Dir   r3.Vec // the axis or the face's outward normal; zero when none is known
}

// Views returns the pose and joint ports of each part that has a body. A part
// is posed by the translation that puts its box centre at the origin, keeping
// the assembled orientation, or by its WithPartPose transform. The pose is for
// drawing, not for making: Views offers no "lay it flat" frame, because
// choosing the side a part rests on is a process decision.
//
// A port's Point is the middle of the matched or declared axis over the
// engagement, or the centre of the contact face for a FaceContact, or else the
// centre of the two parts' box overlap (the midpoint of the shortest segment
// between their boxes when they do not overlap), or the part's own box centre
// when the other part has no body. Dir is the axis, or the part's own face
// normal for a FaceContact; it is zero for the box fallback.
func (s *Set) Views(ctx context.Context, opts ...ViewOption) (*Views, error) {
	c, err := foldOptions(opts)
	if err != nil {
		return nil, err
	}
	if ctx == nil || s == nil || s.doc == nil {
		return nil, fmt.Errorf("parts: %w: nil context or set", decad.ErrDegenerate)
	}
	for name := range c.poses {
		if p, ok := s.index[name]; !ok || s.parts[p].Body == nil {
			return nil, fmt.Errorf("parts: %w: pose names %q, which is unknown or has no body", decad.ErrDegenerate, name)
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := s.live(); err != nil {
		return nil, err
	}
	fc := newFaceCache(false)
	out := &Views{}
	index := map[string]int{}
	for _, p := range s.parts {
		if p.Body == nil {
			continue
		}
		bf, err := fc.get(ctx, p.Body)
		if err != nil {
			return nil, err
		}
		pl, ok := c.poses[p.Name]
		if !ok {
			ctr := bf.box.Min.Add(bf.box.Max).Scale(0.5)
			if pl, err = r3.Translation(ctr.Scale(-1)); err != nil {
				return nil, fmt.Errorf("parts: part %q: %w", p.Name, err)
			}
		}
		bounds, err := viewBounds(ctx, p.Body, bf.box, pl)
		if err != nil {
			return nil, fmt.Errorf("parts: part %q: %w", p.Name, err)
		}
		anchor := r3.Vec{X: (bounds.Min.X + bounds.Max.X) / 2, Y: (bounds.Min.Y + bounds.Max.Y) / 2, Z: bounds.Max.Z}
		index[p.Name] = len(out.Parts)
		out.Parts = append(out.Parts, PartView{Part: p.Name, Placement: pl, Bounds: bounds, Anchor: anchor})
	}
	for _, j := range s.joints {
		r, err := s.readJoint(ctx, j, fc, c)
		if err != nil {
			return nil, err
		}
		for _, end := range []struct {
			name, other string
			b           bool
		}{{j.A, j.B, false}, {j.B, j.A, true}} {
			vi, ok := index[end.name]
			if !ok {
				continue
			}
			v := &out.Parts[vi]
			pt, dir := r.port, r.portD
			if _, otherHasBody := index[end.other]; !otherHasBody {
				bf, _ := fc.get(ctx, s.parts[s.index[end.name]].Body)
				pt, dir = bf.box.Min.Add(bf.box.Max).Scale(0.5), r3.Vec{}
			}
			if end.b && j.Kind == FaceContact {
				dir = dir.Scale(-1)
			}
			v.Ports = append(v.Ports, Port{Joint: j.Name, Other: end.other, Point: v.Placement.Apply(pt),
				Dir: v.Placement.ApplyDir(dir)})
		}
	}
	return out, nil
}

// viewBounds is the box of b moved by pl. A pure translation moves the body's
// own box exactly; any other placement boxes the mesh vertices, grown by the
// mesh bound and the rotation's rounding.
func viewBounds(ctx context.Context, b *decad.Body, box decad.Box, pl r3.Transform) (decad.Box, error) {
	basis := pl.Basis()
	if basis.EX == (r3.Vec{X: 1}) && basis.EY == (r3.Vec{Y: 1}) && basis.EZ == (r3.Vec{Z: 1}) {
		return decad.Box{Min: pl.Apply(box.Min), Max: pl.Apply(box.Max), Exactness: box.Exactness, Bound: box.Bound}, nil
	}
	mesh, _, _, err := meshOf(ctx, b, false)
	if err != nil {
		return decad.Box{}, err
	}
	lo := r3.Vec{X: math.Inf(1), Y: math.Inf(1), Z: math.Inf(1)}
	hi := r3.Vec{X: math.Inf(-1), Y: math.Inf(-1), Z: math.Inf(-1)}
	for _, v := range mesh.Vertices() {
		w := pl.Apply(v)
		lo = r3.Vec{X: math.Min(lo.X, w.X), Y: math.Min(lo.Y, w.Y), Z: math.Min(lo.Z, w.Z)}
		hi = r3.Vec{X: math.Max(hi.X, w.X), Y: math.Max(hi.Y, w.Y), Z: math.Max(hi.Z, w.Z)}
	}
	// The rotation rounds each coordinate by a few units in the last place of
	// the largest magnitude involved.
	extent := math.Max(math.Max(math.Abs(lo.X), math.Abs(hi.X)), math.Max(math.Max(math.Abs(lo.Y), math.Abs(hi.Y)),
		math.Max(math.Abs(lo.Z), math.Abs(hi.Z))))
	grow := mesh.Bound().Base() + box.Bound.Base() + 8*extent*0x1p-52
	g := r3.Vec{X: grow, Y: grow, Z: grow}
	return decad.Box{Min: lo.Sub(g), Max: hi.Add(g), Exactness: decad.Approximate, Bound: units.Millimeters(grow)}, nil
}
