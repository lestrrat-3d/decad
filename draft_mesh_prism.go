package decad

import (
	"context"
	"errors"

	"github.com/lestrrat-3d/decad/internal/orthoshell"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// draftMeshPrism drafts every wall of an exact-vertex faceted solid that is a straight prism along a world axis, about
// one of its two caps (agent-docs/orthogonal-boolean-design.md B21): the draft of all the walls of a prism is the body
// a tapered Extrude builds from the same section and sweep, so the section is read off the solid, exactly, on the grid
// of its coordinates, and swept from the neutral cap to the other with WithTaper. The solid qualifies when it holds
// two distinct coordinates along the axis, the neutral face is a retained plane square to that axis at one of them, the
// selection is exactly the faces that are not the two caps, and the section is one loop without a hole. handled is
// false, with no error, when the route does not apply, and the caller keeps its refusal.
func draftMeshPrism(ctx context.Context, d *Document, b *Body, neutral *Face, selected []*Face,
	angle units.Value) (result *Body, handled bool, err error) {
	payload, ok := orthogonalOperand(b)
	if !ok || neutral == nil {
		return nil, false, nil
	}
	plane, ok := neutral.Surface().(Plane)
	if !ok {
		return nil, false, nil
	}
	axis, _, ok := unitAxis(plane.Frame.N())
	if !ok {
		return nil, false, nil
	}
	coord := func(v r3.Vec) float64 { return [3]float64{v.X, v.Y, v.Z}[axis] }
	lo, hi := coord(payload.Verts[0]), coord(payload.Verts[0])
	for _, v := range payload.Verts {
		c := coord(v)
		switch {
		case c == lo || c == hi:
		case lo == hi:
			lo, hi = min(lo, c), max(lo, c)
		default:
			return nil, false, nil // a third level along the axis: not a straight prism
		}
	}
	level := coord(plane.Frame.Origin())
	if lo == hi || (level != lo && level != hi) {
		return nil, false, nil
	}
	// Every face is a cap or a wall, and the selection is the walls.
	caps := 0
	walls := map[*Face]bool{}
	for _, f := range b.Faces() {
		p, isPlane := f.Surface().(Plane)
		if isPlane {
			if a, _, square := unitAxis(p.Frame.N()); square && a == axis {
				caps++
				continue
			}
		}
		walls[f] = true
	}
	if caps != 2 || len(selected) != len(walls) {
		return nil, false, nil
	}
	for _, f := range selected {
		if !walls[f] {
			return nil, false, nil
		}
		delete(walls, f)
	}
	sweep := 1.0
	if level == hi {
		sweep = -1
	}
	a1, a2 := (axis+1)%3, (axis+2)%3
	uAxis, vAxis := a1, a2
	if sweep < 0 {
		uAxis, vAxis = a2, a1
	}
	loops, err := orthoshell.SectionLoops(payload.Verts, payload.Tris, axis, (lo+hi)/2, uAxis, vAxis)
	if err != nil {
		if errors.Is(err, orthoshell.ErrUnsupported) {
			return nil, false, nil
		}
		return nil, false, err
	}
	if len(loops) != 1 || len(loops[0]) < 3 {
		return nil, false, nil
	}
	origin := [3]float64{}
	origin[axis] = level
	axes := func(i int) r3.Vec { return unitVec(i, 1) }
	frame, err := r3.NewFrame(r3.NewVec(origin[0], origin[1], origin[2]), axes(uAxis), axes(vAxis))
	if err != nil {
		return nil, false, nil //nolint:nilerr // a frame the route cannot build leaves the caller's refusal
	}
	world := sketch.NewWorld()
	pl, err := world.CreatePlaneFromFrame(frame)
	if err != nil {
		return nil, false, nil //nolint:nilerr // a plane the route cannot build leaves the caller's refusal
	}
	s, err := world.CreateSketch(pl)
	if err != nil {
		return nil, false, nil //nolint:nilerr // a sketch the route cannot build leaves the caller's refusal
	}
	points := make([]*sketch.Point, len(loops[0]))
	for i, p := range loops[0] {
		points[i] = s.CreatePoint(p[0], p[1])
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	if _, err := s.Solve(ctx); err != nil || len(s.Profiles()) != 1 {
		return nil, false, nil //nolint:nilerr // a section that does not solve to one profile leaves the caller's refusal
	}
	out, err := Extrude(ctx, d, s, s.Profiles()[0], Distance{D: units.Millimeters(hi - lo), Dir: Along}, WithTaper(angle))
	if err != nil {
		return nil, false, nil //nolint:nilerr // an extrude the route cannot build leaves the caller's refusal
	}
	removeTools([]*Body{b})
	return out, true, nil
}
