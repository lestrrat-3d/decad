package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/orthoshell"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// draftMeshPrism drafts every wall of an exact-vertex faceted solid that is a straight prism along a world axis, about
// one of its two caps (agent-docs/orthogonal-design.md B21): the draft of all the walls of a prism is the body
// a tapered Extrude builds from the same section and sweep, so the section is read off the solid, exactly, on the grid
// of its coordinates, and swept from the neutral cap to the other with Taper. The solid qualifies when it holds
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
	level := coord(plane.Frame.Origin())
	faces := b.Faces()
	capFaces := make([]bool, len(faces))
	faceIndexes := make(map[*Face]int, len(faces))
	for i, f := range faces {
		faceIndexes[f] = i
		p, isPlane := f.Surface().(Plane)
		if isPlane {
			if a, _, square := unitAxis(p.Frame.N()); square && a == axis {
				capFaces[i] = true
			}
		}
	}
	selectedFaces := make([]int, len(selected))
	for i, f := range selected {
		index, exists := faceIndexes[f]
		if !exists {
			selectedFaces[i] = -1
		} else {
			selectedFaces[i] = index
		}
	}
	prepared, ok, err := orthoshell.PrepareDraftPrism(
		payload.Verts, payload.Tris, axis, level, capFaces, selectedFaces)
	if err != nil {
		return nil, false, err
	}
	if !ok {
		return nil, false, nil
	}
	origin := [3]float64{}
	origin[axis] = level
	axes := func(i int) r3.Vec { return unitVec(i, 1) }
	frame, err := r3.NewFrame(r3.NewVec(origin[0], origin[1], origin[2]),
		axes(prepared.UAxis), axes(prepared.VAxis))
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
	points := make([]*sketch.Point, len(prepared.Loop))
	for i, p := range prepared.Loop {
		points[i] = s.CreatePoint(p[0], p[1])
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	if _, err := s.Solve(ctx); err != nil {
		return nil, false, nil //nolint:nilerr // a section that does not solve to one profile leaves the caller's refusal
	}
	profiles := s.Profiles()
	if len(profiles) != 1 {
		return nil, false, nil
	}
	// A step may retire only a body it built, so a refusal comes before the tool is made.
	if _, err := d.stepGate(ctx, nil, []*Body{b}); err != nil {
		return nil, false, err
	}
	out, err := extrudeTool(ctx, d, s, profiles[0],
		Distance{D: units.Millimeters(prepared.Distance), Dir: Along}, withTaper(angle, ""))
	if err != nil {
		return nil, false, nil //nolint:nilerr // an extrude the route cannot build leaves the caller's refusal
	}
	if err := removeConsumed(ctx, b); err != nil {
		return nil, false, err
	}
	return out, true, nil
}
