// Package holeplan turns a hole request into the plain-data tool shapes that
// decad.Hole extrudes and revolves (agent-docs/hole-design.md). It holds the
// arithmetic of the placement and the size gates and touches no body,
// document or sketch.
package holeplan

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
)

// HeadKind names the widening at the mouth of a hole.
type HeadKind uint8

const (
	// HeadNone is a plain hole.
	HeadNone HeadKind = iota
	// HeadCounterbore is a flat-bottomed cylindrical recess.
	HeadCounterbore
	// HeadCountersink is a conical recess.
	HeadCountersink
)

// OnPlaneTol is the largest distance, in millimetres, between the requested
// hole centre and the face plane that Compute accepts. A centre farther away is
// a point that names no spot on the face, so it is refused rather than moved.
const OnPlaneTol = 1e-6

// MouthMargin is how far, in millimetres, every tool starts outside the face,
// so no tool face lies in the face plane (a tangent contact the boolean
// refuses).
const MouthMargin = 1

// sinkOvershoot is how far, as a fraction of the sink diameter, the countersink
// cone continues above the face so its base never lies in the face plane.
const sinkOvershoot = 0.5

// Head is the mouth widening of a hole. Diameter, Depth and Angle are
// millimetres and radians; a counterbore reads Diameter and Depth, a
// countersink reads Diameter and Angle (the included angle of the cone).
type Head struct {
	Kind     HeadKind
	Diameter float64
	Depth    float64
	Angle    float64
}

// Rect is a rectangular bore in millimetres and radians. Angle turns it
// counter-clockwise about the hole axis, measured from the frame's U axis.
type Rect struct {
	Width, Height, Angle float64
}

// Slot is a stadium-shaped bore in millimetres and radians: a Width-wide strip
// with a half circle on each end, Length long end to end. Angle turns it
// counter-clockwise about the hole axis, measured from the frame's U axis.
type Slot struct {
	Length, Width, Angle float64
}

// Input is the request in millimetres and radians. Frame is the face's plane
// frame and Outward the face's outward unit normal, which equals Frame.N() or
// its negation. Reach is the distance from the face plane to the far side of
// the body along the inward normal, read from the body's bounds.
type Input struct {
	Frame    r3.Frame
	Outward  r3.Vec
	At       r3.Vec
	Diameter float64
	Through  bool
	Depth    float64
	Reach    float64
	Head     Head
	// Margin is how far the tools start outside the face, in millimetres. Zero
	// means MouthMargin; a hole on a concave cylinder sets it to CurvedMargin.
	Margin float64
	// Rect, when set, replaces the round bore: Diameter is then ignored and
	// Head must be HeadNone.
	Rect *Rect
	// Slot, when set, replaces the round bore like Rect. Rect and Slot are
	// not both set.
	Slot *Slot
}

// SlotTool is a stadium bore in the sketch plane's coordinates. Start and End
// are the centres of the two end half circles, P0 to P3 the straight sides'
// ends in counter-clockwise order: P0 to P1 is one side, an arc about End runs
// from P1 to P2, P2 to P3 is the other side, and an arc about Start runs from
// P3 to P0.
type SlotTool struct {
	Start, End     [2]float64
	P0, P1, P2, P3 [2]float64
}

// Cylinder is a hole tool swept inward. Length is the depth below the face
// the cylinder reaches; the tool itself starts Margin outside the face and so
// runs Tool() long.
type Cylinder struct {
	Radius float64
	Length float64
	Margin float64
}

// Tool is the length of the extruded tool.
func (c Cylinder) Tool() float64 { return c.Length + c.Margin }

// CurvedMargin returns how far a tool must start outside the tangent plane at
// the hole axis on a cylindrical face of the given radius so that it starts
// clear of the face everywhere its section reaches. toolRadius is the radius of
// the circle that holds the tool's section. A convex face falls away from the
// tangent plane, so MouthMargin serves. A concave face rises toward it by the
// sagitta R - sqrt(R² - r²) at distance r, which the tool must clear as well; a
// tool section reaching the axis of a concave face is refused.
func CurvedMargin(radius, toolRadius float64, concave bool) (float64, error) {
	if !positive(radius) || !positive(toolRadius) {
		return 0, fmt.Errorf(`%w: the cylinder and tool radii must be positive and finite`, decaderr.ErrDegenerate)
	}
	if !concave {
		return MouthMargin, nil
	}
	if toolRadius >= radius {
		return 0, fmt.Errorf(`%w: a hole of radius %g mm reaches the axis of the %g mm radius bore it starts in`,
			decaderr.ErrUnsupported, toolRadius, radius)
	}
	sag := radius - math.Sqrt(radius*radius-toolRadius*toolRadius)
	return MouthMargin + sag, nil
}

// Sink is the revolve profile of a countersink tool. Its sketch plane holds
// the hole axis; Radial and Height are measured from the axis point on the
// face plane, with Height positive outward. The polygon runs from the axis at
// the cone's bottom out to the cone at the overshoot height and back to the
// axis.
type Sink struct {
	Plane   r3.Frame
	Profile [4][2]float64
}

// Plan is the tool shapes of one hole.
type Plan struct {
	// Plane is the face frame moved MouthMargin outward, where the cylinder
	// sketches lie.
	Plane r3.Frame
	// U and V are the hole axis in Plane's coordinates.
	U, V float64
	// Against reports that the inward direction is against Plane's normal.
	Against bool
	// Bore is the main drill.
	Bore Cylinder
	// Corners, when set, are the four corners of the rectangular bore in
	// Plane's coordinates, counter-clockwise. Bore.Radius is then unused.
	Corners *[4][2]float64
	// SlotTool, when set, is the stadium bore. Bore.Radius is then unused.
	SlotTool *SlotTool
	// Counterbore is the recess cylinder, set when the head is a counterbore.
	Counterbore *Cylinder
	// Sink is the countersink cone, set when the head is a countersink.
	Sink *Sink
}

// Compute checks the request and returns its tools.
func Compute(in Input) (Plan, error) {
	if in.Rect != nil || in.Slot != nil {
		return computeShape(in)
	}
	if !positive(in.Diameter) {
		return Plan{}, fmt.Errorf(`%w: the hole diameter must be positive and finite`, decaderr.ErrDegenerate)
	}
	if !positive(in.Reach) {
		return Plan{}, fmt.Errorf(`%w: the body has no material behind the face`, decaderr.ErrDegenerate)
	}
	local := in.Frame.ToLocal(in.At)
	if math.Abs(local.Z) > OnPlaneTol {
		return Plan{}, fmt.Errorf(`%w: the hole centre lies %g mm off the face plane`, decaderr.ErrDegenerate, local.Z)
	}
	boreLength := in.Reach
	if !in.Through {
		if !positive(in.Depth) {
			return Plan{}, fmt.Errorf(`%w: a blind hole needs a positive finite depth`, decaderr.ErrDegenerate)
		}
		boreLength = in.Depth
	}
	margin := in.Margin
	if margin == 0 {
		margin = MouthMargin
	}
	if !positive(margin) {
		return Plan{}, fmt.Errorf(`%w: the tool margin must be positive and finite`, decaderr.ErrDegenerate)
	}
	mouth, err := r3.NewFrame(in.Frame.Origin().Add(in.Outward.Scale(margin)), in.Frame.U(), in.Frame.V())
	if err != nil {
		return Plan{}, fmt.Errorf(`%w: the face frame is degenerate: %s`, decaderr.ErrDegenerate, err)
	}
	radius := in.Diameter / 2
	plan := Plan{
		Plane:   mouth,
		U:       local.X,
		V:       local.Y,
		Against: in.Outward.Dot(in.Frame.N()) > 0,
		Bore:    Cylinder{Radius: radius, Length: boreLength, Margin: margin},
	}
	switch in.Head.Kind {
	case HeadNone:
		return plan, nil
	case HeadCounterbore:
		return plan.withCounterbore(in)
	case HeadCountersink:
		return plan.withSink(in)
	default:
		return Plan{}, fmt.Errorf(`%w: unknown hole head %d`, decaderr.ErrDegenerate, in.Head.Kind)
	}
}

// computeShape plans a rectangular or slot bore. It shares the placement and
// depth gates of the round hole and refuses a head.
func computeShape(in Input) (Plan, error) {
	if in.Rect != nil && in.Slot != nil {
		return Plan{}, fmt.Errorf(`%w: a hole has one shape`, decaderr.ErrDegenerate)
	}
	if in.Head.Kind != HeadNone {
		return Plan{}, fmt.Errorf(`%w: a shaped hole takes no counterbore or countersink`, decaderr.ErrDegenerate)
	}
	switch {
	case in.Rect != nil:
		if !positive(in.Rect.Width) || !positive(in.Rect.Height) || !finite(in.Rect.Angle) {
			return Plan{}, fmt.Errorf(`%w: the rectangular hole needs a positive finite width and height and a finite angle`,
				decaderr.ErrDegenerate)
		}
	default:
		if !positive(in.Slot.Length) || !positive(in.Slot.Width) || !finite(in.Slot.Angle) {
			return Plan{}, fmt.Errorf(`%w: the slot needs a positive finite length and width and a finite angle`,
				decaderr.ErrDegenerate)
		}
		if in.Slot.Length <= in.Slot.Width {
			return Plan{}, fmt.Errorf(`%w: the slot length %g mm must exceed its width %g mm`,
				decaderr.ErrDegenerate, in.Slot.Length, in.Slot.Width)
		}
	}
	round := in
	round.Rect, round.Slot = nil, nil
	round.Diameter = 1
	plan, err := Compute(round)
	if err != nil {
		return Plan{}, err
	}
	plan.Bore.Radius = 0
	if in.Rect != nil {
		plan.Corners = rectCorners(plan, *in.Rect)
		return plan, nil
	}
	plan.SlotTool = slotTool(plan, *in.Slot)
	return plan, nil
}

// rectCorners returns the corners of r about the hole axis in plan's coordinates.
func rectCorners(plan Plan, r Rect) *[4][2]float64 {
	sin, cos := math.Sincos(r.Angle)
	hw, hh := r.Width/2, r.Height/2
	var corners [4][2]float64
	for i, c := range [4][2]float64{{-hw, -hh}, {hw, -hh}, {hw, hh}, {-hw, hh}} {
		corners[i] = [2]float64{plan.U + c[0]*cos - c[1]*sin, plan.V + c[0]*sin + c[1]*cos}
	}
	return &corners
}

// slotTool returns the stadium of sl about the hole axis in plan's coordinates.
func slotTool(plan Plan, sl Slot) *SlotTool {
	sin, cos := math.Sincos(sl.Angle)
	half := (sl.Length - sl.Width) / 2
	radius := sl.Width / 2
	at := func(along, across float64) [2]float64 {
		return [2]float64{plan.U + along*cos - across*sin, plan.V + along*sin + across*cos}
	}
	return &SlotTool{
		Start: at(-half, 0), End: at(half, 0),
		P0: at(-half, -radius), P1: at(half, -radius), P2: at(half, radius), P3: at(-half, radius),
	}
}

func finite(x float64) bool { return !math.IsInf(x, 0) && !math.IsNaN(x) }

func (p Plan) withCounterbore(in Input) (Plan, error) {
	h := in.Head
	if !positive(h.Diameter) || !positive(h.Depth) {
		return Plan{}, fmt.Errorf(`%w: a counterbore needs a positive finite diameter and depth`, decaderr.ErrDegenerate)
	}
	if h.Diameter <= in.Diameter {
		return Plan{}, fmt.Errorf(`%w: the counterbore diameter %g mm must exceed the hole diameter %g mm`,
			decaderr.ErrDegenerate, h.Diameter, in.Diameter)
	}
	if h.Depth >= p.Bore.Length {
		return Plan{}, fmt.Errorf(`%w: the counterbore depth %g mm must be less than the hole depth %g mm`,
			decaderr.ErrDegenerate, h.Depth, p.Bore.Length)
	}
	p.Counterbore = &Cylinder{Radius: h.Diameter / 2, Length: h.Depth, Margin: p.Bore.Margin}
	return p, nil
}

func (p Plan) withSink(in Input) (Plan, error) {
	h := in.Head
	if !positive(h.Diameter) || !positive(h.Angle) || h.Angle >= math.Pi {
		return Plan{}, fmt.Errorf(`%w: a countersink needs a positive diameter and an included angle in (0, π)`,
			decaderr.ErrDegenerate)
	}
	if h.Diameter <= in.Diameter {
		return Plan{}, fmt.Errorf(`%w: the countersink diameter %g mm must exceed the hole diameter %g mm`,
			decaderr.ErrDegenerate, h.Diameter, in.Diameter)
	}
	slope := math.Tan(h.Angle / 2)
	rim := h.Diameter / 2
	depth := (rim - in.Diameter/2) / slope
	if !positive(depth) || depth >= p.Bore.Length {
		return Plan{}, fmt.Errorf(`%w: the countersink is %g mm deep, which must be less than the hole depth %g mm`,
			decaderr.ErrDegenerate, depth, p.Bore.Length)
	}
	over := h.Diameter * sinkOvershoot
	centre := in.Frame.ToWorldUV(p.U, p.V)
	plane, err := r3.NewFrame(centre, in.Frame.U(), in.Outward)
	if err != nil {
		return Plan{}, fmt.Errorf(`%w: the countersink plane is degenerate: %s`, decaderr.ErrDegenerate, err)
	}
	p.Sink = &Sink{
		Plane: plane,
		Profile: [4][2]float64{
			{0, -depth},
			{in.Diameter / 2, -depth},
			{rim + over*slope, over},
			{0, over},
		},
	}
	return p, nil
}

func positive(x float64) bool { return x > 0 && !math.IsInf(x, 0) && !math.IsNaN(x) }

// ThroughMargin is how far, in millimetres, a through hole's tool runs past the
// far side of the body's bounds, so the tool's end never lies in a body face.
const ThroughMargin = 1

// Reach returns the distance from the plane through origin to the far side of
// the box [lo, hi] measured against outward, plus ThroughMargin and bound (the
// box's own bound, in millimetres). It is zero when the whole box lies on the
// outward side of the plane.
func Reach(lo, hi, origin, outward r3.Vec, bound float64) float64 {
	deepest := 0.0
	for _, x := range [2]float64{lo.X, hi.X} {
		for _, y := range [2]float64{lo.Y, hi.Y} {
			for _, z := range [2]float64{lo.Z, hi.Z} {
				deepest = max(deepest, origin.Sub(r3.Vec{X: x, Y: y, Z: z}).Dot(outward))
			}
		}
	}
	if deepest == 0 {
		return 0
	}
	return deepest + bound + ThroughMargin
}
