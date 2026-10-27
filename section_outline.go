package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/meshslice"
	"github.com/lestrrat-3d/decad/internal/outlineclear"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// SectionOutline is the closed outlines where a body's tessellation meets a
// plane (agent-docs/section-outline-design.md). It is a reading of the mesh,
// not of the exact solid, and it is not a sketch: nothing in it can enter a
// feature as an exact profile.
type SectionOutline struct {
	// Plane is the cutting plane. Its normal decides each outline's direction.
	Plane r3.Frame
	// Loops are the closed polylines, in world coordinates in millimetres, the
	// last point joining the first. An outer loop runs counter-clockwise seen
	// from the tip of the plane's normal and a hole runs clockwise.
	Loops [][]r3.Vec
	// OffSurface bounds how far every point of every loop lies from the body's
	// true boundary and from the plane: the mesh's proven displacement bound
	// plus the rounding of the cut's own arithmetic.
	OffSurface units.Value
}

// PolygonArea returns the area the loops enclose, an outer loop counted
// positive and a hole negative. It is the area of the polygons, so it differs
// from the true section's area by an amount OffSurface does not bound: the true
// section can differ in outline by far more than OffSurface where the plane
// meets the surface at a shallow angle.
func (s SectionOutline) PolygonArea() units.Value {
	normal := s.Plane.N()
	total := 0.0
	for _, loop := range s.Loops {
		var twice r3.Vec
		for i, p := range loop {
			twice = twice.Add(p.Cross(loop[(i+1)%len(loop)]))
		}
		total += twice.Dot(normal) / 2
	}
	return units.SquareMillimeters(total)
}

// SectionOutline cuts this solid's verified tessellation with plane and returns
// the closed outlines of the cut (agent-docs/section-outline-design.md). It
// reaches bodies [Body.Section] refuses, a mesh Boolean result among them, and
// states the result as a reading of the mesh: each point of each loop lies
// within OffSurface of the body's true boundary, and the loops are exact for
// the mesh up to rounding. A plane that misses the body gives no loop.
//
// A point of the mesh on the plane counts as on the side of the plane's
// normal, so a plane through a face reads as the cut just behind it along the
// normal. tol is the positive tessellation chord tolerance. Retired bodies
// remain readable. A nil body or context, or an invalid plane, is ErrDegenerate.
// A sheet or unsound solid is ErrNotSolid, a mesh whose boundary is not verified
// or whose cut does not close is ErrUnsupported, and a canceled context returns
// its error.
func (b *Body) SectionOutline(ctx context.Context, plane r3.Frame, tol units.Value) (SectionOutline, error) {
	if b == nil || b.n.Doc == nil || ctx == nil || !plane.IsValid() {
		return SectionOutline{}, fmt.Errorf(`%w: a section outline needs a body, context and valid plane`, ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return SectionOutline{}, err
	}
	if b.Kind() != BodySolid || !b.IsSolid() {
		return SectionOutline{}, fmt.Errorf(`%w: a section outline needs a sound solid`, ErrNotSolid)
	}
	mesh, err := tessellateContext(ctx, b, tol, VerifyBoundary)
	if err != nil {
		return SectionOutline{}, err
	}
	if !mesh.n.BoundaryOK || proofbound.IsNonFinite(mesh.n.Bound) || mesh.n.Bound < 0 {
		return SectionOutline{}, fmt.Errorf(`%w: a section outline needs a verified finite mesh boundary bound`, ErrUnsupported)
	}
	outlines, err := meshslice.Slice(ctx, mesh.n.Vertices, mesh.n.Triangles, plane.Origin(), plane.N())
	if errors.Is(err, meshslice.ErrOpen) {
		return SectionOutline{}, fmt.Errorf(`%w: %s`, ErrUnsupported, err)
	}
	if err != nil {
		return SectionOutline{}, err
	}
	scale := meshslice.MaxAbs(outlines)
	for _, v := range mesh.n.Vertices {
		scale = max(scale, v.X, v.Y, v.Z, -v.X, -v.Y, -v.Z)
	}
	o := plane.Origin()
	scale = max(scale, o.X, o.Y, o.Z, -o.X, -o.Y, -o.Z)
	out := SectionOutline{Plane: plane, OffSurface: units.Millimeters(sectionRoundingUp(mesh.n.Bound, scale))}
	for _, outline := range outlines {
		out.Loops = append(out.Loops, outline.Points)
	}
	return out, nil
}

// sectionRoundingUp returns a float no less than bound plus eight units of
// float64 rounding at the coordinate scale, the allowance for the arithmetic of
// one cut point: a product, a difference and a sum of coordinates at that
// scale.
func sectionRoundingUp(bound, scale float64) float64 {
	const eps = 1.0 / (1 << 52)
	sum := new(big.Rat).Add(proofarith.FloatRat(bound), proofarith.FloatRat(8*eps*scale))
	return proofbound.RatFloatUp(sum)
}

// sectionPlaneTolerance is how far two outlines' planes may differ and still
// be one plane: millimetres in the origin, and the same number in each
// component of the unit axes.
const sectionPlaneTolerance = 1e-9

// SectionClearance is the least distance between the material two
// [SectionOutline] values enclose in their shared plane, read from their
// polylines (agent-docs/section-outline-design.md §5).
type SectionClearance struct {
	// Distance is the polyline clearance in millimetres, zero when Overlaps is
	// true or the outlines touch. Its Bound is the sum of both outlines'
	// OffSurface, the rounding of the arithmetic and the plane tolerance, so
	// the true clearance of the sections lies within Value +/- Bound only to
	// the extent OffSurface does (see Lower).
	Distance Measurement
	// A is the point of the first outline's loops, and B the point of the
	// second's, that attain Distance. Both are in world millimetres. Where
	// the outlines cross or one lies inside the other they are one point.
	A, B r3.Vec
	// Overlaps is true when a loop of one outline properly crosses a loop of
	// the other, or a loop of one lies strictly inside the other's material.
	// Outlines that merely touch or coincide read Distance zero with Overlaps
	// false.
	Overlaps bool
}

// Lower returns the smallest clearance the reading allows: Distance less its
// Bound, and not below zero. A caller may read a certified gap from the
// outlines only when Lower is positive. Lower is zero when Overlaps is true.
func (c SectionClearance) Lower() units.Value {
	if c.Overlaps {
		return units.Millimeters(0)
	}
	return units.Millimeters(math.Max(0, c.Distance.Value.Base()-c.Distance.Bound.Base()))
}

// Clearance returns the least distance between the material region this
// outline encloses and the one other encloses, both seen in the one plane
// (agent-docs/section-outline-design.md §5). Outer loops enclose material and
// holes remove it. When the regions overlap the clearance is zero and
// Overlaps is true; otherwise it is the least distance between any two
// segments of the two outlines, found with bounding-box trees so that two
// outlines of thousands of segments take milliseconds.
//
// The Bound of the reading is the sum of both outlines' OffSurface plus the
// rounding of the arithmetic. It bounds how far the loop points lie from the
// true surface. It does not bound how far the true sections' outlines lie
// from the loops where the plane meets the surface at a shallow angle
// (section-outline design §2), and a gap is certified only when Lower is
// positive.
//
// The two outlines' planes must be the same frame: origin, U and V agree
// within 1e-9 (millimetres in the origin, and in each component of the
// unit axes). Otherwise, an outline without a loop, a loop of fewer than three
// points or a point that is not finite, the result is ErrDegenerate.
func (s SectionOutline) Clearance(other SectionOutline) (SectionClearance, error) {
	if !s.Plane.IsValid() || !s.Plane.Equal(other.Plane, sectionPlaneTolerance) {
		return SectionClearance{}, fmt.Errorf(`%w: section outlines must lie in the same plane`, ErrDegenerate)
	}
	for _, check := range []error{outlineclear.Check(`the first outline`, s.Loops), outlineclear.Check(`the second outline`, other.Loops)} {
		if check != nil {
			return SectionClearance{}, fmt.Errorf(`%w: %s`, ErrDegenerate, check)
		}
	}
	offA, offB := s.OffSurface.Base(), other.OffSurface.Base()
	if !(offA >= 0) || !(offB >= 0) || math.IsInf(offA, 0) || math.IsInf(offB, 0) {
		return SectionClearance{}, fmt.Errorf(`%w: an outline's OffSurface must be finite and not negative`, ErrDegenerate)
	}
	scale := math.Max(vecMaxAbs(s.Plane.Origin()), vecMaxAbs(other.Plane.Origin()))
	for _, loops := range [2][][]r3.Vec{s.Loops, other.Loops} {
		for _, loop := range loops {
			for _, p := range loop {
				scale = math.Max(scale, vecMaxAbs(p))
			}
		}
	}
	// ctx is not a parameter of a pure reading; the search is bounded by the
	// outlines' size and takes milliseconds at thousands of segments.
	res, err := outlineclear.Clearance(context.Background(), s.Plane.Origin(), s.Plane.U(), s.Plane.V(), s.Loops, other.Loops)
	if err != nil {
		return SectionClearance{}, err
	}
	bound := clearanceBound(offA, offB, scale)
	if math.IsInf(bound, 0) || math.IsNaN(bound) || math.IsInf(res.Distance, 0) || math.IsNaN(res.Distance) {
		return SectionClearance{}, fmt.Errorf(`%w: the clearance is not finite`, ErrNotFinite)
	}
	return SectionClearance{
		Distance: Measurement{Value: units.Millimeters(res.Distance), Exactness: Approximate, Bound: units.Millimeters(bound)},
		A:        res.A,
		B:        res.B,
		Overlaps: res.Overlap,
	}, nil
}

func vecMaxAbs(v r3.Vec) float64 {
	return math.Max(math.Abs(v.X), math.Max(math.Abs(v.Y), math.Abs(v.Z)))
}

// clearanceBound returns a float no less than offA + offB, thirty-two units of
// float64 rounding at the coordinate scale (the projection of a point, a
// difference, a product and a square root), and the plane tolerance's effect
// on a point at that scale: sectionPlaneTolerance in the origin and in each
// axis component, which moves a point by at most 4 tolerances times the scale
// plus one.
func clearanceBound(offA, offB, scale float64) float64 {
	const eps = 1.0 / (1 << 52)
	sum := new(big.Rat).Add(proofarith.FloatRat(offA), proofarith.FloatRat(offB))
	sum.Add(sum, proofarith.FloatRat(32*eps*scale))
	sum.Add(sum, proofarith.FloatRat(sectionPlaneTolerance*(4*scale+1)))
	return proofbound.RatFloatUp(sum)
}
