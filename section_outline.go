package decad

import (
	"context"
	"errors"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/meshslice"
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
