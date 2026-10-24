package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/pointcone"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// ConeFromFace builds the full cone about body's axis that extends the
// selected face's straight meridian, using the nearest representable strict
// offset from that face's exact line. This need not be one float step: the
// rounded value may already be strictly sided, or the bound may be MaxFloat64.
// The construction guarantees separation only from the selected face line.
// Other blank and tool relations keep their own checks.
//
// body must be a live, unplaced, full Revolve about world X whose recorded
// section is a closed straight-sided walk of 4 to 128 whole segments with
// strictly positive X coordinates, nonnegative radii, no holes and one side of
// the axis. face must resolve under the implicit exactly-one to one conical
// side of that section. The cone is read from the recorded exact section data,
// never from the rounded public Cone tag. The side's meridian is the exact
// line x + k*r = C with k > 0, so the radius decreases as x rises. The result
// is the full Revolve of the triangle (far, 0), (far, R), (A, 0), where A and
// R are the float64 values nearest C and (C-far)/k that lie strictly on the
// empty side of the face. The cone wall and the face line share no point, so
// the relation between them is a strict sign that does not depend on how the
// platform rounds a fused multiply-add, and for identical recorded endpoint
// and far bits the strict gap is the same on every build. That establishes
// only the separation from the selected face line. Other blank and tool
// relations keep their own checks. The result is an ordinary Revolve body that
// depends on body and does not retire it.
//
// far is the cone's far plane as an axial coordinate. It must be a finite
// negative length (below X = 0, so the cone spans the origin) and strictly
// below both ends of the face. A nil ctx, a nil body, a body of no document,
// or a far that is not a finite length is ErrDegenerate. Selector failures are
// those of a ToFace stop. A retired body is ErrRetiredBody. A body or face
// outside the supported class (a plane or cylinder side, a section edge on the
// axis, a side whose meridian does not slope toward the axis as x rises), a
// far that is not below the face, a far that is not negative, or a cone whose
// float64 generator is not read back as an exact point cone is ErrUnsupported.
// A canceled ctx returns ctx.Err() unchanged. Every failure leaves the
// document and its provenance numbering unchanged.
func ConeFromFace(ctx context.Context, body *Body, face FaceSelector, far units.Value) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control ConeFromFace`, ErrDegenerate)
	}
	if body == nil || body.n.Doc == nil {
		return nil, fmt.Errorf(`%w: ConeFromFace names no body of a document`, ErrDegenerate)
	}
	farMM, err := far.In(units.Millimeter)
	if err != nil {
		return nil, fmt.Errorf(`%w: ConeFromFace far is not a finite length: %s`, ErrDegenerate, err)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d := documentHandle(body.n.Doc)
	if err := d.requireLive(body); err != nil {
		return nil, err
	}
	selected, err := selectImpliedOneFace(body, face, "ConeFromFace")
	if err != nil {
		return nil, err
	}
	spans, _, _, ok := revolveshell.RecordedSection(bodyNode(body))
	if !ok {
		return nil, fmt.Errorf(`%w: ConeFromFace needs an unplaced full Revolve of straight sides about X`,
			ErrUnsupported)
	}
	span, found := revolveSpanOfFace(spans, selected)
	if !found {
		return nil, fmt.Errorf(`%w: the selected face is not a straight side of the recorded Revolve section`,
			ErrUnsupported)
	}
	walk := make([][2]float64, len(spans))
	for i, s := range spans {
		walk[i] = [2]float64{s.Start.U, s.Start.V}
	}
	orientation := pointcone.WalkOrientation(walk)
	if orientation == 0 {
		return nil, fmt.Errorf(`%w: the recorded Revolve section has no orientation`, ErrUnsupported)
	}
	// The outward normal of a walk with orientation o and direction (dx, dr) is
	// o*(dr, -dx), and (1, k).n = o*(dr^2+dx^2)/dr, so the face's empty side has
	// the sign of o*dr.
	side := orientation
	if span.End.V < span.Start.V {
		side = -side
	}
	apex, radius, _, _, err := pointcone.OffsetMeridian(span.Start.U, span.Start.V,
		span.End.U, span.End.V, farMM, side)
	if err != nil {
		if errors.Is(err, pointcone.ErrNonFiniteMeridian) {
			return nil, fmt.Errorf(`%w: ConeFromFace input is not finite`, ErrDegenerate)
		}
		return nil, fmt.Errorf(`%w: the selected face has no offset cone below far: %s`, ErrUnsupported, err)
	}
	// revolveplan.PointConeSource reads only an apex above the origin and a far
	// plane below it.
	if apex <= 0 || farMM >= 0 {
		return nil, fmt.Errorf(`%w: the cone must span X = 0 between its far plane and its apex`, ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	cone, err := revolveMeridianCone(ctx, d, farMM, radius, apex)
	if err != nil {
		return nil, err
	}
	defer d.resultIn(d.bodyContainer(body))()
	if err := commitExactPointCone(ctx, d, cone); err != nil {
		return nil, err
	}
	return cone, nil
}

// commitExactPointCone registers a built cone in d only when revolveplan.PointConeSource
// reads it back as a cone about world X. The recognizer also needs a face
// classified Cone, and Revolve calls a generator that shallow a cylinder. The
// check runs before the body is registered, so a refusal leaves the document
// and its provenance numbering untouched.
func commitExactPointCone(ctx context.Context, d *Document, cone *Body) error {
	if limit, ok := revolveplan.PointConeSource(&cone.n); !ok || !limit.Axis.Canonical() {
		return fmt.Errorf(`%w: the offset cone is not an exact point cone`, ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	d.commit(cone)
	return nil
}

// revolveSpanOfFace returns the recorded section side that Revolve bound to
// face.
func revolveSpanOfFace(spans []revolveSectionSpan, face *Face) (revolveSectionSpan, bool) {
	for _, span := range spans {
		if span.Face == faceNode(face) {
			return span, true
		}
	}
	return revolveSectionSpan{}, false
}

// revolveMeridianCone revolves the triangle (far, 0), (far, radius), (apex, 0)
// about world X in a private sketch and returns the built body without
// registering it in d.
func revolveMeridianCone(ctx context.Context, d *Document, far, radius, apex float64) (*Body, error) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	if err != nil {
		return nil, err
	}
	corners := [3]planePoint{{U: far}, {U: far, V: radius}, {U: apex}}
	var points [3]*sketch.Point
	for i, corner := range corners {
		points[i] = s.CreatePoint(corner.U, corner.V)
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	profiles := s.Profiles()
	if len(profiles) != 1 {
		return nil, fmt.Errorf(`%w: the cone section is not one closed profile`, ErrUnsupported)
	}
	return revolveProfileBuild(ctx, d, s, profiles[0],
		SketchLine{Start: planePoint{}, End: planePoint{U: 1}}, FullRevolution{}, nil)
}
