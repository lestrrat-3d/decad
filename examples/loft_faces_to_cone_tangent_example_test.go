package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Example_loft_faces_to_cone_tangent lofts two cap faces of a point
// loft, then a constructed last section tangent to a cone along one ruling.
func Example_loft_faces_to_cone_tangent() {
	ctx := context.Background()
	w := sketch.NewWorld()
	doc := decad.New()
	frame, err := r3.NewFrame(r3.NewVec(8, 0, 0), r3.NewVec(0.6, -0.8, 0), r3.NewVec(0, 0, 1))
	if err != nil {
		fmt.Println(err)
		return
	}
	plane, err := w.CreatePlaneFromFrame(frame)
	if err != nil {
		fmt.Println(err)
		return
	}
	s, err := w.CreateSketch(plane)
	if err != nil {
		fmt.Println(err)
		return
	}
	rect := s.CreateRectangle(-3.025, -0.025, -2.975, 0.025)
	s.Fix(rect.A)
	if _, err := s.Solve(ctx); err != nil {
		fmt.Println(err)
		return
	}
	point, err := decad.LoftFromPoint(ctx, doc, r3.Vec{}, s, s.Profiles()[0])
	if err != nil {
		fmt.Println(err)
		return
	}
	sheet := func(x float64) (*decad.Body, error) {
		ss, err := w.CreateSketch(w.XY())
		if err != nil {
			return nil, err
		}
		a, b := ss.CreatePoint(x, -20), ss.CreatePoint(x, 20)
		ss.Fix(a)
		ss.Fix(b)
		ss.CreateLine(a, b)
		if _, err := ss.Solve(ctx); err != nil {
			return nil, err
		}
		return decad.Extrude(ctx, doc, ss, ss.Chains()[0], decad.TwoSided{
			One: decad.DistanceSide{D: units.Millimeters(20)}, Two: decad.DistanceSide{D: units.Millimeters(20)}})
	}
	cutter, err := sheet(5)
	if err != nil {
		fmt.Println(err)
		return
	}
	first, err := decad.Split(ctx, point, cutter)
	if err != nil {
		fmt.Println(err)
		return
	}
	cutter, err = sheet(4.5)
	if err != nil {
		fmt.Println(err)
		return
	}
	second, err := decad.Split(ctx, first[0], cutter)
	if err != nil {
		fmt.Println(err)
		return
	}
	near := second[1]
	toe, err := decad.Faces(decad.FaceCreatedBy(decad.CapStart(near))).SelectFaces(near)
	if err != nil {
		fmt.Println(err)
		return
	}
	heel, err := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(near))).SelectFaces(near)
	if err != nil {
		fmt.Println(err)
		return
	}

	// The cone with apex 8 and slope 3/4: the revolved triangle (-4,0), (-4,16), (8,0).
	cs, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Println(err)
		return
	}
	corners := [][2]float64{{-4, 0}, {-4, 16}, {8, 0}}
	points := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		points[i] = cs.CreatePoint(c[0], c[1])
		cs.Fix(points[i])
	}
	for i := range points {
		cs.CreateLine(points[i], points[(i+1)%len(points)])
	}
	if _, err := cs.Solve(ctx); err != nil {
		fmt.Println(err)
		return
	}
	cone, err := decad.Revolve(ctx, doc, cs, cs.Profiles()[0],
		decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{U: 1}}, decad.FullRevolution{})
	if err != nil {
		fmt.Println(err)
		return
	}
	body, err := decad.LoftFacesToConeTangent(ctx,
		decad.ConeTangentEnd{Cone: cone, Radial: r3.NewVec(0, 1, 0)}, toe[0], heel[0])
	if err != nil {
		fmt.Println(err)
		return
	}
	mesh, err := body.Tessellate(ctx, units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	if err != nil {
		fmt.Println(err)
		return
	}
	ends, err := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(body))).SelectFaces(body)
	if err != nil {
		fmt.Println(err)
		return
	}
	fmt.Printf("solid %t; end faces %d; boundary %t; volume %t\n",
		body.IsSolid(), len(ends), mesh.BoundaryVerified(), mesh.VolumeVerified())
	// Output: solid true; end faces 1; boundary true; volume true
}
