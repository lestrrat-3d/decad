package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

func Example_decad_point_cone_trim() {
	ctx := context.Background()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.YZ(), 8)
	if err != nil {
		fmt.Printf("failed to create profile plane: %s\n", err)
		return
	}
	profileSketch, err := w.CreateSketch(plane)
	if err != nil {
		fmt.Printf("failed to create profile sketch: %s\n", err)
		return
	}
	points := []*sketch.Point{
		profileSketch.CreatePoint(3, -1), profileSketch.CreatePoint(4, -1.5),
		profileSketch.CreatePoint(5, -1), profileSketch.CreatePoint(5, 1),
		profileSketch.CreatePoint(3, 1),
	}
	for _, point := range points {
		profileSketch.Fix(point)
	}
	if _, err := profileSketch.CreateFitSpline(points[0], points[1], points[2]); err != nil {
		fmt.Printf("failed to create fitted flank: %s\n", err)
		return
	}
	for i := 2; i < len(points); i++ {
		profileSketch.CreateLine(points[i], points[(i+1)%len(points)])
	}
	if _, err := profileSketch.Solve(ctx); err != nil {
		fmt.Printf("failed to solve profile: %s\n", err)
		return
	}
	coneSketch, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create cone sketch: %s\n", err)
		return
	}
	conePoints := []*sketch.Point{
		coneSketch.CreatePoint(-2, 0), coneSketch.CreatePoint(-2, 7),
		coneSketch.CreatePoint(5, 0),
	}
	for _, point := range conePoints {
		coneSketch.Fix(point)
	}
	for i := range conePoints {
		coneSketch.CreateLine(conePoints[i], conePoints[(i+1)%len(conePoints)])
	}
	if _, err := coneSketch.Solve(ctx); err != nil {
		fmt.Printf("failed to solve cone: %s\n", err)
		return
	}
	doc := decad.New()
	tooth, err := doc.LoftFromPoint(ctx, r3.Vec{}, profileSketch, profileSketch.Profiles()[0])
	if err != nil {
		fmt.Printf("failed to loft from point: %s\n", err)
		return
	}
	cone, err := doc.Revolve(coneSketch, coneSketch.Profiles()[0],
		decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{U: 1}},
		decad.FullRevolution{})
	if err != nil {
		fmt.Printf("failed to revolve cone: %s\n", err)
		return
	}
	trimmed, err := decad.Cut(ctx, tooth, cone)
	if err != nil {
		fmt.Printf("failed to trim loft: %s\n", err)
		return
	}
	mesh, err := trimmed.Tessellate(ctx, units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	if err != nil {
		fmt.Printf("failed to verify trim: %s\n", err)
		return
	}
	var hasCone, hasFlank bool
	for _, face := range trimmed.Faces() {
		switch face.Surface().(type) {
		case decad.Cone:
			hasCone = true
		case decad.NURBSSurface:
			hasFlank = true
		}
	}
	fmt.Printf("cone=%t flank=%t boundary=%t volume=%t\n",
		hasCone, hasFlank, mesh.BoundaryVerified(), mesh.VolumeVerified())
	// Output: cone=true flank=true boundary=true volume=true
}
