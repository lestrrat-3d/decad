package examples_test

import (
	"archive/zip"
	"bytes"
	"context"
	"fmt"
	"io"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

func Example_export_formats() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 2, 3)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve sketch: %s\n", err)
		return
	}
	body, err := decad.New().Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(4), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	tol := units.Millimeters(0.1)
	var out bytes.Buffer
	if err := export.STEP(context.Background(), &out, body, tol,
		export.WithSTEPName("box.step"),
		export.WithSTEPAuthor("Example"),
		export.WithSTEPOrganization("Example")); err != nil {
		fmt.Printf("failed to write STEP: %s\n", err)
		return
	}
	var stl bytes.Buffer
	if err := export.STL(context.Background(), &stl, body, tol); err != nil {
		fmt.Printf("failed to write STL: %s\n", err)
		return
	}
	var threeMF bytes.Buffer
	if err := export.ThreeMF(context.Background(), &threeMF, body, tol); err != nil {
		fmt.Printf("failed to write 3MF: %s\n", err)
		return
	}
	archive, err := zip.NewReader(bytes.NewReader(threeMF.Bytes()), int64(threeMF.Len()))
	if err != nil {
		fmt.Printf("failed to read 3MF: %s\n", err)
		return
	}
	modelFile, err := archive.File[2].Open()
	if err != nil {
		fmt.Printf("failed to open 3MF model: %s\n", err)
		return
	}
	model, err := io.ReadAll(modelFile)
	modelFile.Close()
	if err != nil {
		fmt.Printf("failed to read 3MF model: %s\n", err)
		return
	}
	fmt.Printf("AP214: %v, analytic faces: %d\n",
		strings.Contains(out.String(), "FILE_SCHEMA(('AUTOMOTIVE_DESIGN'))"),
		strings.Count(out.String(), "=ADVANCED_FACE("))
	fmt.Printf("STL facets: %d\n", strings.Count(stl.String(), "facet normal"))
	fmt.Printf("3MF parts: %d, triangles: %d\n", len(archive.File), strings.Count(string(model), "<triangle "))
	// Output:
	// AP214: true, analytic faces: 6
	// STL facets: 12
	// 3MF parts: 3, triangles: 12
}
